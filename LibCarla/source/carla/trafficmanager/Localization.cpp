
#include "carla/trafficmanager/Localization.h"

namespace carla
{
namespace traffic_manager
{

namespace LocalizationConstants
{
static const float WAYPOINT_TIME_HORIZON = 5.0f;
static const float MINIMUM_HORIZON_LENGTH = 30.0f;
static const float TARGET_WAYPOINT_TIME_HORIZON = 0.5f;
static const float TARGET_WAYPOINT_HORIZON_LENGTH = 5.0f;
static const float MINIMUM_JUNCTION_LOOK_AHEAD = 10.0f;
static const float HIGHWAY_SPEED = 50 / 3.6f;
static const float MINIMUM_LANE_CHANGE_DISTANCE = 50.0f;
static const float MAXIMUM_LANE_OBSTACLE_CURVATURE = 0.6f;
static const uint64_t UNREGISTERED_ACTORS_SCAN_INTERVAL = 10;
} // namespace LocalizationConstants

using namespace LocalizationConstants;

Localization::Localization(ActorRegistry &actor_register,
                           InMemoryMap &local_map,
                           BufferMap &buffer_map,
                           Parameters &parameters,
                           TrackTraffic &track_traffic,
                           LocalizationResuts &localization_results,
                           cc::DebugHelper &debug_helper,
                           cc::World &world)
    : actor_register(actor_register), local_map(local_map), buffer_map(buffer_map),
      parameters(parameters), track_traffic(track_traffic), localization_results(localization_results),
      debug_helper(debug_helper), world(world)
{
}

Localization::~Localization()
{
}

void Localization::Localize()
{
    // ScanUnregisteredVehicles();

    const ActorPtr vehicle = actor_register.GetNextActor();
    const ActorId actor_id = vehicle->GetId();
    const cg::Transform vehicle_transform = vehicle->GetTransform();
    const cg::Location vehicle_location = vehicle_transform.location;
    const float vehicle_velocity = vehicle->GetVelocity().Length();
    const auto vehicle_ptr = boost::static_pointer_cast<cc::Vehicle>(vehicle);
    const float vehicle_half_length = vehicle_ptr->GetBoundingBox().extent.x;

    const float horizon_size = std::max(WAYPOINT_TIME_HORIZON * std::sqrt(vehicle_velocity * 10.0f),
                                        MINIMUM_HORIZON_LENGTH);

    if (buffer_map.find(actor_id) == buffer_map.end())
        buffer_map.emplace(actor_id, WaypointBuffer());

    WaypointBuffer &waypoint_buffer = buffer_map.at(actor_id);

    // Purge obsolete waypoints.
    waypoint_buffer.Purge(vehicle_transform, vehicle_half_length);

    // Re-initialize buffer if empty.
    if (waypoint_buffer.Empty())
    {
        SimpleWaypointPtr closest_waypoint = local_map.GetWaypointInVicinity(vehicle_location);
        if (closest_waypoint == nullptr)
        {
            closest_waypoint = local_map.GetWaypoint(vehicle_location);
        }
        waypoint_buffer.Seed(closest_waypoint);
    }

    SimpleWaypointPtr front_waypoint = waypoint_buffer.Front();
    const ChangeLaneInfo lane_change_info = parameters.GetForceLaneChange(vehicle);
    const bool force_lane_change = lane_change_info.change_lane;
    const bool lane_change_direction = lane_change_info.direction;

    // Assign lane change.
    if ((parameters.GetAutoLaneChange(vehicle) || force_lane_change) &&
        !front_waypoint->CheckJunction())
    {
        SimpleWaypointPtr change_over_point = AssignLaneChange(
            vehicle, force_lane_change, lane_change_direction);

        if (change_over_point != nullptr)
        {
            waypoint_buffer.Seed(change_over_point);
        }
    }

    // Populate buffer to horizon.
    waypoint_buffer.Extend(horizon_size);

    // Updating geodesic grid position for actor.
    const std::vector<SimpleWaypointPtr> sparse_buffer = waypoint_buffer.GetSparseWaypoints();
    track_traffic.UpdateGridPosition(actor_id, sparse_buffer);

    // Gathering results.

    front_waypoint = waypoint_buffer.Front();

    const float look_ahead_distance = std::max(2.0f * vehicle_velocity, MINIMUM_JUNCTION_LOOK_AHEAD);
    const SimpleWaypointPtr look_ahead_point = waypoint_buffer.TargetAtDistance(look_ahead_distance);

    // TODO: Take vehicle's length into consideration.
    const float target_point_distance = std::max(std::ceil(vehicle_velocity * TARGET_WAYPOINT_TIME_HORIZON),
                                                 TARGET_WAYPOINT_HORIZON_LENGTH);
    const SimpleWaypointPtr target_waypoint = waypoint_buffer.TargetAtDistance(target_point_distance);

    bool approaching_junction = false;
    if (front_waypoint->CheckJunction() ||
        (look_ahead_point->CheckJunction() && !(front_waypoint->CheckJunction())))
    {
        if (vehicle_ptr->GetSpeedLimit() > HIGHWAY_SPEED)
        {
            approaching_junction = waypoint_buffer.IsDivergenceInDistance(look_ahead_distance);
        }
        else
        {
            approaching_junction = true;
        }
    }

    const cg::Location target_location = target_waypoint->GetLocation();
    float dot_product = DeviationDotProduct(vehicle_transform, target_location);
    float cross_product = DeviationCrossProduct(vehicle_transform, target_location);
    float deviation = 1.0f - dot_product;
    if (cross_product < 0.0f)
    {
        deviation *= -1.0f;
    }

    ActorIdSet overlapping_actor_ids = track_traffic.GetOverlappingVehicles(actor_id);
    std::vector<ActorPtr> overlapping_actors = actor_register.GetActorReferences(
        std::vector<ActorId>(overlapping_actor_ids.begin(), overlapping_actor_ids.end())
    );

    std::vector<SparseActorBuffer> sparse_actor_buffers;
    for (ActorPtr& actor_ptr: overlapping_actors)
    {
        sparse_actor_buffers.push_back({actor_ptr,
                                        buffer_map.at(actor_ptr->GetId()).GetSparseWaypoints()});
    }

    localization_results.actor = vehicle;
    localization_results.approaching_true_junction = approaching_junction;
    localization_results.closest_waypoint = front_waypoint;
    localization_results.collision_candidates = sparse_actor_buffers;
    localization_results.deviation = deviation;
    localization_results.look_ahead_point = look_ahead_point;
}

void Localization::DrawBuffer(std::vector<SimpleWaypointPtr> &buffer)
{

    for (uint64_t i = 0u; i < buffer.size() && i < 5; ++i)
    {
        debug_helper.DrawPoint(buffer.at(i)->GetLocation(), 0.1f, {255u, 0u, 0u}, 0.5f);
    }
}

SimpleWaypointPtr Localization::AssignLaneChange(ActorPtr vehicle, bool force, bool direction)
{

    const ActorId actor_id = vehicle->GetId();
    const cg::Location vehicle_location = vehicle->GetLocation();
    const float vehicle_velocity = vehicle->GetVelocity().Length();
    const float speed_limit = boost::static_pointer_cast<cc::Vehicle>(vehicle)->GetSpeedLimit();

    SimpleWaypointPtr current_waypoint = buffer_map.find(actor_id)->second.Front();

    bool need_to_change_lane = false;
    const auto left_waypoint = current_waypoint->GetLeftWaypoint();
    const auto right_waypoint = current_waypoint->GetRightWaypoint();

    if (!force && current_waypoint != nullptr)
    {

        const auto blocking_vehicles = track_traffic.GetOverlappingVehicles(actor_id);

        bool abort_lane_change = false;
        for (auto i = blocking_vehicles.begin();
             i != blocking_vehicles.end() && !abort_lane_change;
             ++i)
        {
            const ActorId &other_vehicle_id = *i;

            // This is totally ignoring unregistered actors during lane changes.
            // Need to fix this. Only a temporary solution.
            if (buffer_map.find(other_vehicle_id) != buffer_map.end())
            {
                WaypointBuffer &other_buffer = buffer_map.at(other_vehicle_id);

                if (!other_buffer.Empty())
                {
                    const SimpleWaypointPtr &other_current_waypoint = other_buffer.Front();
                    const cg::Location other_location = other_current_waypoint->GetLocation();

                    bool distant_lane_availability = false;
                    const auto other_neighbouring_lanes = {
                        other_current_waypoint->GetLeftWaypoint(),
                        other_current_waypoint->GetRightWaypoint()};
                    if (other_current_waypoint != nullptr)
                    {
                        for (auto &candidate_lane_wp : other_neighbouring_lanes)
                        {
                            if (candidate_lane_wp != nullptr &&
                                track_traffic.GetPassingVehicles(candidate_lane_wp->GetId()).size() == 0 &&
                                speed_limit < HIGHWAY_SPEED)
                            {
                                distant_lane_availability = true;
                            }
                        }

                        const cg::Vector3D reference_heading = current_waypoint->GetForwardVector();
                        const cg::Vector3D other_heading = other_current_waypoint->GetForwardVector();

                        if (other_vehicle_id != actor_id &&
                            !current_waypoint->CheckJunction() &&
                            !other_current_waypoint->CheckJunction() &&
                            cg::Math::Dot(reference_heading, other_heading) > MAXIMUM_LANE_OBSTACLE_CURVATURE)
                        {

                            const float squared_vehicle_distance = cg::Math::DistanceSquared(other_location, vehicle_location);
                            const float deviation_dot = DeviationDotProduct(vehicle->GetTransform(), other_location);

                            if (deviation_dot > 0.0f)
                            {

                                if (distant_lane_availability &&
                                    squared_vehicle_distance > std::pow(MINIMUM_LANE_CHANGE_DISTANCE, 2))
                                {

                                    need_to_change_lane = true;
                                }
                                else if (squared_vehicle_distance < std::pow(MINIMUM_LANE_CHANGE_DISTANCE, 2))
                                {

                                    need_to_change_lane = false;
                                    abort_lane_change = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {

        need_to_change_lane = true;
    }

    const float change_over_distance = std::max(2.0f * vehicle_velocity, 10.0f);
    bool possible_to_lane_change = false;
    SimpleWaypointPtr change_over_point = nullptr;

    if (need_to_change_lane)
    {

        std::vector<SimpleWaypointPtr> candidate_points;
        if (force)
        {
            if (direction && left_waypoint != nullptr)
            {
                candidate_points.push_back(left_waypoint);
            }
            else if (!direction && right_waypoint != nullptr)
            {
                candidate_points.push_back(right_waypoint);
            }
        }
        else
        {
            candidate_points.push_back(left_waypoint);
            candidate_points.push_back(right_waypoint);
        }

        for (auto target_lane_wp : candidate_points)
        {
            if (!force &&
                !possible_to_lane_change &&
                target_lane_wp != nullptr &&
                track_traffic.GetPassingVehicles(target_lane_wp->GetId()).size() == 0)
            {

                possible_to_lane_change = true;
                change_over_point = target_lane_wp;
            }
            else if (force)
            {

                possible_to_lane_change = true;
                change_over_point = target_lane_wp;
            }
        }
    }

    if (need_to_change_lane && possible_to_lane_change)
    {
        const auto starting_point = change_over_point;
        while (change_over_point->DistanceSquared(starting_point) < std::pow(change_over_distance, 2) &&
               !change_over_point->CheckJunction())
        {
            change_over_point = change_over_point->GetNextWaypoint()[0];
        }
        return change_over_point;
    }
    else
    {
        return nullptr;
    }
}

} // namespace traffic_manager
} // namespace carla
