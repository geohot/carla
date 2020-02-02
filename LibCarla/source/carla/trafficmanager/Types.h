
#pragma once

#include <memory>

#include "carla/client/Actor.h"
#include "carla/Memory.h"
#include "carla/rpc/ActorId.h"

#include "carla/trafficmanager/SimpleWaypoint.h"

namespace carla
{
namespace traffic_manager
{

namespace cc = carla::client;

using Actor = carla::SharedPtr<cc::Actor>;
using ActorId = carla::ActorId;
using SimpleWaypointPtr = std::shared_ptr<SimpleWaypointPtr>;

struct SparseActorBuffer
{
    Actor actor;
    std::vector<SimpleWaypointPtr> sparse_buffer;
};

struct LocalizationResuts
{
    Actor actor;
    float deviation;
    bool approaching_true_junction;
    SimpleWaypointPtr closest_waypoint;
    SimpleWaypointPtr look_ahead_point;
    std::vector<SparseActorBuffer> collision_candidates;
};


}
} // namespace carla
