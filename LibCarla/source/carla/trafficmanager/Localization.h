
#pragma once

#include <unordered_map>

#include "carla/client/DebugHelper.h"
#include "carla/client/World.h"
#include "carla/rpc/ActorId.h"

#include "carla/trafficmanager/ActorRegistry.h"
#include "carla/trafficmanager/InMemoryMap.h"
#include "carla/trafficmanager/Parameters.h"
#include "carla/trafficmanager/TrackTraffic.h"
#include "carla/trafficmanager/Types.h"
#include "carla/trafficmanager/WaypointBuffer.h"

namespace carla
{
namespace traffic_manager
{

namespace cc = carla::client;

using BufferMap = std::unordered_map<ActorId, WaypointBuffer>;

class Localization
{

private:
    /// Structure holding registered actors.
    ActorRegistry &actor_register;
    /// Reference to local map-cache object.
    InMemoryMap &local_map;
    /// Structures to hold waypoint buffers for all registered vehicles.
    BufferMap& buffer_map;
    /// Runtime parameterization object.
    Parameters &parameters;
    /// Object for tracking paths of the traffic vehicles.
    TrackTraffic& track_traffic;
    /// Structure to output results of localizations.
    LocalizationResuts& localization_results;
    /// Reference to Carla's debug helper object.
    cc::DebugHelper &debug_helper;
    /// Carla world object;
    cc::World& world;
    /// A structure used to keep track of actors spawned outside of traffic
    /// manager.
    // TODO: Handle unregistered actors
    // std::unordered_map<ActorId, Actor>& unregistered_actors;

    /// A simple method used to draw waypoint buffer ahead of a vehicle.
    void DrawBuffer(std::vector<SimpleWaypointPtr> &buffer);
    /// Method to determine lane change and obtain target lane waypoint.
    SimpleWaypointPtr AssignLaneChange(ActorPtr vehicle, bool force, bool direction);
    /// Method to scan for unregistered actors and update their grid positioning.
    // TODO: Handle unregistered actors
    // void ScanUnregisteredVehicles();
    /// Main compute logic of class.
    void Localize();

public:
    Localization(ActorRegistry &actor_register,
                 InMemoryMap &local_map,
                 BufferMap& buffer_map,
                 Parameters &parameters,
                 TrackTraffic& track_traffic,
                 LocalizationResuts& localization_results,
                 cc::DebugHelper &debug_helper,
                 cc::World &world);

    ~Localization();
};

} // namespace traffic_manager
} // namespace carla
