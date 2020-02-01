
#pragma once

#include <unordered_map>

#include "carla/client/DebugHelper.h"
#include "carla/client/World.h"

#include "carla/trafficmanager/ActorRegistry.h"
#include "carla/trafficmanager/Buffer.h"
#include "carla/trafficmanager/InMemoryMap.h"
#include "carla/trafficmanager/Parameters.h"
#include "carla/rpc/ActorId.h"

namespace carla
{
namespace traffic_manager
{

namespace cc = carla::client;

using BufferMap = std::unordered_map<ActorId, Buffer>;

class Localization
{

private:
    /// Reference to local map-cache object.
    InMemoryMap &local_map;
    /// Structures to hold waypoint buffers for all registered vehicles.
    BufferMap buffer_map;
    /// Runtime parameterization object.
    Parameters &parameters;
    /// Reference to Carla's debug helper object.
    cc::DebugHelper &debug_helper;
    /// Carla world object;
    cc::World& world;


public:
    Localization(ActorRegistry &actor_register,
                 InMemoryMap &local_map,
                 Parameters &parameters,
                 cc::DebugHelper &debug_helper,
                 cc::World &world);

    ~Localization();
};

} // namespace traffic_manager
} // namespace carla
