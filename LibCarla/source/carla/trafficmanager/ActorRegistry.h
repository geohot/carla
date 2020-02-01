
#pragma once

#include <mutex>
#include <unordered_map>

#include "carla/client/Actor.h"
#include "carla/Memory.h"

#include "carla/trafficmanager/Exception.h"

namespace carla
{
namespace traffic_manager
{

namespace cc = carla::client;
using ActorPtr = carla::SharedPtr<cc::Actor>;

/// Class to maintian the collection of registered actors.
class ActorRegistry
{

private:
    using ActorMap = std::unordered_map<ActorId, ActorPtr>;
    ActorMap actor_map;
    ActorMap::iterator looping_iterator;
    std::mutex data_modification_mutex;

public:
    ActorRegistry();
    ~ActorRegistry();

    /// Method to add an actor to the register.
    void RegisterActor(const ActorPtr &actor);
    /// Method to remove an actor from the register.
    void UnregisterActor(const ActorId &actor_id);
    /// Method to retrieve actors from the register in cycle.
    ActorPtr GetNextActor();
    /// Method to retrieve the number of registered actors in the register.
    uint GetNumberOfRegisteredActors();
};

ActorRegistry::ActorRegistry() : looping_iterator(actor_map.begin()) {}

ActorRegistry::~ActorRegistry() {}

void ActorRegistry::RegisterActor(const ActorPtr &actor)
{
    std::lock_guard<std::mutex> lock(data_modification_mutex);
    ActorId actor_id = actor->GetId();
    if (actor_map.find(actor_id) == actor_map.end())
    {
        actor_map.insert({actor_id, actor});
    } else {
        throw DuplicateActorException();
    }
}

void ActorRegistry::UnregisterActor(const ActorId &actor_id)
{
    std::lock_guard<std::mutex> lock(data_modification_mutex);

    ActorId current_id = 0u;
    ActorMap::iterator next_setp;
    if (looping_iterator != actor_map.end())
    {
        current_id = looping_iterator->first;
        next_setp = looping_iterator;
        ++next_setp;
        if (next_setp == actor_map.end())
            next_setp = actor_map.begin();
    }

    if (actor_map.find(actor_id) != actor_map.end())
    {
        actor_map.erase(actor_id);
    } else {
        throw ActorNotFoundException();
    }

    if (current_id == actor_id && looping_iterator != actor_map.end())
        looping_iterator = next_setp;
}

ActorPtr ActorRegistry::GetNextActor()
{
    std::lock_guard<std::mutex> lock(data_modification_mutex);

    ActorPtr current_actor;
    if (actor_map.size() > 0)
    {
        if (looping_iterator == actor_map.end())
            looping_iterator = actor_map.begin();
        current_actor = looping_iterator->second;
        ++looping_iterator;
    } else {
        throw ActorNotFoundException();
    }

    return current_actor;
}

uint ActorRegistry::GetNumberOfRegisteredActors()
{
    std::lock_guard<std::mutex> lock(data_modification_mutex);

    return actor_map.size();
}

} // namespace traffic_manager
} // namespace carla
