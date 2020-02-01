
#pragma once

#include <cmath>
#include <deque>
#include <memory>
#include <mutex>

#include "carla/geom/Transform.h"

#include "carla/trafficmanager/Util.h"
#include "carla/trafficmanager/SimpleWaypoint.h"

namespace carla
{
namespace traffic_manager
{

namespace cg = carla::geom;

using SimpleWaypointPtr = std::shared_ptr<SimpleWaypoint>;

class Buffer
{

private:
    std::deque<SimpleWaypointPtr> queue;
    std::mutex data_modification_mutex;

public:
    Buffer(/* args */);
    ~Buffer();

    SimpleWaypointPtr Front();
    SimpleWaypointPtr Back();
    bool Empty();
    uint Size();
    SimpleWaypointPtr TargetAtDistance(float distance);
    void Purge(cg::Transform transform, float offset);
    void Seed(SimpleWaypointPtr waypoint);
    std::vector<SimpleWaypointPtr> GetSparseWaypoint(uint number_of_cuts);
    void Extend(float distance);
};

Buffer::Buffer(/* args */)
{
}

Buffer::~Buffer()
{
}

SimpleWaypointPtr Buffer::Front()
{
    std::lock_guard<std::mutex> lock(data_modification_mutex);
    return queue.front();
}

SimpleWaypointPtr Buffer::Back()
{
    std::lock_guard<std::mutex> lock(data_modification_mutex);
    return queue.back();
}

uint Buffer::Size()
{
    std::lock_guard<std::mutex> lock(data_modification_mutex);
    return queue.size();
}

SimpleWaypointPtr Buffer::TargetAtDistance(float distance)
{
    std::lock_guard<std::mutex> lock(data_modification_mutex);
    SimpleWaypointPtr target_waypoint = queue.front();
    for (uint j = 0u;
         (j < queue.size()) &&
         (queue.front()->DistanceSquared(target_waypoint) < std::pow(distance, 2));
         ++j)
    {
        target_waypoint = queue.at(j);
    }
    return target_waypoint;
}

void Buffer::Purge(cg::Transform transform, float offset)
{
    std::lock_guard<std::mutex> lock(data_modification_mutex);
    if (!queue.empty())
    {
        float dot_product = DeviationDotProduct(transform, queue.front()->GetLocation(),
                                                true, offset);

        while (dot_product <= 0.0f && !queue.empty())
        {

            queue.pop_front();
            if (!queue.empty())
            {
                dot_product = DeviationDotProduct(transform, queue.front()->GetLocation(),
                                                  true, offset);
            }
        }
    }
}

void Buffer::Seed(SimpleWaypointPtr waypoint)
{
    std::lock_guard<std::mutex> lock(data_modification_mutex);
    queue.clear();
    queue.push_back(waypoint);
}

std::vector<SimpleWaypointPtr> Buffer::GetSparseWaypoint(uint number_of_cuts = 6u)
{
    std::lock_guard<std::mutex> lock(data_modification_mutex);
    uint queue_size = queue.size();
    uint step_size = static_cast<int>(std::floor(queue_size / number_of_cuts));
    std::vector<SimpleWaypointPtr> sparse_waypoints;
    for (uint i = 0u; i < number_of_cuts; ++i)
    {
        sparse_waypoints.push_back(queue.at(i * step_size));
    }
    sparse_waypoints.push_back(queue.back());
    return sparse_waypoints;
}

void Buffer::Extend(float distance)
{
    std::lock_guard<std::mutex> lock(data_modification_mutex);
    while (queue.back()->DistanceSquared(queue.front()) <= std::pow(distance, 2))
    {
        std::vector<SimpleWaypointPtr> next_waypoints = queue.back()->GetNextWaypoint();
        uint64_t selection_index = 0u;
        // Pseudo-randomized path selection if found more than one choice.
        if (next_waypoints.size() > 1)
        {
            selection_index = static_cast<uint64_t>(rand()) % next_waypoints.size();
        }
        SimpleWaypointPtr next_wp = next_waypoints.at(selection_index);
        if (next_wp == nullptr)
        {
            for (auto &wp : next_waypoints)
            {
                if (wp != nullptr)
                {
                    next_wp = wp;
                    break;
                }
            }
        }
        queue.push_back(next_wp);
    }
}

} // namespace traffic_manager
} // namespace carla
