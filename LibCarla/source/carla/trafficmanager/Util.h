
#include "carla/geom/Transform.h"
#include "carla/geom/Location.h"

namespace carla
{
namespace traffic_manager
{

namespace cg = carla::geom;

float DeviationCrossProduct(cg::Transform transform, cg::Location target_location)
{

    cg::Vector3D heading_vector = transform.GetForwardVector();
    heading_vector.z = 0.0f;
    heading_vector = heading_vector.MakeUnitVector();
    cg::Location next_vector = target_location - transform.location;
    next_vector.z = 0.0f;
    if (next_vector.Length() > 2.0f * std::numeric_limits<float>::epsilon())
    {
        next_vector = next_vector.MakeUnitVector();
        const float cross_z = heading_vector.x * next_vector.y - heading_vector.y * next_vector.x;
        return cross_z;
    }
    else
    {
        return 0.0f;
    }
}

float DeviationDotProduct(cg::Transform transform, cg::Location target_location,
                          bool rear_offset=false, float offset=0.0f)
{

    cg::Vector3D heading_vector = transform.GetForwardVector();
    heading_vector.z = 0.0f;
    heading_vector = heading_vector.MakeUnitVector();
    cg::Location next_vector;

    if (!rear_offset)
    {
        next_vector = target_location - transform.location;
    }
    else
    {
        next_vector = target_location - (cg::Location(-1 * offset * heading_vector) + transform.location);
    }

    next_vector.z = 0.0f;
    if (next_vector.Length() > 2.0f * std::numeric_limits<float>::epsilon())
    {
        next_vector = next_vector.MakeUnitVector();
        const float dot_product = cg::Math::Dot(next_vector, heading_vector);
        return dot_product;
    }
    else
    {
        return 0.0f;
    }
}
} // namespace traffic_manager
} // namespace carla