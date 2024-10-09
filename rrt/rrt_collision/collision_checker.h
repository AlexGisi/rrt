//
// Created by Alex Gisi on 10/3/24.
//

#ifndef RRT_PLANNER_COLLISION_CHECKER_H
#define RRT_PLANNER_COLLISION_CHECKER_H

#include <rrt_collision/convex_polygon.h>

namespace rrt::collision {
class CollisionChecker {
public:
    static bool in_collision(const ConvexPolygon &shape1, const ConvexPolygon &shape2) {
        return shape1.collides(shape2);
    }
};
}

#endif //RRT_PLANNER_COLLISION_CHECKER_H
