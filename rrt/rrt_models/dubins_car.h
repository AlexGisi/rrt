//
// Created by Alex Gisi on 10/3/24.
//

#ifndef RRT_DUBINS_CAR_H
#define RRT_DUBINS_CAR_H

#include <string>
#include <rrt_logging/util.h>

namespace rrt::models {
class DubinsState {
public:
    DubinsState() = default;
    DubinsState(float _px, float _py, float _theta, float _v, float _phi) : px(_px), py(_py), theta(_theta), v(_v), phi(_phi) {}

    bool violates_constraints() const;

    std::string log_header();
    std::string log();

    float px = 0;
    float py = 0;
    float theta = 0;
    float v = 0;
    float phi = 0;
};

class DubinsCommand {
public:
    DubinsCommand(float _a, float _psi) : a(_a), psi(_psi) {};

    static DubinsCommand sample() {
        float a = rrt::util::rand(-10, 10);
        float psi = rrt::util::rand(-3, 3);
        return {a, psi};
    }

    std::string log_header() {
        return {"a,psi"};
    }
    std::string log() {
        return rrt::util::comma_join<float>({a, psi});
    }

    float a;
    float psi;
};

class DubinsCar {
public:
    explicit DubinsCar(float _l) : l(_l) {}
    explicit DubinsCar(float _l, DubinsState state_initial) : l(_l), state(state_initial) {}

    void set_state(DubinsState _state) { state = _state; };
    DubinsState get_state() const { return state; };

    void step(DubinsCommand command, float dt=0.05);
    void reset();

private:
    float l;
    DubinsState state;
};
}


#endif //RRT_DUBINS_CAR_H
