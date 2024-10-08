//
// Created by Alex Gisi on 10/3/24.
//

#include <cmath>
#include <stdexcept>
#include <sstream>
#include <rrt_models/dubins_car.h>
#include <rrt_logging/util.h>

namespace rrt::models {

bool DubinsState::violates_constraints() const {
    float v_max = 5;
    float v_min = -5;
    float phi_max = M_PI / 3;
    float phi_min = -M_PI / 3;

    return v < v_min || v > v_max || phi < phi_min || phi > phi_max;
}

std::string DubinsState::log_header() {
    return {"x,y,theta,v,phi"};
}

std::string DubinsState::log() {
    return rrt::util::comma_join<float>({px, py, theta, v, phi});
}

void DubinsCar::step(DubinsCommand command, float dt) {
    if (dt > 0.1) {
        throw std::runtime_error("Timestep must be less than 0.1, was " + std::to_string(dt));
    }

    DubinsState state1;
    state1.px = state.px + (state.v * cos(state.theta)) * dt;
    state1.py = state.py + (state.v * sin(state.theta)) * dt;
    state1.theta = state.theta + ((state.v / l) * tan(state.phi)) * dt;
    state1.v = state.v + command.a * dt;
    state1.phi = state.phi + command.psi * dt;

    state = state1;
}

void DubinsCar::reset() {
    state = DubinsState();
}
}
