//
// Created by Alex Gisi on 10/3/24.
//

#include <cmath>
#include <stdexcept>
#include <sstream>
#include <rrt_models/dubins_car.h>
#include <rrt_logging/util.h>

namespace rrt::models {

float DubinsState::v_max = 5;
float DubinsState::v_min = -5;
float DubinsState::phi_max = M_PI / 3;
float DubinsState::phi_min = -M_PI / 3;

float DubinsCommand::a_min = -10;
float DubinsCommand::a_max = 10;
float DubinsCommand::psi_min = -3;
float DubinsCommand::psi_max = 3;

DubinsState DubinsState::sample(float px_lower, float px_upper, float py_lower, float py_upper) {
    float _px = rrt::util::rand(px_lower, px_upper);
    float _py = rrt::util::rand(py_lower, py_upper);
    float _theta = rrt::util::rand(0, 2*M_PI);
    float _v = rrt::util::rand(v_min, v_max);
    float _phi = rrt::util::rand(phi_min, phi_max);

    return {_px, _py, _theta, _v, _phi };
}

float DubinsState::weighted_distance(DubinsState state0, DubinsState state1) {
    float w_x = 0.1;
    float w_y = 0.1;
    float w_theta = 0.1;
    float w_v = 0.1;
    float w_phi = 0.1;

    float dx = std::pow(state0.px - state1.px, 2);
    float dy = std::pow(state0.py - state1.py, 2);
    float dtheta = std::pow(rrt::util::geodesic_distance(state0.theta, state1.theta), 2);
    float dv = std::pow(state0.v - state1.v, 2);
    float dphi = std::pow(state0.phi - state1.phi, 2);

    float d = w_x * dx + w_y * dy + w_theta * dtheta + w_v * dv + w_phi * dphi;
    return std::sqrt(d);
}

float DubinsState::euclidean_distance(DubinsState state0, DubinsState state1) {
    float d = std::pow(state0.px - state1.px, 2) + std::pow(state0.py - state1.py, 2);
    return std::sqrt(d);
}

bool DubinsState::violates_constraints() const {
    return v < v_min || v > v_max || phi < phi_min || phi > phi_max;
}

std::string DubinsState::log_header() {
    return {"x,y,theta,v,phi"};
}

std::string DubinsState::log() {
    return rrt::util::comma_join<float>({px, py, theta, v, phi});
}

void DubinsCar::step(DubinsCommand command, float dt) {
    DubinsState state1;

    state1.v = state.v + command.a * dt;
    state1.phi = state.phi + command.psi * dt;

    state1.px = state.px + (state1.v * cos(state.theta)) * dt;
    state1.py = state.py + (state1.v * sin(state.theta)) * dt;
    state1.theta = state.theta + ((state1.v / l) * tan(state1.phi)) * dt;

    state1.theta = rrt::util::wrap_angle(state1.theta);
    state = state1;
}

void DubinsCar::reset() {
    state = DubinsState();
}
}
