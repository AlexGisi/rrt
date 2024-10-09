//
// Created by Alex Gisi on 10/3/24.
//

#include <cmath>
#include <sstream>
#include <rrt_models/dubins_car.h>
#include <rrt_logging/util.h>
#include <rrt_collision/vector_2d.h>

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

DubinsState DubinsState::interpolate(const DubinsState &state0, const DubinsState &state1, float s) {
    float q = 1 - s;
    float px = state0.px * q + state1.px * s;
    float py = state0.py * q + state1.py * s;
    float theta = state0.theta * q + s * state1.theta;
    float v = state0.v * q + s * state1.v;
    float phi = state0.phi * q + s * state1.phi;
    return {px, py, theta, v, phi};
}


bool DubinsState::violates_constraints() const {
    return v < v_min || v > v_max || phi < phi_min || phi > phi_max;
}

DubinsState DubinsState::after_brake() const {
    if (v <= 0.0f) {
        return *this;
    }

    // Calculate stopping distance using kinematic equation:
    // v^2 = v0^2 + 2a * s => s = - (v0^2) / (2a)
    float stopping_distance = (v * v) / (-2.0f * -10.0f);

    float new_px = px + stopping_distance * std::cos(theta);
    float new_py = py + stopping_distance * std::sin(theta);

    DubinsState state_final(new_px, new_py, theta, 0.0f, 0.0f);

    return state_final;
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

rrt::collision::ConvexPolygon DubinsCar::get_polygon() const {
    rrt::collision::Vector2D origin_to_com(static_cast<float>(height * 0.8) / 2, 0);
    rrt::collision::Vector2D origin_xy(state.px, state.py);

    auto rect = rrt::collision::ConvexPolygon::create_rectangle(width, height);
    rect.translate(origin_to_com);  // Now origin of car is at origin coordinates.
    rect.rotate(state.theta);  // Rotate around car's origin.
    rect.translate(origin_xy);

    return rect;
}

bool DubinsCar::in_collision(const std::vector<collision::ConvexPolygon>& polygons) const {
    for (auto const &polygon : polygons) {
        if (get_polygon().collides(polygon)) {
            return true;
        }
    }
    return false;
}


std::string DubinsCar::log_header() {
    return {"x0,y0,x1,y1,x2,y2,x3,y3,width,height"};
}

std::string DubinsCar::log() const {
    const auto r = get_polygon();
    auto s = r.log() + "," + std::to_string(width) + "," + std::to_string(height);
    return s;
}
}
