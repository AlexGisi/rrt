//
// Created by agisi2 on 10/28/2024.
//

#ifndef DUBINS_H
#define DUBINS_H

#include <models/model.h>
#include <collision/convex_polygon.h>

namespace rrt::models {

class DubinsState final : public State<DubinsState, 5> {
public:
    static constexpr BoundArr state_bounds = {
        std::nullopt,  // x
        std::nullopt,  // y
        util::ClosedInterval(0, 2*M_PI), // theta
        util::ClosedInterval(-5, 5), // velocity
        util::ClosedInterval(-M_PI / 3, M_PI / 3)  // steer angle
    };

    DubinsState() : State(state_bounds) {}
    explicit DubinsState(const std::array<double, 5> &x) : State(x, state_bounds) {}

    void randomize(const util::ClosedInterval xi, const util::ClosedInterval yi) {
        State::randomize();
        state_[0] = xi.sample();
        state_[1] = yi.sample();
    }

    std::string log_header() const override {  // NOLINT
        return "x,y,theta,v,phi";
    }
};

class DubinsCommand final : public Command<2> {
public:
    static constexpr BoundArr command_bounds = {
        util::ClosedInterval(-10.0, 10.0), // acceleration
        util::ClosedInterval(-3.0, 3.0) // steer
    };

    DubinsCommand() : Command(command_bounds) {}
    explicit DubinsCommand(const std::array<double, 2> &u) : Command(u, command_bounds) {}

    std::string log_header() const override {  // NOLINT
        return "a,psi";
    }
};

class DubinsModel final : public Model<DubinsState, DubinsCommand> {
public:
    DubinsModel() = delete;

    explicit DubinsModel(const float l, const float width, const float length)
        : Model(DubinsState()), l_(l), width_(width), length_(length) {};

    void step(const DubinsCommand& cmd, const float dt) override {
        const auto u = cmd.u();
        const auto x_old = state_.state();
        auto x_new = x_old;

        x_new[3] = x_old[3] + u[0] * dt;  // velocity
        x_new[4] = x_old[4] + u[1] * dt;  // steer

        x_new[0] = x_old[0] + (x_new[3] * cos(x_old[2])) * dt;
        x_new[1] = x_old[1] + (x_new[3] * sin(x_old[2])) * dt;
        x_new[2] = util::wrap_angle(x_old[2] + ((x_new[3] / l_) * tan(x_new[4])) * dt);

        state_ = DubinsState(x_new);
    }

    collision::ConvexPolygon polygon() const override {
        const collision::Vector2D origin_to_com(static_cast<float>(length_ * 0.8) / 2, 0);
        const collision::Vector2D origin(state_.state()[0], state_.state()[1]);

        auto rect = collision::ConvexPolygon::create_rectangle(length_, width_);
        rect.translate(origin_to_com);  // Now origin of car is at origin coordinates.
        rect.rotate(state_.state()[2]);  // Rotate around car's origin.
        rect.translate(origin);

        return rect;
    }

    std::string log_header() const override {
        return util::comma_join<std::vector<std::string>>({state_.log_header(), polygon().log_header()});
    }

    std::string log() const override {
        return util::comma_join<std::vector<std::string>>({state_.log(), polygon().log()});;
    }

private:
    double l_, width_, length_;
};
};


#endif //DUBINS_H
