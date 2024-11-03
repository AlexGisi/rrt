//
// Created by agisi2 on 10/25/2024.
//

#ifndef MODEL_H
#define MODEL_H

#include <functional>
#include <optional>
#include <util/util.h>
#include <collision/convex_polygon.h>


namespace rrt::models {

template <typename Derived, int nx_>
class State {
public:
    using BoundArr = std::array<std::optional<util::ClosedInterval>, nx_>;
    using StateArr = std::array<double, nx_>;

    State() = delete;

    explicit State(const BoundArr &bounds) : bounds_(bounds) {}

    State(const StateArr &x, const BoundArr &bounds)
        : state_(x), bounds_(bounds) {}

    virtual ~State() = default;

    void randomize() {
        for (int i = 0; i < nx_; i++) {
            if (bounds_[i].has_value()) {
                state_[i] = bounds_[i].value().sample();
            }
        }
    }

    virtual double distance(const Derived &other) const = 0;

    virtual Derived interpolate(const Derived& other, double weight) const = 0;

    bool valid() const {
        for (int i = 0; i < nx_; i++) {
            if(bounds_[i].has_value() && !bounds_[i].value().contains(state_[i])) {
                return false;
            }
        }
        return true;
    }

    virtual std::string log_header() const = 0;

    std::string log() const {
        return util::comma_join<StateArr>(state_);
    }

    static int nx() { return nx_; }
    StateArr state() const { return state_; }
    BoundArr bounds() const { return bounds_; }

protected:
    StateArr state_{};
    BoundArr bounds_{};
};


template <int nu_>
class Command {
public:
    using BoundArr = std::array<std::optional<util::ClosedInterval>, nu_>;
    using CommandArr = std::array<double, nu_>;

    Command() = delete;

    virtual ~Command() = default;

    explicit Command(BoundArr bounds): bounds_(bounds) {}

    Command(CommandArr u, BoundArr bounds)
        : u_(u), bounds_(bounds) {}

    void randomize() {
        for (int i = 0; i < nu_; i++) {
            if (bounds_[i].has_value()) {
                u_[i] = bounds_[i].value().sample();
            }
        }
    }

    virtual std::string log_header() const = 0;

    std::string log() const {
        return util::comma_join<std::array<double, nu_>>(u_);
    }

    static int nu() { return nu_; }
    CommandArr u() const { return u_; }
    BoundArr bounds() const { return bounds_; }

protected:
    CommandArr u_{};
    BoundArr bounds_;
};


template <typename StateT, typename CommandT>
class Model {
public:
    explicit Model(StateT state) : state_(state) {}
    virtual ~Model() = default;

    void set_state(const StateT &state) {
        this->state_ = state;
    }

    StateT state() const {
        return this->state_;
    }

    virtual void step(const CommandT &command, float dt) = 0;

    virtual collision::ConvexPolygon polygon() const = 0;

    bool in_collision(const Model& other) const {
        return polygon().collides(other.polygon());
    }

    bool in_collision(const collision::ConvexPolygon& other) const {
        return polygon().collides(other);
    }

    bool in_collision(const std::vector<collision::ConvexPolygon>& others) const {
        for (const auto& p : others) {
            if (in_collision(p)) {
                return true;
            }
        }
        return false;
    }

    virtual std::string log_header() const = 0;
    virtual std::string log() const = 0;

protected:
    StateT state_{};
};
};


#endif //MODEL_H
