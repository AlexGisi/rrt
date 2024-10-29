#include <cassert>
#include <util/interval.h>

using namespace rrt::util;


void closed_intersects() {
    constexpr auto i1 = ClosedInterval(-2.0, 2.0);
    constexpr auto i2 = ClosedInterval(2.0, 10);

    assert(i1.intersects(i2));
}

int main(int argc, char **argv) {
    closed_intersects();
}
