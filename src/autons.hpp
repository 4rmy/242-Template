#include "firebird_template/auton_selector/as.h"
#include "globals.hpp"

inline void red_goal_rush() {
}
inline void blue_goal_rush() {
}
inline void red_ring_rush() {
}
inline void blue_ring_rush() {
}
inline void red_sawp() {
}
inline void blue_sawp() {
}

inline void auton_init() {
    as::c = &chassis;

    as::add_auton(as::Auton("Red Goal Rush", red_goal_rush));
    as::add_auton(as::Auton("Blue Goal Rush", blue_goal_rush));
    as::add_auton(as::Auton("Red Ring Rush", red_ring_rush));
    as::add_auton(as::Auton("Blue Ring Rush", blue_ring_rush));
    as::add_auton(as::Auton("Red Solo AWP", red_sawp));
    as::add_auton(as::Auton("Blue Solo AWP", blue_sawp));

    as::init();
}
