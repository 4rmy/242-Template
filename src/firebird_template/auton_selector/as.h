#include <functional>
#include <string>
#include <vector>
#include "../lcd/lcd.h"
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/core/lv_disp.h" // IWYU pragma: keep
#include "liblvgl/core/lv_obj.h" // IWYU pragma: keep
#include "liblvgl/core/lv_obj_pos.h" // IWYU pragma: keep
#include "liblvgl/core/lv_obj_style.h" // IWYU pragma: keep
#include "liblvgl/font/lv_font.h" // IWYU pragma: keep
#include "liblvgl/lvgl.h" // IWYU pragma: keep
#include "liblvgl/misc/lv_area.h" // IWYU pragma: keep
#include "liblvgl/misc/lv_color.h" // IWYU pragma: keep
#include "liblvgl/misc/lv_style.h" // IWYU pragma: keep
#include "liblvgl/widgets/lv_label.h" // IWYU pragma: keep
#include "pros/rtos.h" // IWYU pragma: keep
#include "pros/rtos.hpp"

namespace as {
    class Auton {
        public:
            std::string Name;
            std::function<void()> func;
            Auton(std::string Name, std::function<void()> func) {
                this->Name = Name;
                this->func = func;
            }
    };

    inline std::vector<Auton> autons = {Auton("No Auton", [](){})};
    inline int current_auton = 1; // CHANGE TO -1 FOR MATCHES
    inline lv_obj_t * screen = nullptr;
    inline bool alive = false;
    inline pros::Task limlib_integration([](){});
    inline lemlib::Chassis * c = nullptr;
    inline pros::adi::DigitalIn auton_btn('a');
    
    void kill();

    void call_selected_auton();
    void add_auton(as::Auton auton);


    void init();
}