#include <cstddef>
#include <functional>
#include <string>
#include <vector>
#include "lcd/lcd.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "liblvgl/core/lv_disp.h" // IWYU pragma: keep
#include "liblvgl/core/lv_obj.h" // IWYU pragma: keep
#include "liblvgl/core/lv_obj_pos.h" // IWYU pragma: keep
#include "liblvgl/core/lv_obj_scroll.h"
#include "liblvgl/core/lv_obj_style.h" // IWYU pragma: keep
#include "liblvgl/font/lv_font.h" // IWYU pragma: keep
#include "liblvgl/lvgl.h" // IWYU pragma: keep
#include "liblvgl/misc/lv_area.h" // IWYU pragma: keep
#include "liblvgl/misc/lv_color.h" // IWYU pragma: keep
#include "liblvgl/misc/lv_style.h" // IWYU pragma: keep
#include "liblvgl/widgets/lv_label.h" // IWYU pragma: keep
#include "pros/rtos.h"
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
    inline int current_auton = -1; // CHANGE TO -1 FOR MATCHES
    inline lv_obj_t * screen = nullptr;
    inline bool alive = false;
    inline pros::Task limlib_integration([](){});
    inline lemlib::Chassis * c = nullptr;
    inline pros::adi::DigitalIn auton_btn('a');
    
    inline void kill() {
        lv_obj_del(screen);
        alive = false;
    }
    inline void call_selected_auton() {
        if (alive) {kill();}
        if (current_auton != -1) { autons[current_auton].func(); }
    }
    inline void add_auton(as::Auton auton) { as::autons.push_back(auton); }

    const inline int bg_color = 0x222222;


    inline void init() {
        alive = true;

        static lv_style_t style;
        lv_style_init(&style);
        //lv_style_set_radius(&style, 0);
        lv_style_set_text_color(&style, lv_color_white());
        lv_style_set_text_align(&style, LV_TEXT_ALIGN_CENTER);
        lv_style_set_border_opa(&style, 0);
        lv_style_set_bg_opa(&style, 255);
        lv_style_set_bg_color(&style, lv_color_black());

        screen = lv_label_create(lv_scr_act());
        lv_obj_set_size(screen, 480, 240);
        lv_obj_add_style(screen, &style, 0);
        lv_label_set_text(screen, "");

        lv_obj_t * holder = lv_obj_create(screen);
        lv_obj_set_size(holder, 200, 200);
        lv_obj_set_pos(holder, 20, 20);
        lv_obj_set_style_pad_all(holder, 0, 0);
        lv_obj_set_style_bg_color(holder, lv_color_hex(bg_color), 0);
        lv_obj_set_scrollbar_mode(holder, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scroll_dir(holder, LV_DIR_VER);

        lv_obj_t * auton_disp = lv_obj_create(screen);
        lv_obj_add_style(auton_disp, &style, 0);
        lv_obj_set_size(auton_disp, 200, 98);
        lv_obj_set_pos(auton_disp, 260, 20);
        lv_obj_set_scrollbar_mode(auton_disp, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scroll_dir(auton_disp, LV_DIR_NONE);
        lv_obj_set_style_border_opa(auton_disp, 255, 0);

        lv_obj_t * auton_lable = lv_label_create(auton_disp);
        lv_obj_center(auton_lable);
        lv_label_set_text(auton_lable, "Selected Auton:\nNone");
        lv_obj_set_style_text_align(auton_lable, LV_TEXT_ALIGN_CENTER, 0);

        lv_obj_t * lemlib_int = lv_obj_create(screen);
        lv_obj_add_style(lemlib_int, &style, 0);
        lv_obj_set_size(lemlib_int, 200, 98);
        lv_obj_set_pos(lemlib_int, 260, 122);
        lv_obj_set_scrollbar_mode(lemlib_int, LV_SCROLLBAR_MODE_OFF);
        lv_obj_set_scroll_dir(lemlib_int, LV_DIR_NONE);
        lv_obj_set_style_border_opa(lemlib_int, 255, 0);

        lv_obj_t * lemlib_int_lable = lv_label_create(lemlib_int);
        lv_obj_center(lemlib_int_lable);
        lv_label_set_text(lemlib_int_lable, "setting up...");
        lv_obj_set_style_text_align(lemlib_int_lable, LV_TEXT_ALIGN_CENTER, 0);

        pros::Task limlib_integration([lemlib_int_lable](){
            while (alive) {
                lv_label_set_text(lemlib_int_lable, (
                    "X: " + std::to_string(c->getPose().x) +
                    "\nY: " + std::to_string(c->getPose().y) +
                    "\nTheta: " + std::to_string(c->getPose().theta)
                ).c_str());

                if (auton_btn.get_new_press()) {
                    c->setPose(lemlib::Pose(0,0,0));
                }

                pros::delay(10);
            }
        });

        // add btns
        static lv_style_t btn_style;
        lv_style_init(&btn_style);
        lv_style_set_bg_color(&btn_style, lv_color_hex(bg_color));

        for (int i = 0; i < autons.size(); i++) {
            lv_obj_t * atn_btn = lv_btn_create(holder);
            lv_obj_add_style(atn_btn, &btn_style, 0);
            lv_obj_set_width(atn_btn, 196);
            lv_obj_set_height(atn_btn, 30);
            lv_obj_set_pos(atn_btn, 0, 31*i);

            lv_obj_t * atn_lbl = lv_label_create(atn_btn);
            lv_obj_center(atn_lbl);
            lv_label_set_text(atn_lbl, autons.at(i).Name.c_str());
            
            if (i != autons.size()-1) {
                lv_obj_t * seperator = lv_obj_create(holder);
                lv_obj_set_pos(seperator, 20, 30+31*i);
                lv_obj_set_size(seperator, 156, 1);
                lv_obj_set_style_bg_color(seperator, lv_color_hex(0x888888), 0);
                lv_obj_set_style_border_opa(seperator, 0, 0);
            }
            lv_obj_set_user_data(atn_btn, (void*)(uintptr_t)i);
            lv_obj_add_event_cb(atn_btn, [](lv_event_t * e) {
                int index = (int)(uintptr_t)lv_obj_get_user_data(lv_event_get_target(e));

                lv_obj_t * lvl = lv_obj_get_child(lv_obj_get_child(lv_obj_get_parent(lv_obj_get_parent(lv_event_get_target(e))),1),0);
                current_auton = index;
                lv_label_set_text(lvl, ("Selected Auton:\n"+autons.at(index).Name).c_str());
            }, LV_EVENT_CLICKED, NULL);
        }
    }
}
