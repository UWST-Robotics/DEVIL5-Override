#pragma once

#include <map>
#include <utility>
#include "liblvgl/lvgl.h"
#include "displayBase.hpp"

namespace devils
{
    class AutoPickerDisplay : public DisplayBase
    {
    public:
        /// @brief List of alliance colors to choose from when selecting an autonomous routine.
        enum AllianceColor
        {
            RED_ALLIANCE = 0,
            BLUE_ALLIANCE = 1,
            NONE_ALLIANCE = 2
        };

        /// @brief A struct representing an autonomous routine that can be selected by the user.
        struct Routine
        {
            uint8_t id = 0;
            std::string displayName = "Routine";
            bool requiresAllianceColor = false;
        };

        /**
         * 
         * @param robotName - The name of the robot to display at the top of the screen
         * @param routines - A list of routines to display for the user to select from. The first routine in the list will be selected by default.
         */
        AutoPickerDisplay(
            std::string robotName,
            const std::vector<Routine>& routines)
            : robotName(std::move(robotName)),
              routines(routines)
        {
            root = lv_obj_create(getRootContainer());
            lv_obj_set_size(root, LV_HOR_RES_MAX, LV_VER_RES_MAX);
            lv_obj_center(root);
            lv_obj_set_style_bg_opa(root, 0, 0);
            lv_obj_set_style_border_opa(root, 0, 0);

            createOptionsDisplayContainer();
        }

        ~AutoPickerDisplay() override
        {
            lv_obj_del(root);
        }

        /**
         * Gets the currently selected autonomous routine.
         * @return The currently selected autonomous routine.
         */
        Routine getSelectedRoutine()
        {
            return selectedRoutine;
        }

        /**
         * Sets the visibility of the auto picker display.
         * @param visible - Whether the auto picker display should be visible or not.
         */
        void setVisible(const bool visible) const
        {
            if (visible)
                lv_obj_clear_flag(root, LV_OBJ_FLAG_HIDDEN);
            else
                lv_obj_add_flag(root, LV_OBJ_FLAG_HIDDEN);
        }

    protected:
        /// @brief An enum to identify the type of callback in the event system, allowing us to use a single callback function for multiple different events.
        enum CallbackType
        {
            ROUTINE,
            ALLIANCE_COLOR,
            SCREEN_SAVER_TOGGLE
        };

        /// @brief A struct to hold data for event callbacks, allowing us to pass multiple pieces of information through the lvgl event system.
        struct CallbackData
        {
            AutoPickerDisplay* display;
            CallbackType type;
            void* data;
        };

        /**
         * Static method used as a callback for all events in the auto picker display. The CallbackData struct is used to determine what type of event occurred and to access the relevant data for that event.
         * @param e - The event data passed by the LVGL event system when an event occurs.
         */
        static void onCallback(lv_event_t* e)
        {
            const auto data = static_cast<CallbackData*>(lv_event_get_user_data(e));
            switch (data->type)
            {
            case ROUTINE:
                data->display->handleRoutineChange(e);
                break;
            case ALLIANCE_COLOR:
                data->display->handleAllianceColorChange(e);
                break;
            case SCREEN_SAVER_TOGGLE:
                data->display->handleScreenSaverToggle(e);
                break;
            }
        }

        void createOptionsDisplayContainer()
        {
            const auto fullscreen_container = lv_obj_create(root);
            lv_obj_set_size(fullscreen_container, lv_pct(100), lv_pct(100));
            lv_obj_set_layout(fullscreen_container, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(fullscreen_container, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_flex_align(
                fullscreen_container,
                LV_FLEX_ALIGN_CENTER,
                LV_FLEX_ALIGN_CENTER,
                LV_FLEX_ALIGN_CENTER);

            lv_obj_t* title = lv_label_create(fullscreen_container);
            lv_label_set_text(title, robotName.c_str());
            lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(title, lv_color_make(255, 255, 255), 0);
            lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);

            auto options_display_container = lv_obj_create(fullscreen_container);
            lv_obj_set_style_pad_all(options_display_container, 0, 0);
            lv_obj_set_flex_grow(options_display_container, 1);
            lv_obj_set_style_border_width(options_display_container, 0, 0);
            lv_obj_set_width(options_display_container, lv_pct(100));
            lv_obj_clear_flag(options_display_container, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_layout(options_display_container, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(options_display_container, LV_FLEX_FLOW_ROW);
            lv_obj_set_style_pad_row(options_display_container, 0, 0);

            createAllianceColorContainer(options_display_container);
            createRoutineContainer(options_display_container);
        }

        void createAllianceColorContainer(lv_obj_t* parent)
        {
            const auto alliance_color_container = lv_obj_create(parent);
            lv_obj_set_size(alliance_color_container, lv_pct(30), lv_pct(100));
            lv_obj_set_layout(alliance_color_container, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(alliance_color_container, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_flex_align(alliance_color_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                                  LV_FLEX_ALIGN_CENTER);

            lv_obj_t* alliance_title = lv_label_create(alliance_color_container);
            lv_label_set_text(alliance_title, "Alliance");
            lv_obj_set_style_text_align(alliance_title, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(alliance_title, lv_color_make(255, 255, 255), 0);
            lv_obj_set_style_text_font(alliance_title, &lv_font_montserrat_16, 0);

            lv_obj_t* alliance_color_button = lv_btn_create(alliance_color_container);
            lv_obj_set_size(alliance_color_button, lv_pct(100), 100);
            lv_obj_set_flex_grow(alliance_color_button, 1);
            lv_obj_set_style_bg_color(alliance_color_button, ALLIANCE_COLOR_MAP.at(selectedAlliance), 0);
            lv_obj_add_event_cb(
                alliance_color_button,
                onCallback,
                LV_EVENT_CLICKED,
                new CallbackData{this, ALLIANCE_COLOR, nullptr});

            lv_obj_t* alliance_color_label = lv_label_create(alliance_color_button);
            lv_label_set_text(alliance_color_label, ALLIANCE_NAME_MAP.at(selectedAlliance).c_str());
            lv_obj_center(alliance_color_label);
        }

        void createRoutineContainer(lv_obj_t* parent)
        {
            const auto right_container = lv_obj_create(parent);
            lv_obj_set_size(right_container, lv_pct(70), lv_pct(100));
            lv_obj_set_layout(right_container, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(right_container, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(right_container, LV_OBJ_FLAG_SCROLLABLE);

            const auto routine_container = lv_obj_create(right_container);
            lv_obj_set_size(routine_container, lv_pct(70), lv_pct(100));
            lv_obj_set_layout(routine_container, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(routine_container, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_style_border_width(routine_container, 0, 0);
            lv_obj_set_style_pad_all(routine_container, 0, 0);

            // title
            lv_obj_t* routine_title = lv_label_create(routine_container);
            lv_label_set_text(routine_title, "Select Routine");
            lv_obj_set_style_text_align(routine_title, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_set_style_text_color(routine_title, lv_color_make(255, 255, 255), 0);
            lv_obj_set_style_text_font(routine_title, &lv_font_montserrat_16, 0);

            for (const auto& routine : routines)
                createRoutineButton(routine_container, routine);

            // std::vector<std::string> routine_names;
            // for (const auto& routine : newRoutines)
            //     routine_names.push_back(routine.displayName);

            // TODO: FIX ME
            // const auto radio_group = new RadioGroup(routine_container, routine_names, handleRoutineChange);
            // lv_obj_t* radio_group_obj = radio_group->getRadioGroup();
            // lv_obj_set_width(radio_group_obj, lv_pct(100));
            // lv_obj_set_flex_grow(radio_group_obj, 1);
            // lv_obj_set_style_pad_all(radio_group_obj, 0, 0);
            // lv_obj_set_style_border_width(radio_group_obj, 0, 0);

            const auto screen_saver_toggle_container = lv_obj_create(right_container);
            lv_obj_set_size(screen_saver_toggle_container, lv_pct(30), lv_pct(100));
            lv_obj_set_layout(screen_saver_toggle_container, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(screen_saver_toggle_container, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_flex_align(screen_saver_toggle_container, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END,
                                  LV_FLEX_ALIGN_END);
            lv_obj_set_style_pad_all(screen_saver_toggle_container, 0, 0);
            lv_obj_set_style_border_width(screen_saver_toggle_container, 0, 0);

            const auto screen_saver_toggle_button = lv_btn_create(screen_saver_toggle_container);
            lv_obj_set_size(screen_saver_toggle_button, lv_pct(100), 50);
            const auto screen_saver_toggle_label = lv_label_create(screen_saver_toggle_button);
            lv_label_set_text(screen_saver_toggle_label, "Save");
            lv_obj_center(screen_saver_toggle_label);
            lv_obj_add_event_cb(
                screen_saver_toggle_button,
                onCallback,
                LV_EVENT_CLICKED,
                new CallbackData{this, SCREEN_SAVER_TOGGLE, nullptr});
        }

        void createRoutineButton(lv_obj_t* parent, const Routine& routine)
        {
            const auto button = lv_checkbox_create(parent);
            lv_checkbox_set_text(button, routine.displayName.c_str());
            lv_obj_add_flag(button, LV_OBJ_FLAG_EVENT_BUBBLE);
            lv_obj_add_event_cb(
                button,
                onCallback,
                LV_EVENT_VALUE_CHANGED,
                new CallbackData{this, ROUTINE, new Routine(routine)});
        }

        void handleRoutineChange(lv_event_t* e)
        {
            // Deselect other checkboxes
            const auto target = lv_event_get_target(e);
            const auto parent = lv_obj_get_parent(target);
            const auto child_count = lv_obj_get_child_cnt(parent);
            for (auto i = 0; i < child_count; i++)
            {
                const auto child = lv_obj_get_child(parent, i);
                if (child != target)
                    lv_obj_clear_state(child, LV_STATE_CHECKED);
            }

            // Ensure the clicked checkbox is selected
            lv_obj_add_state(target, LV_STATE_CHECKED);

            // Update selected routine
            const auto data = static_cast<CallbackData*>(lv_event_get_user_data(e));
            selectedRoutine = *static_cast<Routine*>(data->data);
        }

        void handleAllianceColorChange(lv_event_t* e)
        {
            auto* btn = static_cast<lv_obj_t*>(lv_event_get_target(e));
            lv_obj_t* label = lv_obj_get_child(btn, 0);
            if (label != nullptr)
            {
                const auto text = lv_label_get_text(label);
                const auto currentColor = getCurrentAllianceColor(text);

                const auto nextColor = getNextAllianceColor(currentColor);
                lv_obj_set_style_bg_color(btn, ALLIANCE_COLOR_MAP.at(nextColor), 0);
                lv_label_set_text(label, ALLIANCE_NAME_MAP.at(nextColor).c_str());
                selectedAlliance = nextColor;
            }
        }

        void handleScreenSaverToggle(lv_event_t* e) const
        {
            if (selectedRoutine.requiresAllianceColor &&
                selectedAlliance == NONE_ALLIANCE)
            {
                // show error message
                lv_obj_t* message_box = lv_msgbox_create(
                    root,
                    "Error",
                    "Selected routine can't be used when Alliance is set to 'None'.",
                    nullptr,
                    true);

                // center the message box
                // shadow
                lv_obj_set_style_shadow_color(message_box, lv_color_make(0, 0, 0), 0);
                lv_obj_set_style_shadow_width(message_box, 10, 0);
                lv_obj_set_style_shadow_opa(message_box, LV_OPA_COVER, 0);
                lv_obj_set_style_shadow_ofs_x(message_box, 4, 0);
                lv_obj_set_style_shadow_ofs_y(message_box, 4, 0);
                // border

                lv_obj_center(message_box);
                return;
            }

            // Hide
            setVisible(false);
        }

        AllianceColor getCurrentAllianceColor(const char* text) const
        {
            for (const auto& pair : ALLIANCE_NAME_MAP)
            {
                if (pair.second == text)
                    return pair.first;
            }
            return NONE_ALLIANCE;
        }

        AllianceColor getNextAllianceColor(const AllianceColor current_color) const
        {
            auto it = ALLIANCE_COLOR_MAP.find(current_color);
            auto next_it = std::next(it);
            if (next_it == ALLIANCE_COLOR_MAP.end())
                next_it = ALLIANCE_COLOR_MAP.begin();
            return next_it->first;
        }

    private:
        const std::map<AllianceColor, lv_color_t> ALLIANCE_COLOR_MAP = {
            {RED_ALLIANCE, lv_color_make(255, 0, 0)},
            {BLUE_ALLIANCE, lv_color_make(0, 0, 255)},
            {NONE_ALLIANCE, lv_color_make(124, 124, 124)}
        };
        const std::map<AllianceColor, std::string> ALLIANCE_NAME_MAP = {
            {RED_ALLIANCE, "Red"},
            {BLUE_ALLIANCE, "Blue"},
            {NONE_ALLIANCE, "None"}
        };

        std::string robotName;
        std::vector<Routine> routines;

        AllianceColor selectedAlliance = NONE_ALLIANCE;
        Routine selectedRoutine;
        bool isModalOpen = true;

        lv_obj_t* root;
    };
}
