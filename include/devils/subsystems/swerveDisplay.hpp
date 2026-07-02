#pragma once

#include "devils/devilLib/display/displayBase.hpp"
#include "devils/devilLib/utils/backgroundService.hpp"

namespace devils
{
    class SwerveDisplay :
        public BackgroundService,
        public DisplayBase
    {
    public:
        SwerveDisplay()
        {
            fullScreenContainer = lv_obj_create(getRootContainer());
            lv_obj_set_style_bg_color(fullScreenContainer, lv_color_make(3, 36, 53), 0);
            lv_obj_set_size(fullScreenContainer, lv_pct(100), lv_pct(100));

            eyesGroup = lv_obj_create(fullScreenContainer);
            lv_obj_set_size(eyesGroup, 340, 200);
            lv_obj_align(eyesGroup, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_pos(eyesGroup, 0, 0);
            lv_obj_set_style_radius(eyesGroup, 0, 0);
            lv_obj_set_style_border_width(eyesGroup, 0, 0);
            lv_obj_set_style_bg_color(fullScreenContainer, lv_color_make(3, 36, 53), 0);
            lv_obj_set_style_bg_opa(eyesGroup, 0, 0);
            lv_obj_set_scrollbar_mode(eyesGroup, LV_SCROLLBAR_MODE_OFF);

            leftEye = makeEye(-100, 0, 120);
            rightEye = makeEye(100, 0, 120);
            leftEyebrow = makeEyebrow(-100, -85, 40, 130);
            rightEyebrow = makeEyebrow(100, -110, -40, 130);
        }

        ~SwerveDisplay() override
        {
            lv_obj_del(fullScreenContainer);
        }

    protected:
        void onUpdate() override
        {
            static int16_t t = 0;
            t++;

            lv_obj_set_pos(
                eyesGroup,
                std::cos(t * 0.03) * 24,
                0);
        }

        /**
         * Makes an eye object in LVGL with the given position and size and adds it to the eyes group.
         * @param x - The x position of the eye relative to the center of the screen.
         * @param y - The y position of the eye relative to the center of the screen.
         * @param size - The size of the eye in pixels. The eye will be a square with sides of this length.
         * @return The created eye object.
         */
        lv_obj_t* makeEye(
            const int16_t x,
            const int16_t y,
            const int16_t size) const
        {
            // Object
            lv_obj_t* eye = lv_obj_create(eyesGroup);
            lv_obj_set_size(eye, size, size);
            lv_obj_align(eye, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_pos(eye, x, y);

            // Style
            lv_obj_set_style_bg_color(eye, lv_color_make(255, 255, 255), 0);
            lv_obj_set_style_radius(eye, LV_RADIUS_CIRCLE, 0);
            lv_obj_set_style_border_width(eye, 0, 0);

            return eye;
        }

        /**
         * Makes an eyebrow object in LVGL with the given position, angle, and size and adds it to the eyes group.
         * @param x - The x position of the eyebrow relative to the center of the screen.
         * @param y - The y position of the eyebrow relative to the center of the screen.
         * @param angle - The angle of the eyebrow in degrees. 0 is flat, positive is raised, negative is furrowed.
         * @param size - The size of the eyebrow in pixels. The eyebrow will be a square with sides of this length.
         * @return The created eyebrow object.
         */
        lv_obj_t* makeEyebrow(
            const int16_t x,
            const int16_t y,
            const int16_t angle,
            const int16_t size) const
        {
            // Object
            lv_obj_t* eyebrow = lv_obj_create(eyesGroup);
            lv_obj_set_size(eyebrow, size, size);

            // Style
            lv_obj_set_style_bg_color(eyebrow, lv_color_make(3, 36, 53), 0);
            lv_obj_set_style_radius(eyebrow, 0, 0);
            lv_obj_set_style_border_width(eyebrow, 0, 0);
            lv_obj_set_style_transform_pivot_x(eyebrow, size / 2, 0);
            lv_obj_set_style_transform_pivot_y(eyebrow, size, 0);
            lv_obj_set_style_transform_angle(eyebrow, angle, 0);

            lv_obj_align(eyebrow, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_pos(eyebrow, x, y);

            return eyebrow;
        }

    private:
        lv_obj_t* fullScreenContainer;
        lv_obj_t* eyesGroup;
        lv_obj_t* leftEye;
        lv_obj_t* rightEye;
        lv_obj_t* leftEyebrow;
        lv_obj_t* rightEyebrow;
    };
}
