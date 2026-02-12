#pragma once
#include "liblvgl/core/lv_disp.h"
#include "liblvgl/core/lv_obj.h"

namespace devils
{
    /**
     * Base class for all displays. Provides common functionality for all displays, such as a root container to create display elements in.
     */
    class DisplayBase
    {
    public:
        virtual ~DisplayBase() = default;
        
    protected:
        /**
         * Gets or creates the root container for the display. All display elements should be created as children of this container.
         * @return A pointer to the root container for the display. All display elements should be created as children of this container.
         */
        static lv_obj_t* getRootContainer()
        {
            // Create a single root container for all displays to use.
            // This ensures that multiple displays can be used at the same time without interfering with each other.
            static lv_obj_t* rootContainer = nullptr;
            if (rootContainer == nullptr)
            {
                rootContainer = lv_obj_create(nullptr);
                lv_scr_load(rootContainer);
            }
            
            return rootContainer;
        }
    };
}
