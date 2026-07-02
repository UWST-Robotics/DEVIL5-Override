#pragma once

#include "asyncTask.hpp"

namespace devils
{
    /**
     *  Represents an asynchronous background service.
     *  Like `AsyncTask`, classes derived from `BackgroundService` must be managed by `std::shared_ptr`.
     */
    class BackgroundService : public AsyncTask
    {
    public:
        BackgroundService() : AsyncTask("BackgroundService") {}
    };
}
