#pragma once

#include "hub/update.h"
#include <QtGlobal>

namespace ntk { namespace hub {

class Hub::ProgressUpdate : public Hub::Update
{
public:
    ProgressUpdate (Name name, Percentage progress);

public:
    virtual void updateHub (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

public:
    Percentage progress;
};

} }
