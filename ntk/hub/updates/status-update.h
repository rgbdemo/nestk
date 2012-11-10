#pragma once

#include "hub/update.h"
#include <QString>

namespace ntk { namespace hub {

class Hub::StatusUpdate : public Hub::Update
{
public:
    StatusUpdate (Name name, Line status);

public:
    virtual void updateHub (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

public:
    QString status;
};

} }
