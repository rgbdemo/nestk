#pragma once

#include "hub/update.h"
#include <QString>

namespace ntk { namespace hub {

class Hub::StatusUpdate : public Hub::Update
{
public:
    StatusUpdate (QString name, QString status);

public:
    virtual void updateHub (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

public:
    QString status;
};

} }
