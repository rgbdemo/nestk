#pragma once

#include "hub/update.h"
#include <QtGlobal>

namespace ntk { namespace hub {

class Hub::ProgressUpdate : public Hub::Update
{
public:
    ProgressUpdate (QString name, qreal progress);

public:
    virtual void updateHub (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

public:
    qreal progress;
};

} }
