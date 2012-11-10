#pragma once

#include "hub.h"
#include "ntk/thread/event.h"
#include <QString>

Q_DECLARE_METATYPE(ntk::hub::Name)

namespace ntk { namespace hub {

class Hub;
class Outlet;

class Hub::Update : public ntk::EventData
{
    TYPEDEF_THIS(Update)

//    CLONABLE_EVENT_DATA

public:
             Update (QString name);
    virtual ~Update ();

public:
    virtual void updateHub (Hub& hub) = 0;
    virtual void updateOutlet (Outlet& outlet) = 0;

public:
    const QString name;
};

typedef Hub::Update HubUpdate;

ntk_ptr_typedefs(HubUpdate)

} }
