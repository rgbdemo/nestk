#pragma once

#include "hub/update.h"
#include <QStringList>

namespace ntk { namespace hub {

class Hub::StringsUpdate : public Hub::Update
{
public:
    StringsUpdate (String name);

public:
    virtual void updateHub (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

protected:
    virtual void updateStrings (Strings& strings) = 0;

private:
    Strings strings;
};

//------------------------------------------------------------------------------

class Hub::SetStringsUpdate : public Hub::StringsUpdate
{
public:
    SetStringsUpdate (String name, const Strings& newStrings);

private:
    virtual void updateStrings (Strings& strings);

private:
    Strings newStrings;
};

//------------------------------------------------------------------------------

class Hub::AppendStringsUpdate : public Hub::StringsUpdate
{
public:
    AppendStringsUpdate (String name, String chunk);

private:
    virtual void updateStrings (Strings& strings);

private:
    const String chunk;
};

//------------------------------------------------------------------------------

class Hub::ClearStringsUpdate : public Hub::StringsUpdate
{
public:
    ClearStringsUpdate (String name);

private:
    virtual void updateStrings (Strings& strings);
};

} }
