#pragma once

#include "hub/update.h"
#include <QStringList>

namespace ntk { namespace hub {

class Hub::LogUpdate : public Hub::Update
{
public:
    LogUpdate (Name name);

public:
    virtual void updateHub (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

protected:
    typedef Lines Log;
    virtual void updateLog (Log& log) = 0;

private:
    Lines log;
};

//------------------------------------------------------------------------------

class Hub::SetLogUpdate : public Hub::LogUpdate
{
public:
    SetLogUpdate (Name name, const Log& log);

private:
    virtual void updateLog (Log& log);

private:
    Lines newLog;
};

//------------------------------------------------------------------------------

class Hub::AppendLogUpdate : public Hub::LogUpdate
{
public:
    AppendLogUpdate (Name name, Line line);

private:
    virtual void updateLog (Log& log);

private:
    const Line line;
};

//------------------------------------------------------------------------------

class Hub::ClearLogUpdate : public Hub::LogUpdate
{
public:
    ClearLogUpdate (Name name);

private:
    virtual void updateLog (Log& log);
};

} }
