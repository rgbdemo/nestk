#pragma once

#include "hub/update.h"
#include <QStringList>

namespace ntk { namespace hub {

class Hub::LogUpdate : public Hub::Update
{
public:
    LogUpdate (QString name);

public:
    virtual void updateHub (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

private:
    virtual void updateHubLog (QStringList& hubLog) = 0;

private:
    QStringList hubLog;
};

//------------------------------------------------------------------------------

class Hub::SetLogUpdate : public Hub::LogUpdate
{
public:
    SetLogUpdate (QString name, QStringList log);

private:
    virtual void updateHubLog (QStringList& hubLog);

private:
    QStringList log;
};

//------------------------------------------------------------------------------

class Hub::AppendLogUpdate : public Hub::LogUpdate
{
public:
    AppendLogUpdate (QString name, QString line);

private:
    virtual void updateHubLog (QStringList& hubLog);

private:
    const QString line;
};

//------------------------------------------------------------------------------

class Hub::ClearLogUpdate : public Hub::LogUpdate
{
public:
    ClearLogUpdate (QString name);

private:
    virtual void updateHubLog (QStringList& hubLog);
};

} }
