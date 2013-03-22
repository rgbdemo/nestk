/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
 */

#ifndef NESTK_UTILS_PARAMETERS_SET_H
#define NESTK_UTILS_PARAMETERS_SET_H

#include <ntk/core.h>

#include <QHash>
#include <QVariant>
#include <QMutexLocker>
#include <QSet>

namespace ntk
{

class ParameterSetWatcher
{
public:
    virtual void onParameterUpdated(const QString& name, const QVariant& value) {}
};

class ParameterSet
{
public:
    ParameterSet() {}
    ParameterSet(const ParameterSet& rhs);

    virtual ~ParameterSet () {}

    void addWatcher(ParameterSetWatcher* watcher) { watchers.insert(watcher); }
    void removeWatcher(ParameterSetWatcher* watcher) { watchers.remove(watcher); }
    int numWatchers() const { return watchers.size(); }

    void     setParameter(const QString& name, const QVariant& value);
    QVariant getParameter(const QString& name, QVariant default_value = QVariant()) const;
    bool     hasParameter(const QString& name) const;

    void setParameters(const ParameterSet& rhs);

protected:
    mutable QMutex mutex;
    QHash<QString, QVariant> properties;
    QSet<ParameterSetWatcher*> watchers;
};

class ParameterSetManager : public ParameterSetWatcher
{
public:
    ParameterSetManager(ParameterSet* parameter_set = 0)
        : parameter_set(parameter_set ? parameter_set : new ParameterSet)
    {
        this->parameter_set->addWatcher(this);
    }

    virtual ~ParameterSetManager()
    {
        parameter_set->removeWatcher(this);
        if (parameter_set->numWatchers() == 0)
            delete parameter_set;
    }

public:
    virtual bool hasParameter(const QString& name) const
    { return parameter_set->hasParameter(name); }

    virtual void setParameter(const QString& name, QVariant value)
    {
        parameter_set->setParameter(name, value);
    }

    virtual void setParameters(const ParameterSet& set)
    {
        parameter_set->setParameters(set);
    }

    virtual QVariant getParameter(const QString& name, QVariant default_value = QVariant()) const
    {
        QVariant result = parameter_set->getParameter(name, default_value);
        return result;
    }

    virtual void onParameterUpdated(const QString& name, const QVariant& value) {}

    virtual ParameterSet* parameterSet() { return parameter_set; }

protected:
    ParameterSet* parameter_set;
};

}

#endif // NESTK_UTILS_PARAMETERS_SET_H
