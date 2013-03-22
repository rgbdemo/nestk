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

#include "parameter_set.h"

#include <ntk/utils/debug.h>

namespace ntk
{

ParameterSet::ParameterSet(const ParameterSet &rhs)
{
    QMutexLocker __(&mutex);
    QMutexLocker _(&rhs.mutex);
    this->properties = rhs.properties;
    // FIXME: should also copy watchers?
}

void ParameterSet :: setParameter(const QString& name, const QVariant& value)
{
    {
        QMutexLocker _(&mutex);
        properties[name] = value;
    }

    foreach(ParameterSetWatcher* watcher, watchers)
        watcher->onParameterUpdated(name, value);
}

void ParameterSet :: setParameters(const ParameterSet& rhs)
{
    QMutexLocker _(&mutex);
    QMutexLocker __(&rhs.mutex);
    QHashIterator<QString, QVariant> it (rhs.properties);
    while (it.hasNext())
    {
        it.next ();
        properties[it.key()] = it.value();
    }
}

QVariant ParameterSet :: getParameter(const QString& name, QVariant default_value) const
{
    QVariant value;
    QMutexLocker _(&mutex);
    if (properties.contains(name)) // FIXME: use find.
        value = properties[name];
    else
        value = default_value;
    return value;
}

bool ParameterSet :: hasParameter(const QString& name) const
{
    QMutexLocker _(&mutex);
    return properties.contains(name);
}

} // ntk
