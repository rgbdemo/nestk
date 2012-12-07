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
        ntk_dbg_print (it.key(), 1);
        ntk_dbg_print (it.value().toString(), 1);
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
