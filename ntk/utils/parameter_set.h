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

    void addWatcher(ParameterSetWatcher* watcher) { watchers.insert(watcher); }
    void removeWatcher(ParameterSetWatcher* watcher) { watchers.remove(watcher); }
    int numWatchers() const { return watchers.size(); }

    void setParameter(const QString& name, const QVariant& value)
    {
        {
            QMutexLocker _(&mutex);
            properties[name] = value;
        }

        foreach(ParameterSetWatcher* watcher, watchers)
            watcher->onParameterUpdated(name, value);
    }

    QVariant getParameter(const QString& name, QVariant default_value = QVariant()) const
    {
        QVariant value;
        QMutexLocker _(&mutex);
        if (properties.contains(name)) // FIXME: use find.
            value = properties[name];
        else
            value = default_value;
        return value;
    }

    bool hasParameter(const QString& name) const
    {
        QMutexLocker _(&mutex);
        return properties.contains(name);
    }

protected:
    mutable QMutex mutex;
    QHash<QString, QVariant> properties;
    QSet<ParameterSetWatcher*> watchers;
};

class ParameterSetManager : public ParameterSetWatcher
{
public:
    ParameterSetManager(ParameterSet* Parameter_set)
        : parameter_set(Parameter_set)
    {
        parameter_set->addWatcher(this);
    }

    virtual ~ParameterSetManager()
    {
        parameter_set->removeWatcher(this);
        if (parameter_set->numWatchers() == 0)
            delete parameter_set;
    }

public:
    bool hasParameter(const QString& name) const
    { return parameter_set->hasParameter(name); }

    void setParameter(const QString& name, QVariant value)
    {
        parameter_set->setParameter(name, value);
    }

    QVariant getParameter(const QString& name, QVariant default_value = QVariant())
    {
        QVariant result = parameter_set->getParameter(name, default_value);
        return result;
    }

    virtual void onParameterUpdated(const QString& name, const QVariant& value) {}

    ParameterSet* parameterSet() { return parameter_set; }

protected:
    ParameterSet* parameter_set;
};

}

#endif // NESTK_UTILS_PARAMETERS_SET_H
