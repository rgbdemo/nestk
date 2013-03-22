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
 * Author: Nicolas Tisserand <nicolas.tisserand@manctl.com>
 */

#ifndef NTK_HUB_HUB_IMPL_H
# define NTK_HUB_HUB_IMPL_H

#include "hub.h"
#include "outlet.h"
#include "mesh/mesh.h"
#include <QImage>
#include <QStringList>
#include <QString>
#include <QHash>
#include <QSet>
#include <QMutex>
#include <QObject>

#define HUB_IMPL_MEMBER(name, ...) \
    mutable QMutex      name##Mutex;     \
    __VA_ARGS__         name;

#define HUB_IMPL_MEMBER_TYPEDEF(Name, name, ...) \
    typedef __VA_ARGS__ Name;                    \
    HUB_IMPL_MEMBER(name, Name)

#define HUB_IMPL_VALUES(Type, type, Val)                            \
    HUB_IMPL_MEMBER_TYPEDEF(Type##Values, type##Values, QHash<String, Val>)

#define HUB_IMPL_LOCKED(name) QMutexLocker _(&name##Mutex);

#define HUB_TYPE(Type, type, Arg, Ret, Val) \
    HUB_IMPL_VALUES(Type, type, Val)

namespace ntk { namespace hub {

struct Hub::Impl : QObject
{
    Q_OBJECT

public:
     Impl (Hub* that);
    ~Impl ();

public slots:
     void quit ();

public: // Names
    bool maybeAddName (const QString& name)
    {
        HUB_IMPL_LOCKED(names);

        if (names.ordered.contains(name))
            return false;

        names.ordered.insert(name);
        names.sequenced.append(name);
        return true;
    }

    bool hasName (const QString& name) const
    {
        HUB_IMPL_LOCKED(names);

        return names.ordered.contains(name);
    }

    int getNumNames () const
    {
        HUB_IMPL_LOCKED(names);

        return names.sequenced.size();
    }

    const QString& getNameAt (int idx) const
    {
        HUB_IMPL_LOCKED(names);

        if (idx < 0 || names.sequenced.size() <= idx)
            return emptyName;

        return names.sequenced[idx];
    }

    // Inefficient. Don't call that when looping over long name sequences.
    const QString& getOrderedNameAt (int idx) const
    {
        HUB_IMPL_LOCKED(names);

        if (idx < 0 || names.sequenced.size() <= idx)
            return emptyName;

        Dictionary::Ordered::ConstIterator i = names.ordered.begin();
        std::advance(i, idx);
        return *i;
    }

public:
    HUB_TYPES()

public: // Names
    static const QString emptyName;
    struct Dictionary
    {
        typedef QSet<QString>    Ordered  ; Ordered   ordered;
        typedef QVector<QString> Sequenced; Sequenced sequenced;
    };
    HUB_IMPL_MEMBER_TYPEDEF(Names, names, Dictionary)

public: // Outlets
    void      attachOutlet (Outlet* outlet);
    void      detachOutlet (Outlet* outlet);
    void   subscribeOutlet (Outlet* outlet, const String& name);
    void unsubscribeOutlet (Outlet* outlet, const String& name);
    void       startOutlet (Outlet* outlet);
    void        stopOutlet (Outlet* outlet);

public: // Activity
    void setEnabled (bool enabled);
    void     enable ();
    void    disable ();
    bool   isActive (const QString& name);

private:
    HUB_IMPL_MEMBER(enabled, bool)

private: // Outlets
    struct OutletInfo
    {
        bool running;
        typedef QSet<String> Subscriptions;
        Subscriptions subscriptions;
    };

    mutable QMutex                     outletInfosMutex;
    typedef QHash<Outlet*, OutletInfo> OutletInfos;
    OutletInfos                        outletInfos;

    mutable QMutex             activeSubscriptionsMutex;
    typedef QHash<String, int> ActiveSubscriptions;
    ActiveSubscriptions        activeSubscriptions;

private:
    Hub* that;
};

} }

#undef HUB_TYPE

#endif // !NTK_HUB_HUB_IMPL_H
