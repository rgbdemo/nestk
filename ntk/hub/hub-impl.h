#pragma once

#include "hub.h"
#include "outlet.h"
#include "mesh/mesh.h"
#include <QImage>
#include <QStringList>
#include <QString>
#include <QHash>
#include <QSet>
#include <QMutex>

namespace ntk { namespace hub {

struct Hub::Impl
{
public:
    Impl (Hub* that);

#define HUB_IMPL_MEMBER(name, ...) \
    mutable QMutex      name##Mutex;     \
    __VA_ARGS__         name;

#define HUB_IMPL_MEMBER_TYPEDEF(Name, name, ...) \
    typedef __VA_ARGS__ Name;                    \
    HUB_IMPL_MEMBER(name, Name)

#define HUB_IMPL_VALUES(Type, type, Val)                            \
    HUB_IMPL_MEMBER_TYPEDEF(Type##Values, type##Values, QHash<String, Val>)

#define HUB_TYPE(Type, type, Arg, Ret, Val) \
    HUB_IMPL_VALUES(Type, type, Val)
        HUB_TYPES()
#undef  HUB_TYPE

#define HUB_IMPL_LOCKED(name) QMutexLocker _(&name##Mutex);

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
    void   subscribeOutlet (Outlet* outlet, String name);
    void unsubscribeOutlet (Outlet* outlet, String name);
    void       startOutlet (Outlet* outlet);
    void        stopOutlet (Outlet* outlet);

public: // Activity
    void setEnabled (bool enabled_)
    {
        HUB_IMPL_LOCKED(enabled)

        enabled = enabled_;
    }
    void  enable () { setEnabled(true);  }
    void disable () { setEnabled(false); }

    bool isActive (const QString& name)
    {
        {
            HUB_IMPL_LOCKED(enabled)

            if (!enabled)
                return false;
        }

        {
            HUB_IMPL_LOCKED(activeSubscriptions);

            const ActiveSubscriptions::ConstIterator i = activeSubscriptions.find(name);

            if (i == activeSubscriptions.end())
                return false;

            return 0 < i.value();
        }
    }

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
