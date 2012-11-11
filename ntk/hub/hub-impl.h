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

public: // Real Values
    mutable QMutex              realValuesMutex;
    typedef QHash<String, Real> RealValues;
    RealValues                  realValues;

public: // String Values
    mutable QMutex                stringValuesMutex;
    typedef QHash<String, String> StringValues;
    StringValues                  stringValues;

public: // Strings Values
    mutable QMutex                 stringsValuesMutex;
    typedef QHash<String, Strings> StringsValues;
    StringsValues                  stringsValues;

public: // Matrix Values
    mutable QMutex                matrixValuesMutex;
    typedef QHash<String, Matrix> MatrixValues;
    MatrixValues                  matrixValues;

public: // Images
    mutable QMutex               imageValuesMutex;
    typedef QHash<String, Image> ImageValues;
    ImageValues                  imageValues;

public: // Meshes
    mutable QMutex                      meshValuesMutex;
    typedef QHash<String, MeshConstPtr> MeshValues;
    MeshValues                          meshValues;

public: // Outlets
    void      attachOutlet (Outlet* outlet);
    void      detachOutlet (Outlet* outlet);
    void   subscribeOutlet (Outlet* outlet, String name);
    void unsubscribeOutlet (Outlet* outlet, String name);
    void       startOutlet (Outlet* outlet);
    void        stopOutlet (Outlet* outlet);

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
