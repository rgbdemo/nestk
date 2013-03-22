#ifndef NTK_UTILS_PROGRESSIVE_H
#define NTK_UTILS_PROGRESSIVE_H

#include <ntk/core.h>

#include <QObject>

namespace ntk
{

class ProgressiveImpl : public QObject
{
Q_OBJECT
public:
    //! Send a progress message, percent and optionally give an identifier.
    virtual void progress(const char* message, float percent, const char* id = "unknown") const;

signals:
    void progressChanged(QString, float, QString) const;
};

/*!
 * Objects whose progress can be followed. This class does not inherit directly
 * from QObject to avoid multiple inheritance issues with QObject.
 */
class Progressive
{
public:
    ProgressiveImpl* progressObject() const { return &impl; }

    virtual void progress(const char* message, float percent, const char* id = "unknown") const
    {
        progressObject()->progress(message, percent, id);
    }

private:
    mutable ProgressiveImpl impl;
};

}  // ntk

#endif // NTK_UTILS_PROGRESSIVE_H
