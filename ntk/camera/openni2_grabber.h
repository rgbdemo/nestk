#pragma once

#include <ntk/camera/rgbd_grabber.h>
#include <vector>
#include <QString>

namespace openni {
    class DeviceInfo;
}

//------------------------------------------------------------------------------

namespace ntk {

class Openni2Driver
{
public:
    static bool hasDll ();

public:
     Openni2Driver ();
    ~Openni2Driver ();

public:
    struct SensorInfo
    {
        SensorInfo (const openni::DeviceInfo& info);

        bool operator < (const SensorInfo& rhs) const
        {
            return key < rhs.key;
        }

        QString uri;
        QString vendor;
        QString name;
        quint16 vendorId;
        quint16 productId;

    private:
        QString key;
    };

public:
    typedef std::vector<SensorInfo> SensorInfos;
    void getSensorInfos (SensorInfos& sensorInfos) const;
    bool isReady () const;
    QString getLastError () const;

public:
    struct Impl;
    Impl* impl;
};

//------------------------------------------------------------------------------

class Openni2Grabber : public ntk::RGBDGrabber
{
public:
    explicit Openni2Grabber (Openni2Driver& driver, QString uri = QString());
    ~Openni2Grabber ();

    virtual std::string grabberType () const { return "openni2"; }

    virtual bool      connectToDevice ();
    virtual bool disconnectFromDevice ();

    virtual void setIRMode (bool enabled);

    void setSubsamplingFactor (int factor);
    void setHighRgbResolution (bool enabled);
    void setMirrored (bool enabled);
    void setCustomBayerDecoding (bool enabled);
    void setUseHardwareRegistration (bool enabled);

protected:
    virtual void run();

private:
    struct Impl;
    Impl* impl;
};

}
