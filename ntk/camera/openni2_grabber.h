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

#ifndef NTK_CAMERA_OPENNI2_GRABBER_H
# define NTK_CAMERA_OPENNI2_GRABBER_H

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

#endif // !NTK_CAMERA_OPENNI2_GRABBER_H
