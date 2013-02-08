#pragma once

#include <ntk/camera/rgbd_grabber.h>
#include <OpenNI2/OpenNI.h>

namespace ntk
{

class Openni2Driver
{
public:
    struct DeviceInfo
    {
        std::string creation_info;
        std::string camera_type;
        std::string serial;
        std::string vendor;
        unsigned short vendor_id;
        unsigned short product_id;
        unsigned char bus;
        unsigned char address;
    };

public:
    Openni2Driver();
    ~Openni2Driver();

public:
    int numDevices() const;
    const DeviceInfo& deviceInfo(int index) const;

public:
    static bool hasDll ();
};

class Openni2Grabber : public ntk::RGBDGrabber
{
public:
    Openni2Grabber(Openni2Driver& driver, int camera_id = 0);
    Openni2Grabber(Openni2Driver& driver, const std::string& camera_serial);

    virtual std::string grabberType () const { return "openni2"; }

    /*! Call it before starting the thread. */
    virtual bool connectToDevice();

    /*! Disconnect from Kinect. */
    virtual bool disconnectFromDevice();

    /*! Set whether color images should be in high resolution 1280x1024. */
    void setHighRgbResolution(bool hr) { m_high_resolution = hr; }

    /*! Set an optional subsampling factor for the depth image. */
    void setSubsamplingFactor(int factor);

    /*! Set whether images should be vertically mirrored. */
    void setMirrored(bool m) { m_mirrored = m; }

    /*! Grab IR images instead of RGB images. */
    virtual void setIRMode(bool ir);

    /*! Set whether custom bayer decoding should be used. */
    void setCustomBayerDecoding(bool enable) { m_custom_bayer_decoding = enable; }

    /*! Set whether hardware registraition should be used */
    void setUseHardwareRegistration(bool enable) { m_hardware_registration = enable; }

protected:
    /*! Thread loop. */
    virtual void run();

private:
    Openni2Driver& m_driver;
    int m_camera_id;
    std::string m_camera_serial;
    RGBDImage m_current_image;
    int m_subsampling_factor;
    bool m_high_resolution;
    bool m_mirrored;
    bool m_custom_bayer_decoding;
    bool m_hardware_registration;
    bool m_get_infrared;
    bool m_has_rgb;
};

}
