#pragma once

#include "hub/update.h"
#include "gui/image_widget.h"
#include <QImage>

namespace ntk { namespace hub {

class Hub::ImageUpdate : public Hub::Update
{
public:
    ImageUpdate (QString name);

public:
    virtual void updateHub    (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

private:
    virtual void updateHubImage (QImage& hubImage) = 0;

private:
    QImage hubImage;
};

//------------------------------------------------------------------------------

class Hub::SetImageUpdate : public Hub::ImageUpdate
{
public:
    SetImageUpdate (QString name, QImage image);

private:
    virtual void updateHubImage (QImage& hubImage);

private:
    const QImage image;
};

//------------------------------------------------------------------------------

class Hub::SetOpenCVImageUpdate : public Hub::ImageUpdate
{
public:
    SetOpenCVImageUpdate (QString name, cv::Mat mat);

private:
    virtual void updateHubImage (QImage& hubImage);

private:
    const cv::Mat mat;
};

//------------------------------------------------------------------------------

class Hub::ClearImageUpdate : public Hub::ImageUpdate
{
public:
    ClearImageUpdate (QString name);

private:
    virtual void updateHubImage (QImage& hubImage);
};

} }
