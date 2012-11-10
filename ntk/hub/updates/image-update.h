#pragma once

#include "hub/update.h"
#include <QImage>

namespace ntk { namespace hub {

class Hub::ImageUpdate : public Hub::Update
{
public:
    ImageUpdate (Name name);

public:
    virtual void updateHub    (Hub& hub);
    virtual void updateOutlet (Outlet& outlet);

private:
    virtual void updateImage (Image& image) = 0;

private:
    Image image;
};

//------------------------------------------------------------------------------

class Hub::SetImageUpdate : public Hub::ImageUpdate
{
public:
    SetImageUpdate (Name name, const Image& newImage);

private:
    virtual void updateImage (Image& image);

private:
    const Image newImage;
};

//------------------------------------------------------------------------------

class Hub::SetImageMatrixUpdate : public Hub::ImageUpdate
{
public:
    SetImageMatrixUpdate (QString name, const Matrix& matrix);

private:
    virtual void updateImage (Image& image);

private:
    const Matrix matrix;
};

//------------------------------------------------------------------------------

class Hub::ClearImageUpdate : public Hub::ImageUpdate
{
public:
    ClearImageUpdate (Name name);

private:
    virtual void updateImage (Image& image);
};

} }
