#pragma once

#include "hub/update.h"
#include <QImage>

namespace ntk { namespace hub {

class Hub::ImageUpdate : public Hub::Update
{
public:
    ImageUpdate (String name);

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
    SetImageUpdate (String name, const Image& newImage);

private:
    virtual void updateImage (Image& image);

private:
    const Image newImage;
};

//------------------------------------------------------------------------------

class Hub::SetMatrixImageUpdate : public Hub::ImageUpdate
{
public:
    SetMatrixImageUpdate (QString name, const Matrix& matrix);

private:
    virtual void updateImage (Image& image);

private:
    const Matrix matrix;
};

//------------------------------------------------------------------------------

class Hub::ClearImageUpdate : public Hub::ImageUpdate
{
public:
    ClearImageUpdate (String name);

private:
    virtual void updateImage (Image& image);
};

} }
