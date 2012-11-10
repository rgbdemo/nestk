#include "image-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include "hub/hub-impl.h"
#include "gui/image_widget.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::ImageUpdate::ImageUpdate (String name)
: Hub::Update(name)
{

}

void
Hub::ImageUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.impl->imageValuesMutex);

    image = hub.impl->imageValues[name];

    _.unlock();

    updateImage(image);

    _.relock();

    hub.impl->imageValues[name] = image;
}

void
Hub::ImageUpdate::updateOutlet (Outlet& outlet)
{
    outlet.onImageChanged(name, image);
}

//------------------------------------------------------------------------------

Hub::SetImageUpdate::SetImageUpdate (String name, const Image& newImage)
: ImageUpdate(name)
, newImage(newImage)
{

}

void
Hub::SetImageUpdate::updateImage (Image& image)
{
    image = newImage;
}

//------------------------------------------------------------------------------

Hub::SetMatrixImageUpdate::SetMatrixImageUpdate (String name, const Matrix& matrix)
: Hub::ImageUpdate(name)
, matrix(matrix)
{

}

void
Hub::SetMatrixImageUpdate::updateImage (Image& image)
{
    // FIXME: Depend on opencv_utils functions instead of the clumsy ImageWidget static methods.

    switch (matrix.type())
    {
    case CV_MAT_TYPE(CV_8UC3):
        ImageWidget::setImage(image, cv::Mat3b(matrix));
        break;

    case CV_MAT_TYPE(CV_8UC1):
        ImageWidget::setImage(image, cv::Mat1b(matrix));
        break;

    case CV_MAT_TYPE(CV_32FC1):
        ImageWidget::setImage(image, cv::Mat1f(matrix));
        break;

    default:
        ntk_dbg(0) << "Unsupported image type";
    }
}

//------------------------------------------------------------------------------

Hub::ClearImageUpdate::ClearImageUpdate (String name)
: Hub::ImageUpdate(name)
{

}

void
Hub::ClearImageUpdate::updateImage (Image& image)
{
    image = Image();
}

} }
