#include "image-update.h"
#include "hub/outlet.h"
#include "hub/hub.h"
#include <QMutexLocker>

namespace ntk { namespace hub {

Hub::ImageUpdate::ImageUpdate (QString name)
: Hub::Update(name)
{

}

void
Hub::ImageUpdate::updateHub (Hub& hub)
{
    QMutexLocker _(&hub.imagesMutex);

    hubImage = hub.images[name];

    _.unlock();

    updateHubImage(hubImage);

    _.relock();

    hub.images[name] = hubImage;
}

void
Hub::ImageUpdate::updateOutlet (Outlet& outlet)
{
    outlet.changeImage(name, hubImage);
}

//------------------------------------------------------------------------------

Hub::SetImageUpdate::SetImageUpdate (QString name, QImage image)
: ImageUpdate(name)
, image(image)
{

}

void
Hub::SetImageUpdate::updateHubImage (QImage& hubImage)
{
    hubImage = image;
}

//------------------------------------------------------------------------------

Hub::SetImageMatrixUpdate::SetImageMatrixUpdate (QString name, cv::Mat mat)
: Hub::ImageUpdate(name)
, mat(mat)
{

}

void
Hub::SetImageMatrixUpdate::updateHubImage (QImage& hubImage)
{
    // FIXME: Depend on opencv_utils functions instead of the clumsy ImageWidget static methods.

    switch (mat.type())
    {
    case CV_MAT_TYPE(CV_8UC3):
        ImageWidget::setImage(hubImage, cv::Mat3b(mat));
        break;

    case CV_MAT_TYPE(CV_8UC1):
        ImageWidget::setImage(hubImage, cv::Mat1b(mat));
        break;

    case CV_MAT_TYPE(CV_32FC1):
        ImageWidget::setImage(hubImage, cv::Mat1f(mat));
        break;

    default:
        ntk_dbg(0) << "Unsupported image type";
    }
}

//------------------------------------------------------------------------------

Hub::ClearImageUpdate::ClearImageUpdate (QString name)
: Hub::ImageUpdate(name)
{

}

void
Hub::ClearImageUpdate::updateHubImage (QImage& hubImage)
{
    hubImage = QImage();
}

} }
