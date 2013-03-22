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
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
 */

#include "image_window.h"
#include "ui_image_window.h"

#include <ntk/utils/time.h>

#include <QMutexLocker>
#include <QHash>

namespace ntk
{

ImagePublisher ImagePublisher::instance;

ImagePublisher *ImagePublisher::getInstance()
{
    // if (!instance)
    //     instance = new DirectImageWindowManager;
    return &instance;
}

void ImagePublisher::publishImage(const std::string &image_name, const cv::Mat &im)
{
    ImageEventDataPtr data (new ImageEventData);
    data->image_name = image_name;
    im.copyTo(data->im);

    // FIXME: Trick the event system into believing we have one sender per image name.
    EventBroadcaster* fakeSender = reinterpret_cast<EventBroadcaster*>(qHash(QByteArray(image_name.c_str())));

    newEvent(fakeSender, data);
}

QImage
ImagePublisher::getPublishedImage (QString name) const
{
    const QMutexLocker _(const_cast<QMutex*>(&lock));

    images_map_type::const_iterator it = published_images.find(name.toStdString());

    if (it == published_images.end())
    {
        qWarning() << "No such published image: " << name;
        return QImage();
    }

    return it->second->image;
}

void ImagePublisher::handleAsyncEvent(EventListener::Event event)
{
    ImageEventDataPtr internalData = dynamic_Ptr_cast<ImageEventData>(event.data);
    if (!internalData)
    {
        ntk_dbg(0) << "Invalid data type in handleAsyncEvent, should not happen.";
        return;
    }
    ntk_assert(internalData, "Invalid data type, should not happen");

    PublishedImage* publishedImage = 0;
    {
        const QMutexLocker _(&lock);

        images_map_type::const_iterator it = published_images.find(internalData->image_name);


        if (it == published_images.end())
        {
            publishedImage = new PublishedImage();
            published_images[internalData->image_name] = publishedImage;
            publishedImage->name = QString::fromStdString(internalData->image_name);
        }
        else
        {
            publishedImage = it->second;
        }
    }

    // publishedImage->image = internalData->image;

    switch (internalData->im.type())
    {
    case CV_MAT_TYPE(CV_8UC3): {
        cv::Mat3b mat_ = internalData->im;
        ImageWidget::setImage(publishedImage->image, mat_);
        break;
    }

    case CV_MAT_TYPE(CV_8UC1): {
        cv::Mat1b mat_ = internalData->im;
        ImageWidget::setImage(publishedImage->image, mat_);
        break;
    }

    case CV_MAT_TYPE(CV_32FC1): {
        cv::Mat1f mat_ = internalData->im;
        ImageWidget::setImage(publishedImage->image, mat_);
        break;
    }

        //default:
        //    ntk_dbg(0) << "Unsupported image type";
    }

    PublishedImageEventDataPtr data(new PublishedImageEventData());

    data->image = *publishedImage;

    broadcastEvent(data);
}

//------------------------------------------------------------------------------

ImageWindow::ImageWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::ImageWindow)
{
    ui->setupUi(this);
}

ImageWindow::~ImageWindow()
{
    delete ui;
}

ntk::ImageWidget *ImageWindow::imageWidget()
{
    return ui->centralwidget;
}

void ImageWindow::onImageMouseMoved(int x, int y)
{
    ui->statusbar->showMessage(QString("(%1, %2)").arg(x).arg(y));
}

//------------------------------------------------------------------------------

DirectImageWindowManager DirectImageWindowManager::instance;

DirectImageWindowManager *DirectImageWindowManager::getInstance()
{
    // if (!instance)
    //     instance = new DirectImageWindowManager;
    return &instance;
}

void DirectImageWindowManager::showImage(const std::string &window_name, const cv::Mat &im)
{
    DirectImageWindowManagerEventDataPtr data (new DirectImageWindowManagerEventData);
    data->window_name = window_name;
    im.copyTo(data->im);
    // FIXME: hacky. Make sure the sender url are different for each window to avoid
    // compressing multiple signals into one.
    newEvent(this + qHash(QByteArray(window_name.c_str())), data);
}

void DirectImageWindowManager::handleAsyncEvent(EventListener::Event event)
{
    DirectImageWindowManagerEventDataPtr data = dynamic_Ptr_cast<DirectImageWindowManagerEventData>(event.data);
    ntk_assert(data, "Invalid data type, should not happen");
    windows_map_type::const_iterator it = windows.find(data->window_name);
    ImageWindow* window = 0;
    if (it == windows.end())
    {
        window = new ImageWindow();
        windows[data->window_name] = window;
        window->setWindowTitle(data->window_name.c_str());
        connect(window->imageWidget(), SIGNAL(mouseMoved(int, int)), window, SLOT(onImageMouseMoved(int,int)));
        window->imageWidget()->setRatioKeeping(true);
    }
    else
    {
        window = it->second;
    }

    switch (data->im.type())
    {
    case CV_MAT_TYPE(CV_8UC3): {
        cv::Mat3b mat_ = data->im;
        window->imageWidget()->setImage(mat_);
        break;
    }

    case CV_MAT_TYPE(CV_8UC1): {
        cv::Mat1b mat_ = data->im;
        window->imageWidget()->setImage(mat_);
        break;
    }

    case CV_MAT_TYPE(CV_32FC1): {
        cv::Mat1f mat_ = data->im;
        window->imageWidget()->setImage(mat_);
        break;
    }

    default:
        ntk_dbg(0) << "Unsupported image type";
    }

    window->show();
}

//------------------------------------------------------------------------------

ImageWindowManager ImageWindowManager::instance;

ImageWindowManager *ImageWindowManager::getInstance()
{
    // if (!instance)
    //     instance = new ImageWindowManager;
    return &instance;
}

ImageWindowManager::ImageWindowManager()
: disabled(false)
{
    ImagePublisher::getInstance()->addEventListener (this);
}

ImageWindowManager::~ImageWindowManager()
{
    // FIXME: Make this work.
    // ImagePublisher::getInstance()->removeEventListener(this);
}

void ImageWindowManager::disable ()
{
    disabled = true;
}

void ImageWindowManager::newEvent (EventBroadcaster* sender, EventDataPtr data)
{
    if (disabled)
        return;

    PublishedImageEventDataPtr publishedImageData = dynamic_Ptr_cast<PublishedImageEventData>(data);
    ntk_assert(publishedImageData, "Invalid data type, should not happen");
    windows_map_type::const_iterator it = windows.find(publishedImageData->image.name.toStdString());
    ImageWindow* window = 0;
    if (it == windows.end())
    {
        window = new ImageWindow();
        windows[publishedImageData->image.name.toStdString()] = window;
        window->setWindowTitle(publishedImageData->image.name);
        QWidget::connect(window->imageWidget(), SIGNAL(mouseMoved(int, int)), window, SLOT(onImageMouseMoved(int,int)));
        window->imageWidget()->setRatioKeeping(true);
    }
    else
    {
        window = it->second;
    }

    window->imageWidget()->setImage(publishedImageData->image.image);

    window->show();
}

namespace {

    struct ImageWindowManagerInstance
    {
        ImageWindowManagerInstance ()
        {
            ImageWindowManager::getInstance();
        }
    };

    ImageWindowManagerInstance imageWindowManagerInstance;
}

} // ntk
