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

#ifndef IMAGE_WINDOW_H
#define IMAGE_WINDOW_H

#include <ntk/thread/event.h>

#include <QImage>
#include <QString>
#include <QMainWindow>
#include <QMutex>

namespace ntk
{

struct PublishedImage
{
    QString name;
    QImage image;
};

struct PublishedImageEventData : public ntk::EventData
{
    TYPEDEF_THIS(PublishedImageEventData)

    CLONABLE_EVENT_DATA

    PublishedImage image;
};

ntk_ptr_typedefs(PublishedImageEventData)

//------------------------------------------------------------------------------

class ImagePublisher : public ntk::AsyncEventListener, public ntk::EventBroadcaster
{
public:
    static ImagePublisher* getInstance();

public:
    // void showImage(const std::string& window_name, const QImage& im);
    // void showImage(const std::string& window_name, const cv::Mat1f& im, double* min_val = 0, double* max_val = 0);
    // void showImage(const std::string& window_name, const cv::Mat1b& im);
    void publishImage (const std::string& image_name, const cv::Mat& im);

public:
    QImage getPublishedImage (QString name) const;

protected:
    virtual void handleAsyncEvent (Event event);

private:
    static ImagePublisher instance;
    typedef std::map<std::string, PublishedImage*> images_map_type;
    images_map_type published_images;
    QMutex lock;

private:
    struct ImageEventData : public ntk::EventData
    {
        TYPEDEF_THIS(ImageEventData)

        CLONABLE_EVENT_DATA

        std::string image_name;
        cv::Mat im;
    };
    ntk_ptr_typedefs(ImageEventData)
};

}

//------------------------------------------------------------------------------

namespace Ui {
class ImageWindow;
}

namespace ntk {
class ImageWidget;
}

//------------------------------------------------------------------------------

namespace ntk {

class ImageWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit ImageWindow(QWidget *parent = 0);
    ~ImageWindow();

    ntk::ImageWidget* imageWidget();
    
public slots:
    void onImageMouseMoved(int x, int y);

private:
    Ui::ImageWindow *ui;
};

//------------------------------------------------------------------------------

class DirectImageWindowManager : public ntk::AsyncEventListener, public ntk::EventBroadcaster
{
private:
    struct DirectImageWindowManagerEventData : public ntk::EventData
    {
        TYPEDEF_THIS(DirectImageWindowManagerEventData)

        CLONABLE_EVENT_DATA

        std::string window_name;
        cv::Mat im;
    };
    ntk_ptr_typedefs(DirectImageWindowManagerEventData)

public:
    static DirectImageWindowManager* getInstance();

    // void showImage(const std::string& window_name, const QImage& im);
    // void showImage(const std::string& window_name, const cv::Mat1f& im, double* min_val = 0, double* max_val = 0);
    // void showImage(const std::string& window_name, const cv::Mat1b& im);
    void showImage(const std::string& window_name, const cv::Mat& im);

protected:
    virtual void handleAsyncEvent(Event event);

private:
    static DirectImageWindowManager instance;
    typedef std::map<std::string, ImageWindow*> windows_map_type;
    windows_map_type windows;
};

//------------------------------------------------------------------------------

class ImageWindowManager : public EventListener
{
public:
    static ImageWindowManager* getInstance();

public:
     ImageWindowManager ();
    ~ImageWindowManager ();

public:
    void disable ();

protected:
    virtual void newEvent (EventBroadcaster* sender, EventDataPtr data);

private:
    bool disabled;
    static ImageWindowManager instance;
    typedef std::map<std::string, ImageWindow*> windows_map_type;
    windows_map_type windows;
};

}

#endif // IMAGE_WINDOW_H
