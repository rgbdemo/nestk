#ifndef IMAGE_WINDOW_H
#define IMAGE_WINDOW_H

#include <ntk/thread/event.h>

#include <QMainWindow>

namespace Ui {
class ImageWindow;
}

namespace ntk {
class ImageWidget;
}

namespace ntk
{

class ImageWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit ImageWindow(QWidget *parent = 0);
    ~ImageWindow();

    ntk::ImageWidget* imageWidget();
    
private:
    Ui::ImageWindow *ui;
};

class ImageWindowManager : public ntk::AsyncEventListener, public ntk::EventBroadcaster
{
private:
    struct ImageWindowManagerEventData : public ntk::EventData
    {
        std::string window_name;
        cv::Mat im;
    };
    ntk_ptr_typedefs(ImageWindowManagerEventData)

public:
    static ImageWindowManager* getInstance();

    // void showImage(const std::string& window_name, const QImage& im);
    // void showImage(const std::string& window_name, const cv::Mat1f& im, double* min_val = 0, double* max_val = 0);
    // void showImage(const std::string& window_name, const cv::Mat1b& im);
    void showImage(const std::string& window_name, const cv::Mat& im);

protected:
    virtual void handleAsyncEvent(Event event);

private:
    static ImageWindowManager instance;
    typedef std::map<std::string, ImageWindow*> windows_map_type;
    windows_map_type windows;
};

}

#endif // IMAGE_WINDOW_H
