#include "image_window.h"
#include "ui_image_window.h"

namespace ntk
{

ImageWindowManager ImageWindowManager::instance;

ImageWindow::ImageWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ImageWindow)
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

ImageWindowManager *ImageWindowManager::getInstance()
{
    // if (!instance)
    //     instance = new ImageWindowManager;
    return &instance;
}

void ImageWindowManager::showImage(const std::string &window_name, const cv::Mat &im)
{
    ImageWindowManagerEventDataPtr data (new ImageWindowManagerEventData);
    data->window_name = window_name;
    im.copyTo(data->im);
    newEvent(this, data);
}

void ImageWindowManager::handleAsyncEvent(EventListener::Event event)
{
    ImageWindowManagerEventDataPtr data = dynamic_Ptr_cast<ImageWindowManagerEventData>(event.data);
    ntk_assert(data, "Invalid data type, should not happen");
    windows_map_type::const_iterator it = windows.find(data->window_name);
    ImageWindow* window = 0;
    if (it == windows.end())
    {
        window = new ImageWindow();
        windows[data->window_name] = window;
        window->setWindowTitle(data->window_name);
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

} // ntk
