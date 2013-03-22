
#include <ntk/ntk.h>
#include <ntk/utils/debug.h>
#include <ntk/camera/openni_grabber.h>
#include <ntk/gesture/body_event.h>
#include <ntk/gesture/skeleton.h>

#include <QApplication>
#include <QDir>
#include <QMutex>

using namespace cv;
using namespace ntk;

namespace opt
{
  ntk::arg<bool> high_resolution("--highres", "High resolution color image.", 0);
  ntk::arg<int> kinect_id("--kinect-id", "Kinect id", 0);
}

class MyBodyListener : public BodyEventListener
{
public:
  MyBodyListener() : m_last_hand_pos_2d(0,0,0)
  {

  }

  virtual void triggerEvent(const BodyEvent &event)
  {
    switch (event.kind)
    {
    case BodyEvent::PushEvent:
      ntk_dbg(1) << "PushEvent detected!";
      break;
    case BodyEvent::WaveEvent:
      ntk_dbg(1) << "WaveEvent detected!";
      break;
    case BodyEvent::SteadyEvent:
      ntk_dbg(1) << "SteadyEvent detected!";
      break;
    default:
      ntk_dbg_print(event, 1);
    };
  }

  virtual void triggerHandPoint(const HandPointUpdate &hand_point)
  {
    QMutexLocker locker(&m_lock);
    m_last_hand_pos_2d = hand_point.pos_2d;
  }

  cv::Point3f getLastHandPosInImage() const
  {
    QMutexLocker locker(&m_lock);
    return m_last_hand_pos_2d;
  }

private:
  mutable QMutex m_lock;
  Point3f m_last_hand_pos_2d;
};

int main(int argc, char **argv)
{
  // Parse command line options.
  arg_base::set_help_option("-h");
  arg_parse(argc, argv);

  // Set debug level to 1.
  ntk::ntk_debug_level = 1;

  // Set current directory to application directory.
  // This is to find Nite config in config/ directory.
  // FIXME: this is disabled for OpenCV QT compatibility.
  // QApplication app (argc, argv);
  // QDir::setCurrent(QApplication::applicationDirPath());

  // Prepare body event listeners.
  MyBodyListener body_event_listener;
  BodyEventDetector detector;
  detector.addListener(&body_event_listener);

  // Declare the global OpenNI driver. Only one can be instantiated in a program.
  OpenniDriver ni_driver;

  // Declare the frame grabber.
  OpenniGrabber grabber(ni_driver, opt::kinect_id());
  grabber.setBodyEventDetector(&detector);

  // High resolution 1280x1024 RGB Image.
  if (opt::high_resolution())
    grabber.setHighRgbResolution(true);

  // Start the grabber.
  grabber.connectToDevice();
  grabber.start();

  // Holder for the current image.
  RGBDImage image;

  // Image post processor. Compute mappings when RGB resolution is 1280x1024.
  OpenniRGBDProcessor post_processor;

  namedWindow("depth");
  namedWindow("color");
  namedWindow("users");

  while (true)
  {
    // Wait for a new frame, get a local copy and postprocess it.
    grabber.waitForNextFrame();
    grabber.copyImageTo(image);
    post_processor.processImage(image);

    // Get the last hand point position.
    cv::Point3f handpoint = body_event_listener.getLastHandPosInImage();
    ntk_dbg_print(handpoint, 1);

    // Prepare the depth view, with skeleton and handpoint.
    cv::Mat1b debug_depth_img = normalize_toMat1b(image.depth());
    if (image.skeleton())
      image.skeleton()->drawOnImage(debug_depth_img);
    circle(debug_depth_img, Point(handpoint.x, handpoint.y), 5, Scalar(0, 255, 255));

    // Prepare the color view, with skeleton and handpoint.
    cv::Mat3b debug_color_img;
    image.mappedRgb().copyTo(debug_color_img);
    if (image.skeleton())
      image.skeleton()->drawOnImage(debug_color_img);
    circle(debug_color_img, Point(handpoint.x, handpoint.y), 5, Scalar(0, 255, 255));

    // Prepare the user mask view as colors.
    cv::Mat3b debug_users;
    image.fillRgbFromUserLabels(debug_users);

    imshow("depth", debug_depth_img);
    imshow("color", debug_color_img);
    imshow("users", debug_users);
    cv::waitKey(10);
  }

  // return app.exec();
}
