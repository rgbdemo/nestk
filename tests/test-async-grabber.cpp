
#include <ntk/camera/freenect_grabber.h>
#include <ntk/camera/rgbd_processor.h>

#include <QApplication>

using namespace ntk;
using namespace cv;

class ImageHandler : public AsyncEventListener
{
public:
  ImageHandler(FreenectGrabber& grabber,
	       RGBDProcessor& processor) 
    : m_grabber(grabber),
      m_processor(processor)
  {
    namedWindow("color");
  }

public:
  // From AsyncUpdater. Will be called whener the published
  // has a new image.
  virtual void handleAsyncEvent(ntk::EventListener::Event) { handleNewImage(); }

  void handleNewImage()
  {
    m_grabber.copyImageTo(m_current_frame);
    m_processor.processImage(m_current_frame);
    int fps = m_grabber.frameRate();
    cv::putText(m_current_frame.rgbRef(),
                cv::format("%d fps", fps),
                Point(10,20), 0, 0.5, Scalar(255,0,0,255));

    // Display the image
    imshow("color", m_current_frame.rgb());
    cv::waitKey(10);
  }

private:
  FreenectGrabber& m_grabber;
  RGBDProcessor& m_processor;
  RGBDImage m_current_frame;
};

int main(int argc, char **argv)
{
  QApplication app(argc, argv);

  FreenectGrabber grabber;
  grabber.connectToDevice();

  RGBDProcessor processor;
  processor.setFilterFlag(RGBDProcessorFlags::ComputeKinectDepthBaseline, true);

  ImageHandler handler(grabber, processor);

  // Register image handler as a listener of grabbed data.
  grabber.addEventListener(&handler);

  // Start the grabbing thread.
  grabber.start();

  // Launch QT main loop. Handles the events.
  return app.exec();
}

