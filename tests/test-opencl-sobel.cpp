#include <opencv2/core/core.hpp>
#include <ntk/gpu/opencl.h>

using namespace cv;

int main()
{
  cv::Mat3f im = imread("test.png");
  ntk::CL cl;
  cl.setup();
  cl::Kernel kernel = cl.loadKernel("sobel.cl", "sobel_rgb");

  try {

  cl::ImageFormat format (CL_RGBA, CL_UNORM_INT8);
  cl::Image2D image_in (cl.context,
                        CL_MEM_READ_ONLY,
                        format,
                        im.cols,
                        im.rows);
  cl::Image2D image_out (cl.context,
                         CL_MEM_WRITE_ONLY,
                         format,
                         im.cols,
                         im.rows);

  kernel.setArg(0, image_in);
  kernel.setArg(1, image_out);

  cl::size_t<3> origin; origin[0] = origin[1] = origin[2] = 0;
  cl::size_t<3> region; region[0] = im.cols; region[1] = im.rows; region[2] = 1;
      int err = cl.queue.enqueueReadImage(image_in, CL_TRUE, origin, region, 0, 0, im.ptr<void>());
      printf("cl::enqueueReadImage: %s\n", ntk::oclErrorString(err));
  }
  catch (cl::Error er)
  {
      printf("ERROR: %s(%s)\n", er.what(), ntk::oclErrorString(er.err()));
  }
  return 0;
}
