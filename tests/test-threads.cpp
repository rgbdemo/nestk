
#include <ntk/utils/time.h>
#include <QThread>

using namespace ntk;

cv::RNG rng;

struct Functor : public QThread
{
  virtual void run()
  {
    ntk::sleep(rng(5000));
    std::cout << "hello world" << std::endl;
  }
};

int main()
{
  static const int n_threads = 100;

  Functor functors[n_threads];

  for (int i = 0; i < n_threads; ++i)
  {
    functors[i].start();
  }

  for (int i = 0; i < n_threads; ++i)
  {
    functors[i].wait();
  }
  return 0;
}
