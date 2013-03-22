
#include <ntk/ntk.h>
#include <ntk/geometry/similarity_transform.h>

#include <QTransform>

using namespace ntk;

int main()
{
  ntk::ntk_debug_level = 1;

  cv::Point2f p (50,10);
  QPointF qt_p (p.x,p.y);

  QTransform qt_transform;
  ntk_dbg_print(qt_transform.map(qt_p), 1);
  qt_transform = qt_transform.scale(5,5);
  ntk_dbg_print(qt_transform.map(qt_p), 1);
  qt_transform = qt_transform.translate(10,-1);
  ntk_dbg_print(qt_transform.map(qt_p), 1);
  qt_transform = qt_transform.rotate(1.0);
  ntk_dbg_print(qt_transform.map(qt_p), 1);
  qt_transform = qt_transform.scale(1/5.,1/5.);
  ntk_dbg_print(qt_transform.map(qt_p), 1);
  qt_transform = qt_transform.translate(-1,1);
  ntk_dbg_print(qt_transform.map(qt_p), 1);
  qt_transform = qt_transform.scale(7,7);
  ntk_dbg_print(qt_transform.map(qt_p), 1);
  qt_transform = qt_transform.rotate(-2.0);
  ntk_dbg_print(qt_transform.map(qt_p), 1);

  SimilarityTransform transform;
  ntk_dbg_print(transform.transform(p), 1);
  transform.applyTransformBefore(cv::Vec2f(0,0), 0, 5);
  ntk_dbg_print(transform.transform(p), 1);
  transform.applyTransformBefore(cv::Vec2f(10,-1), 0, 1);
  ntk_dbg_print(transform.transform(p), 1);
  transform.applyTransformBefore(cv::Vec2f(0,0), 1, 1);
  ntk_dbg_print(transform.transform(p), 1);
  transform.applyTransformBefore(cv::Vec2f(0,0), 0, 1/5.);
  ntk_dbg_print(transform.transform(p), 1);
  transform.applyTransformBefore(cv::Vec2f(-1,1), 0, 1);
  ntk_dbg_print(transform.transform(p), 1);
  transform.applyTransformBefore(cv::Vec2f(0,0), 0, 7);
  ntk_dbg_print(transform.transform(p), 1);
  transform.applyTransformBefore(cv::Vec2f(0,0), -2, 1);
  ntk_dbg_print(transform.transform(p), 1);

  return 0;
}
