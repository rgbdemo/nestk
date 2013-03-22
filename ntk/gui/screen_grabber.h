#ifndef NTK_GUI_SCREENGRABBER_H
#define NTK_GUI_SCREENGRABBER_H

#include <ntk/core.h>

#include <QPixmap>
#include <QDir>

namespace ntk
{

class ScreenGrabber
{
public:
  ScreenGrabber(const std::string& dir_name);

public:
  void reset();
  void saveFrame(const QPixmap& pixmap);

private:
  int m_frame_count;
  QDir m_dir;
};

} // ntk

#endif // NTK_GUI_SCREENGRABBER_H
