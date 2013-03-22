
#include "screen_grabber.h"

namespace ntk
{

ScreenGrabber :: ScreenGrabber(const std::string& dir_name)
  : m_frame_count(0), m_dir(dir_name.c_str())
{
  m_dir.mkpath(".");
}

void ScreenGrabber :: reset()
{
  m_frame_count = 0;
}

void ScreenGrabber :: saveFrame(const QPixmap& pixmap)
{
  pixmap.save(m_dir.absoluteFilePath(QString("frame%1.png").arg(m_frame_count, 4, 10, QChar('0'))));
  ++m_frame_count;
}

} // ntk
