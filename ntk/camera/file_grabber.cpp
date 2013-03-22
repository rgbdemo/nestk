
#include "file_grabber.h"

#include <ntk/utils/time.h>
#include <ntk/utils/stl.h>

#include <fstream>

namespace ntk
{

FileGrabber::FileGrabber(const std::string& path, bool is_directory)
    : RGBDGrabber(),
      m_path(path.c_str()),
      m_current_image_index(0),
      m_is_directory(is_directory),
      m_grabber_type("unknown")
{
    if (!is_directory)
    {
        m_rgbd_image.loadFromDir(path);
        m_rgbd_image.setDirectory(path);
        m_grabber_type = m_rgbd_image.grabberType();
    }
    else
    {
        m_image_list = m_path.entryList(QStringList("view????*"), QDir::Dirs, QDir::Name);
        ntk_ensure(!m_image_list.empty(), "No view???? images in given directory.");
        std::string grabber_file = m_path.absoluteFilePath(m_image_list[0] + "/grabber-type").toStdString();
        if (ntk::is_file(grabber_file))
        {
            std::ifstream f (grabber_file.c_str());
            f >> m_grabber_type;
            f.close();
        }
    }

    setCameraSerial(QDir(path.c_str()).dirName().toStdString());

    // Loop by default.
    m_loop = false;
}

void FileGrabber::run()
{
    m_rgbd_image.setCalibration(m_calib_data);
    m_buffer_image.setCalibration(m_calib_data);

    m_rgbd_image.setCameraSerial(cameraSerial());
    m_buffer_image.setCameraSerial(cameraSerial());

    while (!threadShouldExit())
    {
        waitForNewEvent(-1); // Use infinite timeout in order to honor sync mode.

        if (m_current_image_index >= m_image_list.size())
        {
            if (!m_loop)
            {
                ntk::sleep(50);
                continue;
            }
            else
            {
                m_current_image_index = 0;
            }
        }


        if (m_is_directory)
        {
            QString filepath = m_path.absoluteFilePath(m_image_list[m_current_image_index]);
            ntk_dbg(1) << "Reading " << filepath;
            {
                QWriteLocker locker(&m_lock);
                m_rgbd_image.loadFromDir(filepath.toStdString(), m_calib_data);
                m_rgbd_image.setTimestamp(getCurrentTimestamp());
            }
            ++m_current_image_index;
            ntk::sleep(10);
        }
        else
        {
            ntk::sleep(30);
        }
        advertiseNewFrame();
    }
}

} // ntk
