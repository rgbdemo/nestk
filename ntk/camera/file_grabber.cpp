/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
 */


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
