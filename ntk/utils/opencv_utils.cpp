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


#include "opencv_utils.h"
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/core/core.hpp>

#ifdef NESTK_USE_PCL
#include <pcl/io/lzf.h>
#endif /* NESTK_USE_PCL */

#include <fstream>

#include <QImage>
#include <QTemporaryFile>
#include <QDir>
#include <QByteArray>
#include <QFile>

using namespace cv;

const NtkDebug& operator<<(const NtkDebug& stream, const cv::Mat1d& m)
{
  stream << "\n";
  for (int r = 0; r < m.rows; ++r)
  {
    for (int c = 0; c < m.cols; ++c)
      stream << m(r,c) << " ";
    stream << "\n";
  }
  return stream;
}

const NtkDebug& operator<<(const NtkDebug& stream, const cv::Mat1f& m)
{
  stream << "\n";
  for (int r = 0; r < m.rows; ++r)
  {
    for (int c = 0; c < m.cols; ++c)
      stream << m(r,c) << " ";
    stream << "\n";
  }
  return stream;
}

namespace {

// Taken from OpenNI.
bool openni_compress_depth_16z(const unsigned short* pInput, const unsigned int nInputSize, unsigned char* pOutput, unsigned int* pnOutputSize)
{
    // Local function variables
    const unsigned short* pInputEnd = pInput + (nInputSize / sizeof(unsigned short));
    unsigned char* pOrigOutput = pOutput;
    unsigned short nCurrValue = 0;
    unsigned short nLastValue = 0;
    unsigned short nAbsDiffValue = 0;
    short nDiffValue = 0;
    unsigned char cOutStage = 0;
    unsigned char cOutChar = 0;
    unsigned char cZeroCounter = 0;

    // Note: this function does not make sure it stay within the output memory boundaries!

    if (nInputSize == 0)
    {
        *pnOutputSize = 0;
        return true;
    }

    // Encode the data...
    nLastValue = *pInput;
    *(unsigned short*)pOutput = nLastValue;
    pInput++;
    pOutput+=2;

    while (pInput != pInputEnd)
    {
        nCurrValue = *pInput;

        nDiffValue = (nLastValue - nCurrValue);
        nAbsDiffValue = (unsigned short)abs(nDiffValue);

        if (nAbsDiffValue <= 6)
        {
            nDiffValue += 6;

            if (cOutStage == 0)
            {
                cOutChar = (unsigned char)(nDiffValue << 4);

                cOutStage = 1;
            }
            else
            {
                cOutChar += (unsigned char)nDiffValue;

                if (cOutChar == 0x66)
                {
                    cZeroCounter++;

                    if (cZeroCounter == 15)
                    {
                        *pOutput = 0xEF;
                        pOutput++;

                        cZeroCounter = 0;
                    }
                }
                else
                {
                    if (cZeroCounter != 0)
                    {
                        *pOutput = 0xE0 + cZeroCounter;
                        pOutput++;

                        cZeroCounter = 0;
                    }

                    *pOutput = cOutChar;
                    pOutput++;
                }

                cOutStage = 0;
            }
        }
        else
        {
            if (cZeroCounter != 0)
            {
                *pOutput = 0xE0 + cZeroCounter;
                pOutput++;

                cZeroCounter = 0;
            }

            if (cOutStage == 0)
            {
                cOutChar = 0xFF;
            }
            else
            {
                cOutChar += 0x0F;
                cOutStage = 0;
            }

            *pOutput = cOutChar;
            pOutput++;

            if (nAbsDiffValue <= 63)
            {
                nDiffValue += 192;

                *pOutput = (unsigned char)nDiffValue;
                pOutput++;
            }
            else
            {
                *(unsigned short*)pOutput = (nCurrValue << 8) + (nCurrValue >> 8);
                pOutput+=2;
            }
        }

        nLastValue = nCurrValue;
        pInput++;
    }

    if (cOutStage != 0)
    {
        *pOutput = cOutChar + 0x0D;
        pOutput++;
    }

    if (cZeroCounter != 0)
    {
        *pOutput = 0xE0 + cZeroCounter;
        pOutput++;
    }

    *pnOutputSize = (unsigned int)(pOutput - pOrigOutput);

    // All is good...
    return true;
}

// Taken from OpenNI.
bool openni_uncompress_depth_16z(const unsigned char* pInput, const unsigned int nInputSize, unsigned short* pOutput, unsigned int* pnOutputSize)
{
    // Local function variables
    const unsigned char* pInputEnd = pInput + nInputSize;
    unsigned short* pOutputEnd = 0;
    const unsigned short* pOrigOutput = pOutput;
    unsigned short nLastFullValue = 0;
    unsigned char cInput = 0;
    unsigned char cZeroCounter = 0;
    char cInData1 = 0;
    char cInData2 = 0;
    unsigned char cInData3 = 0;

    if (nInputSize < sizeof(unsigned short))
    {
        ntk_dbg(0) << "Input size too small";
        return false;
    }

    pOutputEnd = pOutput + (*pnOutputSize / sizeof(unsigned short));

    // Decode the data...
    nLastFullValue = *(unsigned short*)pInput;
    *pOutput = nLastFullValue;
    pInput+=2;
    pOutput++;

    while (pInput != pInputEnd)
    {
        cInput = *pInput;

        if (cInput < 0xE0)
        {
            cInData1 = cInput >> 4;
            cInData2 = (cInput & 0x0f);

            nLastFullValue -= (cInData1 - 6);
            // XN_CHECK_OUTPUT_OVERFLOW(pOutput, pOutputEnd);
            *pOutput = nLastFullValue;
            pOutput++;

            if (cInData2 != 0x0f)
            {
                if (cInData2 != 0x0d)
                {
                    nLastFullValue -= (cInData2 - 6);
                    // XN_CHECK_OUTPUT_OVERFLOW(pOutput, pOutputEnd);
                    *pOutput = nLastFullValue;
                    pOutput++;
                }

                pInput++;
            }
            else
            {
                pInput++;

                cInData3 = *pInput;
                if (cInData3 & 0x80)
                {
                    nLastFullValue -= (cInData3 - 192);

                    // XN_CHECK_OUTPUT_OVERFLOW(pOutput, pOutputEnd);
                    *pOutput = nLastFullValue;

                    pOutput++;
                    pInput++;
                }
                else
                {
                    nLastFullValue = cInData3 << 8;
                    pInput++;
                    nLastFullValue += *pInput;

                    // XN_CHECK_OUTPUT_OVERFLOW(pOutput, pOutputEnd);
                    *pOutput = nLastFullValue;

                    pOutput++;
                    pInput++;
                }
            }
        }
        else if (cInput == 0xFF)
        {
            pInput++;

            cInData3 = *pInput;

            if (cInData3 & 0x80)
            {
                nLastFullValue -= (cInData3 - 192);

                // XN_CHECK_OUTPUT_OVERFLOW(pOutput, pOutputEnd);
                *pOutput = nLastFullValue;

                pInput++;
                pOutput++;
            }
            else
            {
                nLastFullValue = cInData3 << 8;
                pInput++;
                nLastFullValue += *pInput;

                // XN_CHECK_OUTPUT_OVERFLOW(pOutput, pOutputEnd);
                *pOutput = nLastFullValue;

                pInput++;
                pOutput++;
            }
        }
        else //It must be 0xE?
        {
            cZeroCounter = cInput - 0xE0;

            while (cZeroCounter != 0)
            {
                // XN_CHECK_OUTPUT_OVERFLOW(pOutput+1, pOutputEnd);
                *pOutput = nLastFullValue;
                pOutput++;

                *pOutput = nLastFullValue;
                pOutput++;

                cZeroCounter--;
            }

            pInput++;
        }
    }

    *pnOutputSize = (unsigned int)((pOutput - pOrigOutput) * sizeof(unsigned short));

    // All is good...
    return true;
}

}

namespace ntk
{

void extendToInclude(cv::Rect& rect, const cv::Point& p)
{
    if (rect.area() == 0)
        rect = cv::Rect(p.x, p.y, 1, 1);
    else
        rect |= cv::Rect(p.x, p.y, 1, 1);
}

bool leftRectFitIntoRight(const cv::Rect& left, const cv::Rect& right)
{
    if (left.width <= right.width && left.height <= right.height)
        return true;
    return false;
}

cv::Point3f computeCentroid(const std::vector<cv::Point3f>& points)
{
    cv::Point3f centroid (0,0,0);
    foreach_idx(i, points)
    {
        centroid = centroid + points[i];
    }
    centroid = centroid * (1.f/points.size());
    return centroid;
}

  cv::Mat4b toMat4b(const cv::Mat3b& im)
  {
    Mat4b out (im.size());
    for_all_rc(im)
    {
      Vec3b v = im(r,c);
      out(r,c) = Vec4b(v[0], v[1], v[2], 255);
    }
    return out;
  }

  cv::Mat3b toMat3b(const cv::Mat4b& im)
  {
    Mat3b out (im.size());
    for_all_rc(im)
    {
      Vec4b v = im(r,c);
      out(r,c) = Vec3b(v[0], v[1], v[2]);
    }
    return out;
  }

  cv::Mat1b qimage_to_opencv(const QImage& im)
  {
    QTemporaryFile f(QDir::tempPath() + "/ntk_XXXXXX.pgm");
    bool ok = f.open(); if (!ok) qFatal("Cannot create temporary file.");
    im.save(f.fileName(), "PGM");
    return cv::imread(f.fileName().toStdString(), 0);
  }

  void opencv_to_qimage(QImage& qim, const cv::Mat1b& im)
  {
    qim = QImage(im.cols, im.rows, QImage::Format_RGB32);
    for_all_rc(im)
    {
      int v = im(r,c);
      qim.setPixel(c,r,qRgb(v,v,v));
    }
  }

  void opencv_to_qimage(QImage& qim, const cv::Mat3b& im)
  {
    qim = QImage(im.cols, im.rows, QImage::Format_RGB32);
    for_all_rc(im)
    {
      cv::Vec3b color = im(r,c);
      qim.setPixel(c,r,qRgb(color[2],color[1],color[0]));
    }
  }

  void opencv_to_qimage(QImage& qim, const cv::Mat4b& im)
  {
    qim = QImage(im.cols, im.rows, QImage::Format_ARGB32);
    for_all_rc(im)
    {
      cv::Vec4b color = im(r,c);
      qim.setPixel(c,r,qRgba(color[2],color[1],color[0], color[3]));
    }
  }

  cv::Mat4b qimage_argb_to_opencv(const QImage& im)
  {
    cv::Mat4b cv_im (im.height(), im.width());
    for_all_rc(cv_im)
    {
      QRgb pixel = im.pixel(c, r);
      cv::Vec4b color (qBlue(pixel), qGreen(pixel), qRed(pixel), qAlpha(pixel));
      cv_im(r,c) = color;
    }
    return cv_im;
  }

  void apply_mask(cv::Mat1b& im, const cv::Mat1b& mask)
  {
    if (!mask.data)
      return;
    ntk_assert(im.size() == mask.size(), "Wrong mask size");
    for_all_rc(im)
      if (mask(r,c) == 0)
        im(r,c) = 0;
  }

  void apply_mask(cv::Mat3b& im, const cv::Mat1b& mask)
  {
    if (!mask.data)
      return;
    ntk_assert(im.size() == mask.size(), "Wrong mask size");
    for_all_rc(im)
      if (mask(r,c) == 0)
        im(r,c) = Vec3b(0,0,0);
  }

  void apply_mask(cv::Mat1f& im, const cv::Mat1b& mask)
  {
    if (!mask.data)
      return;
    ntk_assert(im.size() == mask.size(), "Wrong mask size");
    for_all_rc(im)
      if (mask(r,c) == 0)
        im(r,c) = 0.f;
  }

  void read_from_yaml(cv::FileNode node, bool& b)
  {
    ntk_throw_exception_if(node.empty(), "Could not read " + node.name() + " from yaml file.");
    int i = cvReadInt(*node, -1);
    ntk_assert(i >= 0 && i <= 1, "Invalid boolean value");
    b = i;
  }

  void write_to_yaml(cv::FileStorage& output_file, const std::string& name, bool b)
  {
    cvWriteInt(*output_file, name.c_str(), b);
  }

  void read_from_yaml(cv::FileNode node, int& i)
  {
    ntk_throw_exception_if(node.empty(), "Could not read " + node.name() + " from yaml file.");
    i = cvReadInt(*node, -1);
  }

  void write_to_yaml(cv::FileStorage& output_file, const std::string& name, int i)
  {
    cvWriteInt(*output_file, name.c_str(), i);
  }

  void read_from_yaml(cv::FileNode node, double& b)
  {
    ntk_throw_exception_if(node.empty(), "Could not read " + node.name() + " from yaml file.");
    b = cvReadReal(*node, 0);
  }

  void write_to_yaml(cv::FileStorage& output_file, const std::string& name, double b)
  {
    cvWriteReal(*output_file, name.c_str(), b);
  }

  void write_to_yaml(FileStorage& output_file, const std::string& name, const cv::Rect& r)
  {
    cv::Mat1f m(1,4);
    m << r.x, r.y, r.width, r.height;
    CvMat c_m = m;
    output_file.writeObj(name, &c_m);
  }

  void read_from_yaml(FileNode node, cv::Rect& r)
  {
    CvMat* c_m;
    c_m = (CvMat*)node.readObj();
    ntk_throw_exception_if(!c_m, std::string("Could not read field ") + node.name() + " from yml file.");
    cv::Mat1f m (c_m);
    ntk_assert(m.cols == 4, "Bad Rect.");
    r = cv::Rect(m(0,0), m(0,1), m(0,2), m(0,3));
  }

  void write_to_yaml(FileStorage& output_file, const std::string& name, const cv::Vec3f& v)
  {
    cv::Mat1f m(1,3);
    std::copy(&v[0], &v[0]+3, m.ptr<float>());
    CvMat c_m = m;
    output_file.writeObj(name, &c_m);
  }

  void write_to_yaml(FileStorage& output_file, const std::string& name, const cv::Vec2f& v)
  {
    cv::Mat1f m(1,2);
    std::copy(&v[0], &v[0]+2, m.ptr<float>());
    CvMat c_m = m;
    output_file.writeObj(name, &c_m);
  }

  void read_from_yaml(FileNode node, cv::Vec3f& v)
  {
    CvMat* c_m;
    c_m = (CvMat*)node.readObj();
    ntk_throw_exception_if(!c_m, std::string("Could not read field ") + node.name() + " from yml file.");
    cv::Mat1f m (c_m);
    ntk_assert(m.cols == 3, "Bad vector.");
    std::copy(m.ptr<float>(), m.ptr<float>()+3, &v[0]);
  }

  void read_from_yaml(FileNode node, cv::Vec2f& v)
  {
    CvMat* c_m;
    c_m = (CvMat*)node.readObj();
    ntk_throw_exception_if(!c_m, std::string("Could not read field ") + node.name() + " from yml file.");
    cv::Mat1f m (c_m);
    ntk_assert(m.cols == 2, "Bad vector.");
    std::copy(m.ptr<float>(), m.ptr<float>()+2, &v[0]);
  }

  void write_to_yaml(cv::FileStorage& output_file, const std::string& name, const cv::Mat& matrix)
  {
    CvMat m = matrix;
    output_file.writeObj(name, &m);
  }

  void read_from_yaml(cv::FileNode node, cv::Mat& matrix)
  {
    CvMat* m = (CvMat*)node.readObj();
    ntk_throw_exception_if(!m, std::string("Could not read field ") + node.name() + " from yml file.");
    matrix = m;
  }

  void writeMatrix(FileStorage& output_file, const std::string& name, const cv::Mat& matrix)
  {
    CvMat m = matrix;
    output_file.writeObj(name, &m);
  }

  void readMatrix(FileStorage& input_file, const std::string& name, cv::Mat& matrix)
  {
    FileNode node = input_file[name];
    CvMat* m;
    m = (CvMat*)node.readObj();
    ntk_throw_exception_if(!m, std::string("Could not read field ") + name + " from yml file.");
    matrix = m;
  }

  void imwrite_yml(const std::string& filename, const cv::Mat& image)
  {
    IplImage tmp = image;
    cvSave(filename.c_str(), &tmp);
  }

  cv::Mat1f imread_Mat1f_raw(const std::string& filename)
  {
    ntk_throw_exception_if(sizeof(float) != 4, "Cannot use raw with sizeof(float) != 4");
    ntk_throw_exception_if(QSysInfo::ByteOrder != QSysInfo::LittleEndian, "Cannot use raw with big endian");
    std::ifstream f (filename.c_str(), std::ios::binary);
    ntk_throw_exception_if(!f, "Could not open " + filename);
    qint32 rows = -1, cols = -1;
    f.read((char*)&rows, sizeof(qint32));
    f.read((char*)&cols, sizeof(qint32));
    cv::Mat1f m(rows, cols);
    f.read((char*)m.data, m.rows*m.cols*sizeof(float));
    ntk_throw_exception_if(f.bad(), "Failure reading " + filename + ": file too short.");
    return m;
  }

  cv::Mat1w imread_Mat1w_raw(const std::string& filename)
  {
    ntk_throw_exception_if(QSysInfo::ByteOrder != QSysInfo::LittleEndian, "Cannot use raw with big endian");
    std::ifstream f (filename.c_str(), std::ios::binary);
    ntk_throw_exception_if(!f, "Could not open " + filename);
    qint32 rows = -1, cols = -1;
    f.read((char*)&rows, sizeof(qint32));
    f.read((char*)&cols, sizeof(qint32));
    cv::Mat1w m(rows, cols);
    f.read((char*)m.data, m.rows*m.cols*sizeof(uint16_t));
    ntk_throw_exception_if(f.bad(), "Failure reading " + filename + ": file too short.");
    return m;
  }

  cv::Mat1w imread_Mat1w_lzf(const std::string& filename)
  {
#ifdef NESTK_USE_PCL
    ntk_throw_exception_if(QSysInfo::ByteOrder != QSysInfo::LittleEndian, "Cannot use raw with big endian");
    QFile f (filename.c_str());
    f.open (QIODevice::ReadOnly);
    QDataStream file_stream (&f);

    qint32 rows = -1, cols = -1;
    file_stream >> rows >> cols;

    unsigned int total_bytes = rows*cols*sizeof(uint16_t);
    QByteArray lzf_data (f.size(), 0);
    int nread_bytes = file_stream.readRawData(lzf_data.data(), f.size());
    lzf_data.truncate(nread_bytes);

    cv::Mat1w m (rows, cols);
    unsigned int nbytes = pcl::lzfDecompress (lzf_data.constData(), lzf_data.size(), m.ptr<char*>(), total_bytes);
    ntk_throw_exception_if (nbytes != total_bytes, "Could not decode the image");
    return m;
#else
    return cv::Mat1w(0, 0);
#endif /* NESTK_USE_PCL */
  }


  cv::Mat1w imread_Mat1w_grad_lzf(const std::string& filename)
  {
#ifdef NESTK_USE_PCL
    ntk_throw_exception_if(QSysInfo::ByteOrder != QSysInfo::LittleEndian, "Cannot use raw with big endian");
    QFile f (filename.c_str());
    f.open (QIODevice::ReadOnly);
    QDataStream file_stream (&f);

    qint32 rows = -1, cols = -1;
    file_stream >> rows >> cols;

    unsigned int total_bytes = rows*cols*sizeof(uint16_t);
    QByteArray lzf_data (f.size(), 0);
    int nread_bytes = file_stream.readRawData(lzf_data.data(), f.size());
    lzf_data.truncate(nread_bytes);

    cv::Mat1w m (rows, cols);
    cv::Mat1w grad (rows, cols);

    unsigned int nbytes = pcl::lzfDecompress (lzf_data.constData(), lzf_data.size(), grad.ptr<char*>(), total_bytes);
    ntk_throw_exception_if (nbytes != total_bytes, "Could not decode the image");

    uint16_t* cur = m.ptr<uint16_t>();
    const uint16_t* cur_grad = grad.ptr<uint16_t>();
    const uint16_t* end_grad = grad.ptr<uint16_t>(m.rows, m.cols);
    uint16_t prev = 0;
    while (cur_grad != end_grad)
    {
        int diff = *cur_grad % 2 == 0 ? *cur_grad / 2 : -(*cur_grad+1)/2;
        *cur = prev + diff;
        prev = *cur;
        ++cur;
        ++cur_grad;
    }

    return m;
#else
    return cv::Mat1w(0, 0);
#endif /* NESTK_USE_PCL */
  }

  cv::Mat1w imread_Mat1w_openni_lzf(const std::string& filename)
  {
#ifdef NESTK_USE_PCL
    ntk_throw_exception_if(QSysInfo::ByteOrder != QSysInfo::LittleEndian, "Cannot use raw with big endian");
    QFile f (filename.c_str());
    f.open (QIODevice::ReadOnly);
    QDataStream file_stream (&f);

    qint32 rows = -1, cols = -1;
    file_stream >> rows >> cols;

    unsigned int total_bytes = rows*cols*sizeof(uint16_t);
    QByteArray lzf_data (f.size(), 0);
    int nread_bytes = file_stream.readRawData(lzf_data.data(), f.size());
    lzf_data.truncate(nread_bytes);

    QByteArray openni_data (total_bytes * 2, 0);

    unsigned int nbytes = pcl::lzfDecompress (lzf_data.constData(), lzf_data.size(), openni_data.data(), total_bytes * 2);
    ntk_throw_exception_if (nbytes < 1, "Could not LZF uncompress the image");

    cv::Mat1w m (rows, cols);
    unsigned int ni_bytes = 0;
    bool openni_ok = openni_uncompress_depth_16z(reinterpret_cast<const unsigned char*>(openni_data.constData()),
                                                 nbytes,
                                                 m.ptr<unsigned short>(),
                                                 &ni_bytes);
    ntk_dbg_print (ni_bytes, 1);
    ntk_throw_exception_if (!openni_ok, "Could not OpenNI decode the image");
    ntk_throw_exception_if (ni_bytes != sizeof(uint16_t)*rows*cols, "Corrupted openni output");
    return m;
#else
    return cv::Mat1w(0, 0);
#endif /* NESTK_USE_PCL */
  }

  cv::Mat1w imread_Mat1w_openni(const std::string& filename)
  {
    ntk_throw_exception_if(QSysInfo::ByteOrder != QSysInfo::LittleEndian, "Cannot use raw with big endian");
    QFile f (filename.c_str());
    f.open (QIODevice::ReadOnly);
    QDataStream file_stream (&f);

    qint32 rows = -1, cols = -1;
    file_stream >> rows >> cols;

    unsigned int total_bytes = rows*cols*sizeof(uint16_t);
    QByteArray openni_data (f.size(), 0);
    int nread_bytes = file_stream.readRawData(openni_data.data(), f.size());
    openni_data.truncate(nread_bytes);

    cv::Mat1w m (rows, cols);
    unsigned int ni_bytes = 0;
    bool openni_ok = openni_uncompress_depth_16z(reinterpret_cast<const unsigned char*>(openni_data.constData()),
                                                 openni_data.size(),
                                                 m.ptr<unsigned short>(),
                                                 &ni_bytes);

    ntk_throw_exception_if (!openni_ok, "Could not OpenNI decode the image");
    ntk_throw_exception_if (ni_bytes != sizeof(uint16_t)*rows*cols, "Corrupted openni output");
    return m;
  }

  void imwrite_Mat1f_raw(const std::string& filename, const cv::Mat1f& m)
  {
    ntk_throw_exception_if(sizeof(float) != 4, "Cannot use raw with sizeof(float) != 4");
    ntk_throw_exception_if(QSysInfo::ByteOrder != QSysInfo::LittleEndian, "Cannot use raw with big endian");
    std::ofstream f (filename.c_str(), std::ios::binary);
    ntk_throw_exception_if(!f, "Could not open " + filename);
    qint32 rows = m.rows, cols = m.cols;
    f.write((char*)&rows, sizeof(qint32));
    f.write((char*)&cols, sizeof(qint32));
    f.write((char*)m.data, m.rows*m.cols*sizeof(float));
    ntk_throw_exception_if(f.bad(), "Failure writing " + filename);
  }

  void imwrite_Mat1w_raw(const std::string& filename, const cv::Mat1w& m)
  {
    ntk_throw_exception_if(QSysInfo::ByteOrder != QSysInfo::LittleEndian, "Cannot use raw with big endian");
    std::ofstream f (filename.c_str(), std::ios::binary);
    ntk_throw_exception_if(!f, "Could not open " + filename);
    qint32 rows = m.rows, cols = m.cols;
    f.write((char*)&rows, sizeof(qint32));
    f.write((char*)&cols, sizeof(qint32));
    f.write((char*)m.data, m.rows*m.cols*sizeof(uint16_t));
    ntk_throw_exception_if(f.bad(), "Failure writing " + filename);
  }

  void imwrite_Mat1w_lzf(const std::string& filename, const cv::Mat1w& m)
  {
#ifdef NESTK_USE_PCL
      ntk_throw_exception_if(QSysInfo::ByteOrder != QSysInfo::LittleEndian, "Cannot use raw with big endian");

      qint32 rows = m.rows, cols = m.cols;

      /* compressed data could be bigger, factor 2 to make sure it's safe. */
      int total_bytes = m.rows * m.cols * sizeof(uint16_t);
      QByteArray lzf_data (total_bytes * 2, 0u);
      unsigned int nbytes = pcl::lzfCompress(m.ptr<char*>(), total_bytes, lzf_data.data(), total_bytes * 2);
      ntk_throw_exception_if (nbytes == 0, "Could not compress stream.");
      lzf_data.truncate (nbytes);

      QFile f (filename.c_str());
      f.open (QIODevice::WriteOnly);
      QDataStream file_stream (&f);
      file_stream << rows << cols;
      file_stream.writeRawData(lzf_data.constData(), lzf_data.size());
      ntk_throw_exception_if (file_stream.status() != QDataStream::Ok, "Could not write to file.");
      f.close ();
#endif /* NESTK_USE_PCL */
  }

  void imwrite_Mat1w_grad_lzf(const std::string& filename, const cv::Mat1w& m)
  {
#ifdef NESTK_USE_PCL
      ntk_throw_exception_if(QSysInfo::ByteOrder != QSysInfo::LittleEndian, "Cannot use raw with big endian");

      qint32 rows = m.rows, cols = m.cols;

      cv::Mat1w grad (m.size());
      grad = (uint16_t) 0;
      const uint16_t* cur = m.ptr<uint16_t>();
      uint16_t* cur_grad = grad.ptr<uint16_t>();
      const uint16_t* end = m.ptr<uint16_t>(m.rows, m.cols);
      uint16_t prev = 0;
      while (cur != end)
      {
          int diff = *cur - prev;
          if (diff < 0) *cur_grad = 2*std::abs(diff)-1;
          else *cur_grad = 2*diff;
          prev = *cur;
          ++cur;
          ++cur_grad;
      }

      /* compressed data could be bigger, factor 2 to make sure it's safe. */
      int total_bytes = m.rows * m.cols * sizeof(uint16_t);
      QByteArray lzf_data (total_bytes * 2, 0u);
      unsigned int nbytes = pcl::lzfCompress(grad.ptr<char*>(), total_bytes, lzf_data.data(), total_bytes * 2);
      ntk_throw_exception_if (nbytes == 0, "Could not compress stream.");
      lzf_data.truncate (nbytes);

      QFile f (filename.c_str());
      f.open (QIODevice::WriteOnly);
      QDataStream file_stream (&f);
      file_stream << rows << cols;
      file_stream.writeRawData(lzf_data.constData(), lzf_data.size());
      ntk_throw_exception_if (file_stream.status() != QDataStream::Ok, "Could not write to file.");
      f.close ();
#endif /* NESTK_USE_PCL */
  }

  void imwrite_Mat1w_openni_lzf(const std::string& filename, const cv::Mat1w& m)
  {
#ifdef NESTK_USE_PCL
      ntk_throw_exception_if(QSysInfo::ByteOrder != QSysInfo::LittleEndian, "Cannot use raw with big endian");

      qint32 rows = m.rows, cols = m.cols;
      int total_bytes = m.rows * m.cols * sizeof(uint16_t);

      QByteArray openni_data (total_bytes * 2, 0u);

      unsigned int openni_compressed_size = 0;
      bool ok = openni_compress_depth_16z (m.ptr<unsigned short>(), total_bytes,
                                           reinterpret_cast<unsigned char*>(openni_data.data()),
                                           &openni_compressed_size);

      ntk_throw_exception_if (!ok, "OpenNI compression failed.");

      QByteArray lzf_data (openni_compressed_size * 2, 0u);
      unsigned int nbytes = pcl::lzfCompress(openni_data.constData(),
                                             openni_compressed_size,
                                             lzf_data.data(),
                                             openni_compressed_size * 2);
      ntk_throw_exception_if (nbytes == 0, "Could not compress stream.");
      lzf_data.truncate (nbytes);

      QFile f (filename.c_str());
      f.open (QIODevice::WriteOnly);
      QDataStream file_stream (&f);
      file_stream << rows << cols;
      file_stream.writeRawData(lzf_data.constData(), lzf_data.size());
      ntk_throw_exception_if (file_stream.status() != QDataStream::Ok, "Could not write to file.");
      f.close ();
#endif /* NESTK_USE_PCL */
  }

  void imwrite_Mat1w_openni(const std::string& filename, const cv::Mat1w& m)
  {
      ntk_throw_exception_if(QSysInfo::ByteOrder != QSysInfo::LittleEndian, "Cannot use raw with big endian");

      qint32 rows = m.rows, cols = m.cols;
      int total_bytes = m.rows * m.cols * sizeof(uint16_t);

      QByteArray openni_data (total_bytes * 2, 0u);

      unsigned int openni_compressed_size = 0;
      bool ok = openni_compress_depth_16z (m.ptr<unsigned short>(), total_bytes,
                                           reinterpret_cast<unsigned char*>(openni_data.data()),
                                           &openni_compressed_size);

      ntk_throw_exception_if (!ok, "OpenNI compression failed.");
      openni_data.truncate (openni_compressed_size);

      QFile f (filename.c_str());
      f.open (QIODevice::WriteOnly);
      QDataStream file_stream (&f);
      file_stream << rows << cols;
      file_stream.writeRawData(openni_data.constData(), openni_data.size());
      ntk_throw_exception_if (file_stream.status() != QDataStream::Ok, "Could not write to file.");
      f.close ();
  }

  cv::Mat imread_yml(const std::string& filename)
  {
    IplImage* tmp = (IplImage*) cvLoad(filename.c_str());
    ntk_throw_exception_if(!tmp, "Could not load " + filename);
    return cv::Mat(tmp);
  }

  cv::Mat3b toMat3b(const cv::Mat1b& image)
  {
    cv::Mat3b tmp;
    cv::cvtColor(image, tmp, CV_GRAY2BGR);
    return tmp;
  }

  cv::Mat1b normalize_toMat1b(const cv::Mat1f& image)
  {
    cv::Mat1b tmp;
    normalize(image, tmp, 0, 255, NORM_MINMAX, 0);
    return tmp;
  }

  void imwrite_normalized(const std::string& filename, const cv::Mat1b& image)
  {
      cv::Mat1b tmp;
      normalize(image, tmp, 0, 255, NORM_MINMAX, 0);
      imwrite(filename, tmp);
  }

  void imwrite_normalized(const std::string& filename, const cv::Mat1f& image)
  {
    cv::Mat1b tmp;
    normalize(image, tmp, 0, 255, NORM_MINMAX, 0);
    imwrite(filename, tmp);
  }

  void imshow_normalized(const std::string& window_name, const cv::Mat1f& image)
  {
    cv::Mat1b tmp;
    normalize(image, tmp, 0, 255, NORM_MINMAX, 0);
    imshow(window_name, tmp);
  }

  double overlap_ratio(const cv::Rect_<float>& r1, const cv::Rect_<float>& r2)
  {
    cv::Rect_<float> union12  = r1 | r2;

    cv::Rect_<float> intersection12 = r1 & r2;
    return intersection12.area()/union12.area();
  }

  void adjustRectToImage(cv::Rect& rect, const cv::Size& image_size)
  {
    rect.x = std::max(rect.x, 0);
    rect.y = std::max(rect.y, 0);
    rect.x = std::min(rect.x, image_size.width-1);
    rect.y = std::min(rect.y, image_size.height-1);
    rect.width = std::min(image_size.width-rect.x-1, rect.width);
    rect.height = std::min(image_size.height-rect.y-1, rect.height);
  }


  ntk::Rect3f bounding_box(const std::vector<cv::Point3f>& points)
  {
    ntk::Rect3f box;
    foreach_idx(i, points)
    {
      box.extendToInclude(points[i]);
    }
    return box;
  }

  ntk::Rect3f readBoundingBoxFromYamlFile(const std::string& filename)
  {
      QFileInfo f (filename.c_str());
      ntk_throw_exception_if(!f.exists(), "Could not find bounding box file.");
      cv::FileStorage cv_file (filename, CV_STORAGE_READ);
      cv::Mat1f mat (2,3);
      readMatrix(cv_file, "bounding_box", mat);
      return ntk::Rect3f(mat(0,0), mat(0,1), mat(0,2),
                         mat(1,0), mat(1,1), mat(1,2));
  }

  void writeBoundingBoxToYamlFile(const std::string& filename, const ntk::Rect3f& bbox)
  {
      FileStorage output_file (filename, CV_STORAGE_WRITE);
      cv::Mat1f mat(2,3);
      mat(0,0) = bbox.x;
      mat(0,1) = bbox.y;
      mat(0,2) = bbox.z;
      mat(1,0) = bbox.width;
      mat(1,1) = bbox.height;
      mat(1,2) = bbox.depth;
      writeMatrix(output_file, "bounding_box", mat);
  }

  float triangleArea(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3)
  {
      float tmp = p1.x*p2.y - p1.x*p3.y + p2.x*p3.y - p2.x*p1.y + p3.x*p1.y - p3.x*p2.y;
      return 0.5f * std::abs(tmp);
  }

  float triangleArea(const cv::Point3f& p1, const cv::Point3f& p2, const cv::Point3f& p3)
  {
      cv::Point3f p21 = p2 - p1;
      cv::Point3f p31 = p3 - p1;
      return 0.5f*cv::norm(p21.cross(p31));
  }

}
