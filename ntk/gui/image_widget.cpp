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


#include "image_widget.h"

#include <ntk/ntk.h>

#include <QMouseEvent>
#include <QPainter>

using namespace cv;

namespace ntk
{

ImageWidget::ImageWidget (QWidget* parent)
: QWidget(parent)
, keep_ratio(false)
, m_last_mouse_pos(-1,-1)
{
}

void ImageWidget :: mouseMoveEvent ( QMouseEvent * event )
{
    if (m_image.isNull())
    {
        event->ignore();
        return;
    }

    int x = event->x() * (m_image.width() / float(width()));
    int y = event->y() * (m_image.height() / float(height()));
    m_last_mouse_pos = QPoint(x,y);
    emit mouseMoved(x,y);
}

void ImageWidget :: setImage(const QImage& im)
{
    m_image = im;
    update();
}

void ImageWidget :: setRects(const std::list<cv::Rect>& rects, const cv::Vec3b& color)
{
    m_rects = rects;
    m_rect_color = color;
    update();
}

void ImageWidget :: setTexts(const std::vector<TextData> texts)
{
    m_texts = texts;
    update();
}

bool
ImageWidget :: setImage(QImage& image, const cv::Mat1b& im)
{
    bool geometryUpdated = false;

    if (image.width() != im.cols
            || image.height() != im.rows)
    {
        image = QImage(im.cols, im.rows, QImage::Format_RGB32);
        geometryUpdated = true;
    }

    for (int r = 0; r < im.rows; ++r)
    {
        QRgb* ptr = (QRgb*) image.scanLine(r);
        for (int c = 0; c < im.cols; ++c)
        {
            int v = im(r,c);
            *ptr = qRgb(v,v,v);
            ++ptr;
        }
    }

    return geometryUpdated;
}

bool
ImageWidget :: setImage(QImage& image, const cv::Mat1f& im, double* i_min_val, double* i_max_val)
{
    bool geometryUpdated = false;

    if (image.width() != im.cols
            || image.height() != im.rows)
    {
        image = QImage(im.cols, im.rows, QImage::Format_RGB32);
        geometryUpdated = true;
    }

    double min_val, max_val;
    if (i_min_val && i_max_val)
    {
        min_val = *i_min_val;
        max_val = *i_max_val;
    }
    else
        minMaxLoc(im, &min_val, &max_val);
    if (min_val == max_val)
    {
        image.fill(qRgb(0,0,0));
        return geometryUpdated;
    }

    for (int r = 0; r < im.rows; ++r)
    {
        QRgb* ptr = (QRgb*) image.scanLine(r);
        const float* cv_ptr = im.ptr<float>(r);
        for (int c = 0; c < im.cols; ++c)
        {
            int v = 255*(*cv_ptr-min_val)/(max_val-min_val);
            v = ntk::saturate_to_range(v, 0, 255);
            int rgb = (0xff << 24) + (v << 16) + (v << 8) + v;
            *ptr = rgb;
            ++ptr;
            ++cv_ptr;
        }
    }

    return geometryUpdated;
}

bool
ImageWidget :: setImage(QImage& image, const cv::Mat3b& im)
{
    bool geometryUpdated = false;

    if (image.isNull()
            || image.width() != im.cols
            || image.height() != im.rows)
    {
        image = QImage(im.cols, im.rows, QImage::Format_RGB32);
        geometryUpdated = true;
    }

    for (int r = 0; r < im.rows; ++r)
    {
        QRgb* ptr = (QRgb*) image.scanLine(r);
        const uchar* cv_ptr = im.ptr(r);
        for (int i = 0; i < im.cols; ++i)
        {
            int rgb = 0xff << 24;
            rgb |= (*cv_ptr++);
            rgb |= ((*cv_ptr++) << 8);
            rgb |= ((*cv_ptr++) << 16);
            *ptr++ = rgb;
        }
    }

    return geometryUpdated;
}

void ImageWidget :: setImage(const cv::Mat1b& im)
{
    if (setImage(m_image, im))
        updateGeometry();
    
    update();
}

void ImageWidget :: setImage(const cv::Mat1f& im, double* i_min_val, double* i_max_val)
{
    if (setImage(m_image, im, i_min_val, i_max_val))
        updateGeometry();

    update();
}

void ImageWidget :: setImage(const cv::Mat3b& im)
{
    if (setImage(m_image, im))
        updateGeometry();

    update();
}

double ImageWidget :: scaleX() const
{
    return double(rect().width())/m_image.rect().width();
}

double ImageWidget :: scaleY() const
{
    return double(rect().height())/m_image.rect().height();
}

void ImageWidget :: setPen(QPen pen)
{
    m_pen = pen;
}

void ImageWidget :: paintEvent(QPaintEvent * event)
{
    double sx = scaleX();
    double sy = scaleY();

    if (m_pen.data_ptr() == NULL){
        QPen p;
        m_pen = p;
        m_pen.setColor(Qt::white);
        m_pen.setWidth(2);
    }

    QPainter painter(this);

    // FIXME: The image rect should be cached and recomputed on resize events.

    painter.drawImage(imageRect(), m_image, m_image.rect());

    m_pen.setColor(qRgb(m_rect_color[0], m_rect_color[1], m_rect_color[2]));
    painter.setPen(m_pen);
    foreach_const_it(it, m_rects, std::list<cv::Rect>)
    {
        const cv::Rect& r = *it;
        QRect qr (r.x*sx, r.y*sy, r.width*sx, r.height*sy);
        painter.drawRect(qr);
    }

    foreach_idx(i, m_texts)
    {
        const cv::Vec3b& c = m_texts[i].color;
        m_pen.setColor(qRgb(c[0], c[1], c[2]));
        painter.setFont(QFont("Helvetica", 14));
        painter.setPen(m_pen);
        QString s (m_texts[i].text.c_str());
        QPoint p (m_texts[i].x*sx, m_texts[i].y*sy);
        painter.drawText(p, s);
    }
}

void
ImageWidget::setRatioKeeping (bool ratio_keeping)
{
    if (keep_ratio == ratio_keeping)
        return;

    keep_ratio = ratio_keeping;

    if (keep_ratio)
    {
        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        sizePolicy.setHeightForWidth(true);
        setSizePolicy(sizePolicy);
    }
    else
    {
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHeightForWidth(false);
        setSizePolicy(sizePolicy);
    }

    updateGeometry();
}

QSize ImageWidget :: sizeHint() const
{
    if (!keep_ratio || m_image.isNull() || m_image.width() * m_image.height() == 0)
        return QWidget::sizeHint();

    return QSize(m_image.width(), m_image.height());
}

QSize ImageWidget :: minimumSizeHint () const
{
    if (!keep_ratio || m_image.isNull() || m_image.width() * m_image.height() == 0)
        return QWidget::minimumSizeHint();

    return QSize(m_image.width() / 8, m_image.height() / 8);
}

int ImageWidget :: heightForWidth(int width) const
{
    if (!keep_ratio || m_image.isNull() || m_image.width() * m_image.height() == 0)
        return QWidget::heightForWidth(width);

    return m_image.height() / m_image.width() * width;
}

QRect ImageWidget :: imageRect () const
{
    if (!keep_ratio || m_image.isNull() || m_image.width() * m_image.height() == 0)
        return rect();

    const QRect widgetRect = rect();

    const float widgetRatio = float(widgetRect.height()) / float(widgetRect.width());
    const float imageRatio  = float(m_image   .height()) / float(m_image   .width());

    QRect ret = widgetRect;

    if (imageRatio < widgetRatio)
    {
        const int extraHeight = widgetRect.height() - int(widgetRect.width() * imageRatio);

        ret.adjust(0, extraHeight / 2, 0, -extraHeight / 2);
    }
    else if (widgetRatio < imageRatio)
    {
        const int extraWidth = widgetRect.width() - int(widgetRect.height() / imageRatio);

        ret.adjust(extraWidth / 2, 0, -extraWidth / 2, 0);
    }

    return ret;
}

} // ntk
