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

#ifndef NTK_GUI_IMAGEWIDGET_H
#define NTK_GUI_IMAGEWIDGET_H

#include <ntk/core.h>

#include <QPoint>
#include <QImage>
#include <QWidget>
#include <QPen>

class QMouseEvent;
class QPaintEvent;
class QPen;
namespace ntk
{

class ImageWidget : public QWidget
{
    Q_OBJECT

public:
    struct TextData
    {
        std::string text;
        int x, y;
        float size;
        cv::Vec3b color;
    };

public: // FIXME: Move these where they really belong.
    static bool setImage(QImage& image, const cv::Mat1f& im, double* min_val = 0, double* max_val = 0);
    static bool setImage(QImage& image, const cv::Mat1b& im);
    static bool setImage(QImage& image, const cv::Mat3b& im);

public:
    ImageWidget(QWidget* parent);

public:
    double scaleX() const;
    double scaleY() const;

    void getLastMousePos(int& x, int& y)
    { x = m_last_mouse_pos.x(); y = m_last_mouse_pos.y(); }

    void setImage(const QImage& im);
    void setImage(const cv::Mat1f& im, double* min_val = 0, double* max_val = 0);
    void setImage(const cv::Mat1b& im);
    void setImage(const cv::Mat3b& im);

    void setPen(QPen q);
    void setRects(const std::list<cv::Rect>& rects, const cv::Vec3b& color = cv::Vec3b(0,0,0));
    void setTexts(const std::vector<TextData> texts);

signals:
    void mouseMoved(int image_x, int image_y);

protected:
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void paintEvent(QPaintEvent* event);
    virtual QSize        sizeHint() const;
    virtual QSize minimumSizeHint() const;
    virtual int heightForWidth(int width) const;

public:
    void setRatioKeeping (bool ratio_keeping = true);

private:
    QRect imageRect () const;

private:
    bool keep_ratio;

private:
    QPoint m_last_mouse_pos;
    QImage m_image;
    QPen m_pen;
    std::list<cv::Rect> m_rects;
    cv::Vec3b m_rect_color;
    std::vector<TextData> m_texts;
};

} // ntk

#endif // NTK_GUI_IMAGEWIDGET_H
