#include "common/qt/image_frame/if_overlay.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <QtDebug>

///////////////////////////
// ImageFrameOverlayBase //
///////////////////////////

ImageFrameOverlayBase::ImageFrameOverlayBase() :
    ImageSize(0, 0),
    DrawOffset(0, 0),
    DrawScale(1),
    visible(false),
    pickable(false),
    editable(false)
{
}

ImageFrameOverlayBase::ImageFrameOverlayBase(bool visible_, bool pickable_, bool editable_) :
    ImageSize(0, 0),
    DrawOffset(0, 0),
    DrawScale(1),
    visible(visible_),
    pickable(pickable_),
    editable(editable_)
{
}

ImageFrameOverlayBase::~ImageFrameOverlayBase()
{
}

QPointF ImageFrameOverlayBase::ToImageCoord(const QPointF& frm_pos) const
{
    return (frm_pos - DrawOffset) / DrawScale;
}

QRectF ImageFrameOverlayBase::ToImageCoord(const QRectF& frm_rect) const
{
    return QRectF(ToImageCoord(frm_rect.topLeft()), ToImageScale(frm_rect.size()));
}

float ImageFrameOverlayBase::ToImageScale(float frm_length) const
{
    return frm_length / DrawScale;
}

QSizeF ImageFrameOverlayBase::ToImageScale(const QSizeF& frm_size) const
{
    return frm_size / DrawScale;
}

QPointF ImageFrameOverlayBase::ToFrameCoord(const QPointF& image_pos) const
{
    return DrawOffset + DrawScale * image_pos;
}

QRectF ImageFrameOverlayBase::ToFrameCoord(const QRectF& image_rect) const
{
    return QRectF(ToFrameCoord(image_rect.topLeft()), ToFrameScale(image_rect.size()));
}

float ImageFrameOverlayBase::ToFrameScale(float image_length) const
{
    return DrawScale * image_length;
}

QSizeF ImageFrameOverlayBase::ToFrameScale(const QSizeF& image_size) const
{
    return DrawScale * image_size;
}

bool ImageFrameOverlayBase::ClipToImage(const QPointF& image_pos, QPointF& clipped_pos) const
{
    if (ImageSize.width() < 1 || ImageSize.height() < 1) return false;
    clipped_pos = image_pos;
    if (clipped_pos.x() < 0) clipped_pos.setX(0);
    if (clipped_pos.y() < 0) clipped_pos.setY(0);
    if (clipped_pos.x() > ImageSize.width() - 1) clipped_pos.setX(ImageSize.width() - 1);
    if (clipped_pos.y() > ImageSize.height() - 1) clipped_pos.setY(ImageSize.height() - 1);
    return true;
}

bool ImageFrameOverlayBase::ClipToImage(const QRectF& image_rect, QRectF& clipped_rect, bool allow_resize) const
{
    if (ImageSize.width() < 1 || ImageSize.height() < 1) return false;
    clipped_rect = image_rect;
    if  (allow_resize) {
        if (clipped_rect.x() < 0) clipped_rect.setX(0);
        if (clipped_rect.x() > ImageSize.width() - 1) clipped_rect.setX(ImageSize.width() - 1);
        if (clipped_rect.y() < 0) clipped_rect.setY(0);
        if (clipped_rect.y() > ImageSize.height() - 1) clipped_rect.setY(ImageSize.height() - 1);
        if (clipped_rect.right() < 0) clipped_rect.setRight(0);
        if (clipped_rect.right() > ImageSize.width() - 1) clipped_rect.setRight(ImageSize.width() - 1);
        if (clipped_rect.bottom() < 0) clipped_rect.setBottom(0);
        if (clipped_rect.bottom() > ImageSize.height() - 1) clipped_rect.setBottom(ImageSize.height() - 1);
    } else {
        if (clipped_rect.x() < 0.0f) clipped_rect.moveLeft(0.0f);
        if (clipped_rect.y() < 0.0f) clipped_rect.moveTop(0.0f);
        if (clipped_rect.right() >= ImageSize.width() - 1) clipped_rect.moveRight(ImageSize.width() - 1);
        if (clipped_rect.bottom() >= ImageSize.height() - 1) clipped_rect.moveBottom(ImageSize.height() - 1);
    }
    return true;
}

void ImageFrameOverlayBase::RGBtoHSV(float r, float g, float b, float& h, float& s, float& v)
{
    float rgb_max;
    int max_ch;

    if (r > g) {
        if (b > r) {
            rgb_max = b;
            max_ch = 3;
        } else {
            rgb_max = r;
            max_ch = 1;
        }
    } else {
        if (b > g) {
            rgb_max = b;
            max_ch = 3;
        } else {
            rgb_max = g;
            max_ch = 2;
        }
    }

    float rgb_min = std::min(std::min(r, g), b);
    float rgb_range = rgb_max - rgb_min;

    if (rgb_range > 0.0f) {
        if (max_ch == 1) {
            h = 60.0f * (::fmod(((g - b) / rgb_range), 6.0f));
        } else if(max_ch == 2) {
            h = 60.0f * (((b - r) / rgb_range) + 2.0f);
        } else if(max_ch == 3) {
            h = 60.0f * (((r - g) / rgb_range) + 4.0f);
        }

        if(rgb_max > 0) {
            s = rgb_range / rgb_max;
        } else {
            s = 0.0f;
        }

        v = rgb_max;
    } else {
        h = 0.0f;
        s = 0.0f;
        v = rgb_max;
    }

    if (h < 0) {
        h = 360.0f + h;
    }
}

void ImageFrameOverlayBase::HSVtoRGB(float h, float s, float v, float& r, float& g, float& b)
{
    float chroma = v * s;
    float prime = ::fmod(h / 60.0f, 6.0f);
    float x = chroma * (1.0f - ::fabs(::fmod(prime, 2.0f) - 1.0f));
    float m = v - chroma;

    if (0.0f <= prime && prime < 1.0f) {
        r = chroma;
        g = x;
        b = 0.0f;
    } else if (1.0f <= prime && prime < 2.0f) {
        r = x;
        g = chroma;
        b = 0.0f;
    } else if (2.0f <= prime && prime < 3.0f) {
        r = 0.0f;
        g = chroma;
        b = x;
    } else if (3.0f <= prime && prime < 4.0f) {
        r = 0.0f;
        g = x;
        b = chroma;
    } else if (4.0f <= prime && prime < 5.0f) {
        r = x;
        g = 0.0f;
        b = chroma;
    } else if (5.0f <= prime && prime < 6.0f) {
        r = chroma;
        g = 0.0f;
        b = x;
    } else {
        r = 0.0f;
        g = 0.0f;
        b = 0.0f;
    }

    r += m;
    g += m;
    b += m;
}

QVector<QPointF> ImageFrameOverlayBase::HandlePositions() const
{
    return QVector<QPointF>();
}

QPointF ImageFrameOverlayBase::HandlePosition(int handle_id) const
{
    return QPointF(-1, -1);
}

bool ImageFrameOverlayBase::onSelect(int handle_id)
{
    return false;
}

bool ImageFrameOverlayBase::onDeselect()
{
    return false;
}

bool ImageFrameOverlayBase::onHandleGrab(int handle_id)
{
    return false;
}

bool ImageFrameOverlayBase::onHandleMove(int handle_id, const QPointF& frm_pos)
{
    return false;
}

bool ImageFrameOverlayBase::onHandleRelease(int handle_id)
{
    return false;
}

//////////////////////
// IfOverlayEllipse //
//////////////////////

IfOverlayEllipse::IfOverlayEllipse() :
    ImageFrameOverlayBase(false),
    center(0, 0),
    radius_x(0),
    radius_y(0),
    angle(0),
    line_thickness(1),
    line_color(0, 0, 0),
    fill_color(255, 255, 255),
    draw_line(true),
    draw_fill(false)
{
}

IfOverlayEllipse::IfOverlayEllipse(const QPointF& center_, float radius_, const QColor& line_color_, float line_thickness_) :
    ImageFrameOverlayBase(true),
    center(center_),
    radius_x(radius_),
    radius_y(radius_),
    angle(0),
    line_thickness(line_thickness_),
    line_color(line_color_),
    fill_color(255, 255, 255),
    draw_line(true),
    draw_fill(false)
{
}

IfOverlayEllipse::IfOverlayEllipse(const QPointF& center_, float radius_, const QColor& fill_color_) :
    ImageFrameOverlayBase(true),
    center(center_),
    radius_x(radius_),
    radius_y(radius_),
    angle(0),
    line_thickness(1),
    line_color(0, 0, 0),
    fill_color(fill_color_),
    draw_line(false),
    draw_fill(true)
{
}

IfOverlayEllipse::IfOverlayEllipse(const QPointF& center_, float radius_, const QColor& line_color_, float line_thickness_, const QColor& fill_color_) :
    ImageFrameOverlayBase(true),
    center(center_),
    radius_x(radius_),
    radius_y(radius_),
    angle(0),
    line_thickness(line_thickness_),
    line_color(line_color_),
    fill_color(fill_color_),
    draw_line(true),
    draw_fill(true)
{
}

IfOverlayEllipse::IfOverlayEllipse(const QPointF& center_, float radius_x_, float radius_y_, float angle_, const QColor& line_color_, float line_thickness_) :
    ImageFrameOverlayBase(true),
    center(center_),
    radius_x(radius_x_),
    radius_y(radius_y_),
    angle(angle_),
    line_thickness(line_thickness_),
    line_color(line_color_),
    fill_color(255, 255, 255),
    draw_line(true),
    draw_fill(false)
{
}

IfOverlayEllipse::IfOverlayEllipse(const QPointF& center_, float radius_x_, float radius_y_, float angle_, const QColor& fill_color_) :
    ImageFrameOverlayBase(true),
    center(center_),
    radius_x(radius_x_),
    radius_y(radius_y_),
    angle(angle_),
    line_thickness(1),
    line_color(0, 0, 0),
    fill_color(fill_color_),
    draw_line(false),
    draw_fill(true)
{
}

IfOverlayEllipse::IfOverlayEllipse(const QPointF& center_, float radius_x_, float radius_y_, float angle_, const QColor& line_color_, float line_thickness_, const QColor& fill_color_) :
    ImageFrameOverlayBase(true),
    center(center_),
    radius_x(radius_x_),
    radius_y(radius_y_),
    angle(angle_),
    line_thickness(line_thickness_),
    line_color(line_color_),
    fill_color(fill_color_),
    draw_line(true),
    draw_fill(true)
{
}

void IfOverlayEllipse::Draw(QPainter* painter) const
{
    if (painter) {
        painter->save();

        painter->setPen(draw_line ? QPen(line_color, line_thickness) : Qt::NoPen);
        painter->setBrush(draw_fill ? QBrush(fill_color) : Qt::NoBrush);

        QPointF frm_center = ToFrameCoord(center);

        if (std::abs(angle) >= 0.001f && std::abs(radius_x - radius_y) > 0.001) {
            painter->translate(frm_center.x(), frm_center.y());
            painter->rotate(angle);
            painter->translate(-frm_center.x(), -frm_center.y());
        }

        painter->drawEllipse(frm_center, ToFrameScale(radius_x), ToFrameScale(radius_y));

        painter->restore();
    }
}

////////////////////////
// IfOverlayRectangle //
////////////////////////

IfOverlayRectangle::IfOverlayRectangle() :
    ImageFrameOverlayBase(false),
    rect(0, 0, 0, 0),
    angle(0),
    line_thickness(1),
    line_color(0, 0, 0),
    fill_color(255, 255, 255),
    draw_line(true),
    draw_fill(false),
    draw_handles(false),
    handles_size(4.0f),
    handles_color(line_color)
{
}

IfOverlayRectangle::IfOverlayRectangle(const QRectF& rect_, const QColor& line_color_, float line_thickness_) :
    ImageFrameOverlayBase(true),
    rect(rect_),
    angle(0),
    line_thickness(line_thickness_),
    line_color(line_color_),
    fill_color(255, 255, 255),
    draw_line(true),
    draw_fill(false),
    draw_handles(false),
    handles_size(4.0f),
    handles_color(line_color)
{
}

IfOverlayRectangle::IfOverlayRectangle(const QRectF& rect_, const QColor& fill_color_) :
    ImageFrameOverlayBase(true),
    rect(rect_),
    angle(0),
    line_thickness(1),
    line_color(0, 0, 0),
    fill_color(fill_color_),
    draw_line(false),
    draw_fill(true),
    draw_handles(false),
    handles_size(4.0f),
    handles_color(line_color)
{
}

IfOverlayRectangle::IfOverlayRectangle(const QRectF& rect_, const QColor& line_color_, float line_thickness_, const QColor& fill_color_) :
    ImageFrameOverlayBase(true),
    rect(rect_),
    angle(0),
    line_thickness(line_thickness_),
    line_color(line_color_),
    fill_color(fill_color_),
    draw_line(true),
    draw_fill(true),
    draw_handles(false),
    handles_size(4.0f),
    handles_color(line_color)
{
}

IfOverlayRectangle::IfOverlayRectangle(const QRectF& rect_, float angle_, const QColor& line_color_, float line_thickness_) :
    ImageFrameOverlayBase(true),
    rect(rect_),
    angle(angle_),
    line_thickness(line_thickness_),
    line_color(line_color_),
    fill_color(255, 255, 255),
    draw_line(true),
    draw_fill(false),
    draw_handles(false),
    handles_size(4.0f),
    handles_color(line_color)
{
}

IfOverlayRectangle::IfOverlayRectangle(const QRectF& rect_, float angle_, const QColor& fill_color_) :
    ImageFrameOverlayBase(true),
    rect(rect_),
    angle(angle_),
    line_thickness(1),
    line_color(0, 0, 0),
    fill_color(fill_color_),
    draw_line(false),
    draw_fill(true),
    draw_handles(false),
    handles_size(4.0f),
    handles_color(line_color)
{
}

IfOverlayRectangle::IfOverlayRectangle(const QRectF& rect_, float angle_, const QColor& line_color_, float line_thickness_, const QColor& fill_color_) :
    ImageFrameOverlayBase(true),
    rect(rect_),
    angle(angle_),
    line_thickness(line_thickness_),
    line_color(line_color_),
    fill_color(fill_color_),
    draw_line(true),
    draw_fill(true),
    draw_handles(false),
    handles_size(4.0f),
    handles_color(line_color)
{
}

void IfOverlayRectangle::Draw(QPainter* painter) const
{
    if (painter) {
        painter->save();

        painter->setPen(draw_line ? QPen(line_color, line_thickness) : Qt::NoPen);
        painter->setBrush(draw_fill ? QBrush(fill_color) : Qt::NoBrush);

        QRectF frm_rect = ToFrameCoord(rect);
        QPointF frm_center = frm_rect.center();

        if (std::abs(angle) >= 0.001f) {
            painter->translate(frm_center.x(), frm_center.y());
            painter->rotate(angle);
            painter->translate(-frm_center.x(), -frm_center.y());
        }

        painter->drawRect(frm_rect);

        if (draw_handles) {
            painter->setPen(Qt::NoPen);
            painter->setBrush(handles_color);

            float radius = (handles_size > 0.0f ? handles_size * 0.5f : 2.0f);

            painter->drawEllipse(frm_rect.topLeft(), radius, radius);
            painter->drawEllipse(frm_rect.topRight(), radius, radius);
            painter->drawEllipse(frm_rect.bottomLeft(), radius, radius);
            painter->drawEllipse(frm_rect.bottomRight(), radius, radius);
            painter->drawEllipse(frm_rect.center(), radius, radius);
        }

        painter->restore();
    }
}

QVector<QPointF> IfOverlayRectangle::HandlePositions() const
{
    QVector<QPointF> positions(5);
    positions[0] = rect.topLeft();
    positions[1] = rect.topRight();
    positions[2] = rect.bottomLeft();
    positions[3] = rect.bottomRight();
    positions[4] = rect.center();
    return positions;
}

QPointF IfOverlayRectangle::HandlePosition(int handle_id) const
{
    switch (handle_id) {
        case 0: return rect.topLeft();
        case 1: return rect.topRight();
        case 2: return rect.bottomLeft();
        case 3: return rect.bottomRight();
        case 4: return rect.center();
        default: break;
    }
    return QPointF(-1, -1);
}

bool IfOverlayRectangle::onHandleMove(int handle_id, const QPointF& frm_pos)
{
    QPointF pos;
    ClipToImage(ToImageCoord(frm_pos), pos);
    bool success = true;
    switch (handle_id) {
        case 0: rect.setTopLeft(pos); break;
        case 1: rect.setTopRight(pos); break;
        case 2: rect.setBottomLeft(pos); break;
        case 3: rect.setBottomRight(pos); break;
        case 4: rect.moveCenter(pos); ClipToImage(rect, rect, false); break;
        default: success = false; break;
    }
    return success;
}

bool IfOverlayRectangle::onHandleRelease(int handle_id)
{
    rect = rect.normalized();
    return false;
}

///////////////////
// IfOverlayLine //
///////////////////

IfOverlayLine::IfOverlayLine() :
    ImageFrameOverlayBase(false),
    p1(0, 0),
    p2(0, 0),
    line_thickness(0),
    line_color(0, 0, 0)
{
}

IfOverlayLine::IfOverlayLine(const QPointF& p1_, const QPointF& p2_, const QColor& line_color_, float line_thickness_) :
    ImageFrameOverlayBase(true),
    p1(p1_),
    p2(p2_),
    line_thickness(line_thickness_),
    line_color(line_color_)
{
}

void IfOverlayLine::Draw(QPainter* painter) const
{
    if (painter) {
        painter->save();

        painter->setPen(QPen(line_color, line_thickness));
        painter->drawLine(ToFrameCoord(p1), ToFrameCoord(p2));

        painter->restore();
    }
}

///////////////////
// IfOverlayText //
///////////////////

IfOverlayText::IfOverlayText() :
    ImageFrameOverlayBase(false),
    pos(0, 0),
    frm_offset(0, 0),
    size(0),
    color(0, 0, 0),
    shadow(false),
    shadow_color(0, 0, 0),
    shadow_offset(1.0f, 1.0f)
{
}

IfOverlayText::IfOverlayText(const QString& text_, const QPointF& pos_, const QPointF& frm_offset_, const QColor& color_, float size_) :
    ImageFrameOverlayBase(true),
    text(text_),
    pos(pos_),
    frm_offset(frm_offset_),
    size(size_),
    color(color_),
    shadow(false),
    shadow_color(0, 0, 0),
    shadow_offset(1.0f, 1.0f)
{
}

IfOverlayText::IfOverlayText(const QString& text_, const QPointF& pos_, const QPointF& frm_offset_, const QColor& color_, float size_, const QColor& shadow_color_) :
    ImageFrameOverlayBase(true),
    text(text_),
    pos(pos_),
    frm_offset(frm_offset_),
    size(size_),
    color(color_),
    shadow(true),
    shadow_color(shadow_color_),
    shadow_offset(1.0f, 1.0f)
{
}

void IfOverlayText::Draw(QPainter* painter) const
{
    if (painter) {
        if (text.isEmpty() == false) {
            painter->save();

            painter->setFont(QFont("Helvetica", static_cast<int>(size)));

            if (shadow) {
                painter->setPen(shadow_color);
                painter->drawText(ToFrameCoord(pos) + frm_offset + QPointF(0, size) + shadow_offset, text);
            }

            painter->setPen(color);
            painter->drawText(ToFrameCoord(pos) + frm_offset + QPointF(0, size), text);

            painter->restore();
        }
    }
}

/////////////////////
// IfOverlayMarker //
/////////////////////

IfOverlayMarker::IfOverlayMarker() :
    ImageFrameOverlayBase(false),
    pos(0, 0),
    shape(DEFAULT),
    size(0),
    line_thickness(-1),
    angle(0),
    color(0, 0, 0),
    selected_color(0, 0, 0),
    dot(false),
    selected(false)
{
}

IfOverlayMarker::IfOverlayMarker(const QPointF& pos_, const QColor& color_, Shape shape_, float size_, float line_thickness_, bool dot_, float angle_) :
    ImageFrameOverlayBase(true),
    pos(pos_),
    shape(shape_),
    size(size_),
    line_thickness(line_thickness_),
    angle(angle_),
    color(color_),
    selected_color(0, 0, 0),
    dot(dot_),
    selected(false)
{
}

IfOverlayMarker::IfOverlayMarker(const QString& name_, const QPointF& pos_, const QColor& color_, Shape shape_, float size_, float line_thickness_, bool dot_, float angle_) :
    ImageFrameOverlayBase(true),
    pos(pos_),
    shape(shape_),
    size(size_),
    line_thickness(line_thickness_),
    angle(angle_),
    color(color_),
    selected_color(0, 0, 0),
    dot(dot_),
    name(name_),
    selected(false)
{
}

void IfOverlayMarker::Draw(QPainter* painter, QPointF frm_pos_, Shape shape_, float size_, float line_thickness_, float angle_, QColor color_, bool dot_, QString name_)
{
    Shape shape__ = (shape_ > DEFAULT ? DEFAULT : shape_);

    float line_thickness__ = (line_thickness_ > 0.0f ? line_thickness_ : 2.0f);
    // Antialising seems to be disabled when line thickness is exactly 1; this might be a Qt bug
    if (line_thickness__ == 1.0f) line_thickness__ = 1.01f;

    QPointF name_offset;

    switch (shape__) {
        case TRIANGLE:
        {
            painter->setPen(QPen(color_, line_thickness__));
            painter->setBrush(Qt::NoBrush);

            float size__ = (size_ > 0.0f ? size_ : 4.0f);

            float height = ::sqrt(size__ * size__ * 3.0f);
            float vert_offset = height / -6.0f;
            QPointF pt1 = QPointF(-size__, height * 0.5f + vert_offset);
            QPointF pt2 = QPointF(size__, height * 0.5f + vert_offset);
            QPointF pt3 = QPointF(0, height * -0.5f + vert_offset);
            QLineF frm_lines[3] = { QLineF(frm_pos_ + pt1, frm_pos_ + pt2),
                                    QLineF(frm_pos_ + pt2, frm_pos_ + pt3),
                                    QLineF(frm_pos_ + pt3, frm_pos_ + pt1) };

            if (std::abs(angle_) >= 0.001f) {
                painter->translate(frm_pos_.x(), frm_pos_.y());
                painter->rotate(angle_);
                painter->translate(-frm_pos_.x(), -frm_pos_.y());
            }

            painter->drawLines(frm_lines, 3);

            if (dot_) {
                painter->setPen(Qt::white);
                painter->drawEllipse(frm_pos_, 0.4f, 0.4f);
            }

            name_offset = QPointF(size__, size__);
        }
        break;

        case SQUARE:
        {
            painter->setPen(QPen(color_, line_thickness__));
            painter->setBrush(Qt::NoBrush);

            float size__ = (size_ > 0.0f ? size_ : 4.0f);
            float side_len = size__ * 2.0f;

            QRectF frm_rect = QRectF(frm_pos_ - QPointF(size__, size__), QSizeF(side_len, side_len));
            QPointF frm_center = frm_rect.center();

            if (std::abs(angle_) >= 0.001f) {
                painter->translate(frm_center.x(), frm_center.y());
                painter->rotate(angle_);
                painter->translate(-frm_center.x(), -frm_center.y());
            }

            painter->drawRect(frm_rect);

            if (dot_) {
                painter->setPen(Qt::white);
                painter->drawEllipse(frm_pos_, 0.4f, 0.4f);
            }

            name_offset = QPointF(size__, size__);
        }
        break;

        case CIRCLE:
        {
            if (line_thickness_ > 0.0f) {
                painter->setPen(QPen(color_, line_thickness__));
                painter->setBrush(Qt::NoBrush);
            } else {
                painter->setPen(Qt::NoPen);
                painter->setBrush(color_);
            }

            float radius = (size_ > 0.0f ? size_ : 4.0f);

            painter->drawEllipse(frm_pos_, radius, radius);

            if (dot_) {
                painter->setPen(Qt::white);
                painter->drawEllipse(frm_pos_, 0.4f, 0.4f);
            }

            name_offset = QPointF(radius - 1.0f, radius - 1.0f);
        }
        break;

        case CROSS:
        {
            painter->setPen(QPen(color_, line_thickness__));
            painter->setBrush(Qt::NoBrush);

            float size__ = (size_ > 0.0f ? size_ : 4.0f);

            QLineF frm_lines[2] = { QLineF(frm_pos_ + QPointF(-size__, 0), frm_pos_ + QPointF(size__, 0)),
                                    QLineF(frm_pos_ + QPointF(0, -size__), frm_pos_ + QPointF(0, size__)) };

            if (std::abs(angle_) >= 0.001f) {
                painter->translate(frm_pos_.x(), frm_pos_.y());
                painter->rotate(angle_);
                painter->translate(-frm_pos_.x(), -frm_pos_.y());
            }

            painter->drawLines(frm_lines, 2);

            if (dot_) {
                painter->setPen(Qt::white);
                painter->drawEllipse(frm_pos_, 0.4f, 0.4f);
            }

            name_offset = QPointF(size__ - 1.0f, size__ - 1.0f);
        }
        break;

        case STAR:
        {
            painter->setPen(QPen(color_, line_thickness__));
            painter->setBrush(Qt::NoBrush);

            float size__ = (size_ > 0.0f ? size_ : 4.0f);
            float diag_size = size__ * static_cast<float>(M_SQRT1_2);

            QLineF frm_lines[4] = { QLineF(frm_pos_ + QPointF(-size__, 0), frm_pos_ + QPointF(size__, 0)),
                                    QLineF(frm_pos_ + QPointF(0, -size__), frm_pos_ + QPointF(0, size__)),
                                    QLineF(frm_pos_ + QPointF(-diag_size, -diag_size), frm_pos_ + QPointF(diag_size, diag_size)),
                                    QLineF(frm_pos_ + QPointF(-diag_size, diag_size), frm_pos_ + QPointF(diag_size, -diag_size)) };

            if (std::abs(angle_) >= 0.001f) {
                painter->translate(frm_pos_.x(), frm_pos_.y());
                painter->rotate(angle_);
                painter->translate(-frm_pos_.x(), -frm_pos_.y());
            }

            painter->drawLines(frm_lines, 4);

            if (dot_) {
                painter->setPen(Qt::white);
                painter->drawEllipse(frm_pos_, 0.4f, 0.4f);
            }

            name_offset = QPointF(size__ - 1.0f, size__ - 1.0f);
        }
        break;

        case CROSSHAIR:
        case DEFAULT:
        {
            painter->setBrush(Qt::NoBrush);

            float radius4 = (size_ > 0.0f ? size_ : 8.0f);
            float radius3 = radius4 * 0.85f;
            float radius2 = radius3 - line_thickness__ * 0.5f;
            float radius1 = radius4 * 0.35f;

            QLineF frm_inner_lines[4] = { QLineF(frm_pos_ + QPointF(-radius2, 0), frm_pos_ + QPointF(-radius1, 0)),
                                          QLineF(frm_pos_ + QPointF(radius1, 0), frm_pos_ + QPointF(radius2, 0)),
                                          QLineF(frm_pos_ + QPointF(0, -radius2), frm_pos_ + QPointF(0, -radius1)),
                                          QLineF(frm_pos_ + QPointF(0, radius1), frm_pos_ + QPointF(0, radius2)) };
            QLineF frm_outer_lines[4] = { QLineF(frm_pos_ + QPointF(-radius4, 0), frm_pos_ + QPointF(-radius3, 0)),
                                          QLineF(frm_pos_ + QPointF(radius3, 0), frm_pos_ + QPointF(radius4, 0)),
                                          QLineF(frm_pos_ + QPointF(0, -radius4), frm_pos_ + QPointF(0, -radius3)),
                                          QLineF(frm_pos_ + QPointF(0, radius3), frm_pos_ + QPointF(0, radius4)) };

            if (std::abs(angle_) >= 0.001f) {
                painter->translate(frm_pos_.x(), frm_pos_.y());
                painter->rotate(angle_);
                painter->translate(-frm_pos_.x(), -frm_pos_.y());
            }

            painter->setPen(QPen(color_, 1.01f));
            painter->drawLines(frm_inner_lines, 4);
            painter->setPen(QPen(color_, line_thickness__));
            painter->drawLines(frm_outer_lines, 4);

            if (dot_) {
                painter->setPen(Qt::white);
                painter->drawEllipse(frm_pos_, 0.4f, 0.4f);
            }

            name_offset = QPointF(radius4 - 1.0f, radius4 - 1.0f);
        }
        break;
    }

    if (name_.isEmpty() == false) {
        const float font_size = 10.0f;
        const float bg_margin = 2.0f;
        const float extra_bottom_margin = 2.0f; // Qt seems to position text with only integer precision

        QFont font("Helvetica", static_cast<int>(font_size));
        QFontMetricsF fm(font);
        QRectF bb = fm.tightBoundingRect(name_);

        QPointF text_pos = frm_pos_ + name_offset + QPointF(bg_margin, bg_margin);

        painter->setFont(font);
        painter->setPen(Qt::NoPen);
        float r, g, b, h, s, v;
        RGBtoHSV(color_.red() / 255.0f, color_.green() / 255.0f, color_.blue() / 255.0f, h, s, v);
        HSVtoRGB(h, s, (v < 0.7f ? 1.0f : 0.25f), r, g, b);
        painter->setBrush(QColor(static_cast<int>(r * 255.0f), static_cast<int>(g * 255.0f), static_cast<int>(b * 255.0f)));
        painter->drawRect(QRectF(text_pos - QPointF(bg_margin, bg_margin),
                                 bb.size() + QSizeF(bg_margin * 2.0f, bg_margin * 2.0f + extra_bottom_margin)));

        painter->setPen(color_);
        painter->setBrush(Qt::NoBrush);
        painter->drawText(text_pos + QPointF(0, font_size), name_);
    }
}

void IfOverlayMarker::Draw(QPainter* painter) const
{
    painter->save();
    Draw(painter, ToFrameCoord(pos), (shape > DEFAULT ? DEFAULT : shape), size, line_thickness, angle, (selected ? selected_color : color), dot, name);
    painter->restore();
}

QVector<QPointF> IfOverlayMarker::HandlePositions() const
{
    return QVector<QPointF>(1, pos);
}

QPointF IfOverlayMarker::HandlePosition(int handle_id) const
{
    if (handle_id == 0) return pos;
    return QPointF(-1, -1);
}

bool IfOverlayMarker::onSelect(int handle_id)
{
    selected = true;
    return true;
}

bool IfOverlayMarker::onDeselect()
{
    selected = false;
    return true;
}

bool IfOverlayMarker::onHandleGrab(int handle_id)
{
    return false;
}

bool IfOverlayMarker::onHandleMove(int handle_id, const QPointF& frm_pos)
{
    ClipToImage(ToImageCoord(frm_pos), pos);
    return true;
}

bool IfOverlayMarker::onHandleRelease(int handle_id)
{
    return false;
}

////////////////////////
// IfOverlayMarkerSet //
////////////////////////

IfOverlayMarkerSet::IfOverlayMarkerSet() :
    ImageFrameOverlayBase(false),
    selected_marker(-1),
    shape(IfOverlayMarker::DEFAULT),
    size(0),
    line_thickness(-1),
    angle(0),
    color(0, 0, 0),
    selected_color(0, 0, 0),
    dot(false)
{
}

IfOverlayMarkerSet::IfOverlayMarkerSet(const QColor& color_, IfOverlayMarker::Shape shape_, float size_, float line_thickness_, bool dot_, float angle_) :
    ImageFrameOverlayBase(true),
    selected_marker(-1),
    shape(shape_),
    size(size_),
    line_thickness(line_thickness_),
    angle(angle_),
    color(color_),
    selected_color(0, 0, 0),
    dot(dot_)
{
}

void IfOverlayMarkerSet::Draw(QPainter* painter) const
{
    IfOverlayMarker::Shape shape_ = (shape > IfOverlayMarker::DEFAULT ? IfOverlayMarker::DEFAULT : shape);
    painter->save();
    for (int i = 0; i < markers.size(); i ++) {
        IfOverlayMarker::Draw(painter, ToFrameCoord(markers[i].pos), shape_, size, line_thickness, angle, (selected_marker == i ? selected_color : color), dot, markers[i].name);
    }
    painter->restore();
}

void IfOverlayMarkerSet::push_back(const QString& name, const QPointF& pos)
{
    markers.push_back(Marker(name, pos));
}

void IfOverlayMarkerSet::erase(int index)
{
    markers.erase(markers.begin() + index);
}

void IfOverlayMarkerSet::clear()
{
    markers.clear();
}

QVector<QPointF> IfOverlayMarkerSet::HandlePositions() const
{
    const int size = markers.size();
    QVector<QPointF> positions(size);
    for (int i = 0; i < size; i ++) positions[i] = markers[i].pos;
    return positions;
}

QPointF IfOverlayMarkerSet::HandlePosition(int handle_id) const
{
    if (handle_id >= 0 && handle_id < markers.size()) return markers[handle_id].pos;
    return QPointF(-1, -1);
}

bool IfOverlayMarkerSet::onSelect(int handle_id)
{
    selected_marker = handle_id;
    return true;
}

bool IfOverlayMarkerSet::onDeselect()
{
    selected_marker = -1;
    return true;
}

bool IfOverlayMarkerSet::onHandleGrab(int handle_id)
{
    return false;
}

bool IfOverlayMarkerSet::onHandleMove(int handle_id, const QPointF& frm_pos)
{
    if (handle_id >= 0 && handle_id < markers.size()) {
        ClipToImage(ToImageCoord(frm_pos), markers[handle_id].pos);
        return true;
    }
    return false;
}

bool IfOverlayMarkerSet::onHandleRelease(int handle_id)
{
    return false;
}
