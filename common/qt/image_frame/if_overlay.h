#ifndef _if_overlay_h_
#define _if_overlay_h_

#include "image_frame.h"


class ImageFrameOverlayBase
{
    friend class ImageFrame;

public:
    ImageFrameOverlayBase();
    ImageFrameOverlayBase(bool visible, bool pickable = false, bool editable = false);
    virtual ~ImageFrameOverlayBase();

    static void RGBtoHSV(float r, float g, float b, float& h, float& s, float& v);
    static void HSVtoRGB(float h, float s, float v, float& r, float& g, float& b);

    QPointF ToImageCoord(const QPointF& frm_pos) const;
    QRectF ToImageCoord(const QRectF& frm_rect) const;
    float ToImageScale(float frm_length) const;
    QSizeF ToImageScale(const QSizeF& frm_size) const;

    QPointF ToFrameCoord(const QPointF& image_pos) const;
    QRectF ToFrameCoord(const QRectF& image_rect) const;
    float ToFrameScale(float image_length) const;
    QSizeF ToFrameScale(const QSizeF& image_size) const;

    bool ClipToImage(const QPointF& image_pos, QPointF& clipped_pos) const;
    bool ClipToImage(const QRectF& image_rect, QRectF& clipped_rect, bool allow_resize) const;

protected:
    virtual void Draw(QPainter* painter) const = 0;
    virtual QVector<QPointF> HandlePositions() const;
    virtual QPointF HandlePosition(int handle_id) const;

protected:
    virtual bool onSelect(int handle_id);
    virtual bool onDeselect();
    virtual bool onHandleGrab(int handle_id);
    virtual bool onHandleMove(int handle_id, const QPointF& frm_pos);
    virtual bool onHandleRelease(int handle_id);

private:
    QSizeF ImageSize;
    QPointF DrawOffset;
    float DrawScale;

public:
    bool visible;
    bool pickable;
    bool editable;
};

class IfOverlayEllipse : public ImageFrameOverlayBase
{
public:
    IfOverlayEllipse();
    IfOverlayEllipse(const QPointF& center, float radius, const QColor& line_color, float line_thickness);
    IfOverlayEllipse(const QPointF& center, float radius, const QColor& fill_color);
    IfOverlayEllipse(const QPointF& center, float radius, const QColor& line_color, float line_thickness, const QColor& fill_color);
    IfOverlayEllipse(const QPointF& center, float radius_x, float radius_y, float angle, const QColor& line_color, float line_thickness);
    IfOverlayEllipse(const QPointF& center, float radius_x, float radius_y, float angle, const QColor& fill_color);
    IfOverlayEllipse(const QPointF& center, float radius_x, float radius_y, float angle, const QColor& line_color, float line_thickness, const QColor& fill_color);

protected:
    virtual void Draw(QPainter* painter) const;

public:
    QPointF center;
    float radius_x;
    float radius_y;
    float angle;
    float line_thickness;
    QColor line_color;
    QColor fill_color;
    bool draw_line;
    bool draw_fill;
};

class IfOverlayRectangle : public ImageFrameOverlayBase
{
public:
    IfOverlayRectangle();
    IfOverlayRectangle(const QRectF& rect, const QColor& line_color, float line_thickness);
    IfOverlayRectangle(const QRectF& rect, const QColor& fill_color);
    IfOverlayRectangle(const QRectF& rect, const QColor& line_color, float line_thickness, const QColor& fill_color);
    IfOverlayRectangle(const QRectF& rect, float angle, const QColor& line_color, float line_thickness);
    IfOverlayRectangle(const QRectF& rect, float angle, const QColor& fill_color);
    IfOverlayRectangle(const QRectF& rect, float angle, const QColor& line_color, float line_thickness, const QColor& fill_color);

protected:
    virtual void Draw(QPainter* painter) const;
    virtual QVector<QPointF> HandlePositions() const;
    virtual QPointF HandlePosition(int handle_id) const;

protected:
    virtual bool onHandleMove(int handle_id, const QPointF& frm_pos);
    virtual bool onHandleRelease(int handle_id);

public:
    QRectF rect;
    float angle;
    float line_thickness;
    QColor line_color;
    QColor fill_color;
    bool draw_line;
    bool draw_fill;
    bool draw_handles;
    float handles_size;
    QColor handles_color;
};

class IfOverlayLine : public ImageFrameOverlayBase
{
public:
    IfOverlayLine();
    IfOverlayLine(const QPointF& p1, const QPointF& p2, const QColor& line_color, float line_thickness);

protected:
    virtual void Draw(QPainter* painter) const;

public:
    QPointF p1;
    QPointF p2;
    float line_thickness;
    QColor line_color;
};

class IfOverlayText : public ImageFrameOverlayBase
{
public:
    IfOverlayText();
    IfOverlayText(const QString& text, const QPointF& pos, const QPointF& frm_offset, const QColor& color, float size);
    IfOverlayText(const QString& text, const QPointF& pos, const QPointF& frm_offset, const QColor& color, float size, const QColor& shadow_color);

protected:
    virtual void Draw(QPainter* painter) const;

public:
    QString text;
    QPointF pos;
    QPointF frm_offset;
    float size;
    QColor color;
    bool shadow;
    QColor shadow_color;
    QPointF shadow_offset;
};

class IfOverlayMarker : public ImageFrameOverlayBase
{
public:
    enum Shape {
        TRIANGLE,
        SQUARE,
        CIRCLE,
        CROSS,
        STAR,
        CROSSHAIR,
        DEFAULT
    };

    IfOverlayMarker();
    IfOverlayMarker(const QPointF& pos, const QColor& color, Shape shape = DEFAULT, float size = -1, float line_thickness = -1, bool dot = false, float angle = 0.0f);
    IfOverlayMarker(const QString& name, const QPointF& pos, const QColor& color, Shape shape = DEFAULT, float size = -1, float line_thickness = -1, bool dot = false, float angle = 0.0f);

    static void Draw(QPainter* painter, QPointF frm_pos, Shape shape, float size, float line_thickness, float angle, QColor color, bool dot, QString name);

protected:
    virtual void Draw(QPainter* painter) const;
    virtual QVector<QPointF> HandlePositions() const;
    virtual QPointF HandlePosition(int handle_id) const;

protected:
    virtual bool onSelect(int handle_id);
    virtual bool onDeselect();
    virtual bool onHandleGrab(int handle_id);
    virtual bool onHandleMove(int handle_id, const QPointF& frm_pos);
    virtual bool onHandleRelease(int handle_id);

public:
    QPointF pos;
    Shape shape;
    float size;
    float line_thickness;
    float angle;
    QColor color;
    QColor selected_color;
    bool dot;
    QString name;
    bool selected;
};

class IfOverlayMarkerSet : public ImageFrameOverlayBase
{
public:
    struct Marker
    {
        Marker() : pos(-1, -1) {}
        Marker(const QString& name_, const QPointF& pos_) : name(name_), pos(pos_) {}
        QString name;
        QPointF pos;
    };

    IfOverlayMarkerSet();
    IfOverlayMarkerSet(const QColor& color, IfOverlayMarker::Shape shape = IfOverlayMarker::DEFAULT, float size = -1, float line_thickness = -1, bool dot = false, float angle = 0.0f);

    void push_back(const QString& name, const QPointF& pos);
    void erase(int index);
    void clear();

protected:
    virtual void Draw(QPainter* painter) const;
    virtual QVector<QPointF> HandlePositions() const;
    virtual QPointF HandlePosition(int handle_id) const;

protected:
    virtual bool onSelect(int handle_id);
    virtual bool onDeselect();
    virtual bool onHandleGrab(int handle_id);
    virtual bool onHandleMove(int handle_id, const QPointF& frm_pos);
    virtual bool onHandleRelease(int handle_id);

public:
    QVector<Marker> markers;
    int selected_marker;

    IfOverlayMarker::Shape shape;
    float size;
    float line_thickness;
    float angle;
    QColor color;
    QColor selected_color;
    bool dot;
};

#endif // _if_overlay_h_
