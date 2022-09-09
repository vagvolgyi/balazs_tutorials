#ifndef _image_frame_h_
#define _image_frame_h_

#include <QFrame>
#include <QPainter>
#include <QImage>
#include <QMutex>


class ImageFrameOverlayBase;

class ImageFrame : public QFrame
{
    Q_OBJECT

public:
    ImageFrame(QWidget* parent, Qt::WindowFlags flags = nullptr);
    virtual ~ImageFrame();

    void SetImage(const QImage& image);
    QRect GetCenteredRect(double aspect_ratio, int margin) const;

    // AddOverlay() takes ownership of the object
    void AddOverlay(ImageFrameOverlayBase* overlay);

    // DeleteOverlay() and DeleteAllOverlays() deallocate objects from the heap
    void DeleteOverlay(ImageFrameOverlayBase* overlay);
    void DeleteAllOverlays();

    void SelectOverlay(int overlay_id, int handle_id, bool emit_events);

    bool ToImageCoord(const QPointF& frm_pos, QPointF& image_pos) const;
    bool ToImageScale(float frm_length, float& image_length) const;

signals:
    void delayedUpdate();
    void mouseLeft(int x, int y);
    void mouseLeftReleased(int x, int y);
    void mouseMove(int x, int y);
    void overlaySelected(int overlay_id, int handle_id);
    void overlayChanged(int overlay_id, int handle_id);

protected:
    virtual void paintEvent(QPaintEvent*);
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);

    virtual bool FindNearestOverlayHandle(const QPointF& frm_pos, int& overlay_id, int& handle_id);

protected:
    bool ImageUpdated;
    QImage Image;
    QImage ScaledImage;
    mutable QMutex ImageMutex;

    std::vector<ImageFrameOverlayBase*> Overlays;

    bool HandleIsMoved;
    float HandleGrabMaxDistance;
    QPointF HandleGrabOffset;
    int SelectedOverlay;
    int SelectedHandle;
};

#endif // _image_frame_h_
