#include "common/qt/image_frame/image_frame.h"
#include "common/qt/image_frame/if_overlay.h"

#include <QMouseEvent>

#include <QtDebug>

ImageFrame::ImageFrame(QWidget* parent, Qt::WindowFlags flags) :
    QFrame(parent, flags),
    ImageUpdated(false),
    HandleGrabMaxDistance(10.0f),
    SelectedOverlay(-1),
    SelectedHandle(-1)
{
    connect(this, SIGNAL(delayedUpdate()), this, SLOT(update()), Qt::QueuedConnection);
}

ImageFrame::~ImageFrame()
{
    DeleteAllOverlays();
}

void ImageFrame::SetImage(const QImage& image)
{
    ImageMutex.lock();
    Image = image.copy();
    ImageUpdated = true;
    ImageMutex.unlock();
    emit delayedUpdate();
}

QRect ImageFrame::GetCenteredRect(double aspect_ratio, int margin) const
{
    QRect frame_rect = frameRect();
    frame_rect = QRect(frame_rect.x() + margin, frame_rect.y() + margin, frame_rect.width() - 2 * margin, frame_rect.height() - 2 * margin);
    const double frame_ar = static_cast<double>(frame_rect.width()) / static_cast<double>(frame_rect.height());
    QRect centered_rect;
    if (frame_ar > aspect_ratio) {
        centered_rect = QRect(margin + static_cast<int>((static_cast<double>(frame_rect.width()) - static_cast<double>(frame_rect.height()) * aspect_ratio) * 0.5),
                              margin,
                              static_cast<int>(static_cast<double>(frame_rect.height()) * aspect_ratio),
                              frame_rect.height());
    } else {
        centered_rect = QRect(margin,
                              margin + static_cast<int>((static_cast<double>(frame_rect.height()) - static_cast<double>(frame_rect.width()) / aspect_ratio) * 0.5),
                              frame_rect.width(),
                              static_cast<int>(static_cast<double>(frame_rect.width()) / aspect_ratio));
    }
    return centered_rect;
}

void ImageFrame::AddOverlay(ImageFrameOverlayBase* overlay)
{
    if (!overlay) return;
    Overlays.push_back(overlay);
    emit delayedUpdate();
}

void ImageFrame::DeleteOverlay(ImageFrameOverlayBase* overlay)
{
    if (!overlay) return;
    for (size_t i = 0; i < Overlays.size(); i ++) {
        if (Overlays[i] == overlay) {
            if (SelectedOverlay == static_cast<int>(i)) SelectOverlay(-1, -1, false);
            delete overlay;
            Overlays.erase(Overlays.begin() + static_cast<int>(i));
            return;
        }
    }
    emit delayedUpdate();
}

void ImageFrame::DeleteAllOverlays()
{
    SelectOverlay(-1, -1, false);
    for (size_t i = 0; i < Overlays.size(); i ++) {
        delete Overlays[i];
    }
    Overlays.clear();
    emit delayedUpdate();
}

void ImageFrame::SelectOverlay(int overlay_id, int handle_id, bool emit_events)
{
    bool redraw = false;
    if (overlay_id >= 0 && overlay_id < static_cast<int>(Overlays.size())) {
        if (SelectedOverlay < 0) {
            SelectedOverlay = overlay_id;
            if (Overlays[static_cast<size_t>(overlay_id)]->onSelect(handle_id)) redraw = true;
            if (emit_events) {
                emit overlaySelected(overlay_id, handle_id);
            }
        } else if (SelectedOverlay != overlay_id || SelectedHandle != handle_id) {
            if (Overlays[static_cast<size_t>(SelectedOverlay)]->onDeselect()) redraw = true;
            SelectedOverlay = overlay_id;
            if (Overlays[static_cast<size_t>(overlay_id)]->onSelect(handle_id)) redraw = true;
            if (emit_events) {
                emit overlaySelected(overlay_id, handle_id);
            }
        }
        SelectedHandle = -1;
    } else {
        if (SelectedOverlay >= 0) {
            if (Overlays[static_cast<size_t>(SelectedOverlay)]->onDeselect()) redraw = true;
            SelectedOverlay = -1;
            if (emit_events) {
                emit overlaySelected(overlay_id, handle_id);
            }
        }
        SelectedHandle = -1;
    }
    if (redraw) {
        emit delayedUpdate();
    }
}

bool ImageFrame::ToImageCoord(const QPointF& frm_pos, QPointF& image_pos) const
{
    if (Image.isNull()) return false;

    QRect image_rect = GetCenteredRect(static_cast<double>(Image.width()) / static_cast<double>(Image.height()), 1);
    QSize image_size = Image.size();

    QPointF draw_offset = image_rect.topLeft();
    float draw_scale = static_cast<float>(image_rect.width()) / static_cast<float>(image_size.width());

    image_pos = (frm_pos - draw_offset) / draw_scale;

    if (image_pos.x() < 0 || image_pos.x() > Image.width() - 1 ||
        image_pos.y() < 0 || image_pos.y() > Image.height() - 1) return false;

    return true;
}

bool ImageFrame::ToImageScale(float frm_length, float& image_length) const
{
    if (Image.isNull()) return false;

    QRect image_rect = GetCenteredRect(static_cast<double>(Image.width()) / static_cast<double>(Image.height()), 1);
    QSize image_size = Image.size();

    float draw_scale = static_cast<float>(image_rect.width()) / static_cast<float>(image_size.width());

    image_length = frm_length / draw_scale;

    return true;
}

void ImageFrame::paintEvent(QPaintEvent*)
{
    QPainter painter(this);

    painter.save();

    painter.setPen(Qt::darkGray);
    painter.setBrush(QBrush(Qt::lightGray, Qt::DiagCrossPattern));
    painter.drawRect(0, 0, frameRect().width() - 1, frameRect().height() - 1);

    painter.restore();

    bool image_drawn = false;
    QRect image_rect;
    QSize image_size;

    ImageMutex.lock();
    if (!Image.isNull()) {
        image_rect = GetCenteredRect(static_cast<double>(Image.width()) / static_cast<double>(Image.height()), 1);
        image_size = Image.size();
        if (ImageUpdated || image_rect.size() != ScaledImage.size()) {
            ScaledImage = Image.scaled(image_rect.size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
            ImageUpdated = false;
        }
        ImageMutex.unlock();

        painter.setRenderHint(QPainter::Antialiasing);
        painter.drawImage(image_rect, ScaledImage);
        image_drawn = true;
    } else {
        ImageMutex.unlock();
    }

    if (image_drawn) {
        painter.setRenderHint(QPainter::Antialiasing);
        for (size_t i = 0; i < Overlays.size(); i ++) {
            ImageFrameOverlayBase* overlay = Overlays[i];
            if (overlay && overlay->visible) {
                // Setup overlay for correct scaling
                overlay->ImageSize = image_size;
                overlay->DrawOffset = image_rect.topLeft();
                overlay->DrawScale = static_cast<float>(image_rect.width()) / static_cast<float>(image_size.width());
                // Render overlay
                overlay->Draw(&painter);
            }
        }
    }
}

void ImageFrame::mousePressEvent(QMouseEvent* event)
{
    QPointF mouse_pos(event->x(), event->y());
    int overlay_id, handle_id;
    if (FindNearestOverlayHandle(mouse_pos, overlay_id, handle_id)) {
        SelectOverlay(overlay_id, handle_id, true);
        SelectedHandle = handle_id;

        ImageFrameOverlayBase* overlay = Overlays[static_cast<size_t>(overlay_id)];
        if (overlay->editable) {
            HandleIsMoved = true;
            HandleGrabOffset = mouse_pos - overlay->ToFrameCoord(overlay->HandlePosition(handle_id));
            if (overlay->onHandleGrab(handle_id)) {
                emit delayedUpdate();
            }
        }
    } else {
        SelectOverlay(-1, -1, true);
        SelectedHandle = -1;
    }

    if(event->button() == Qt::LeftButton) {
        emit mouseLeft(event->x(), event->y());
    }
    QFrame::mousePressEvent(event);
}

void ImageFrame::mouseReleaseEvent(QMouseEvent* event)
{
    bool redraw = false;
    if (HandleIsMoved && SelectedOverlay >= 0 && SelectedHandle >= 0) {
        QPointF mouse_pos(event->x(), event->y());
        ImageFrameOverlayBase* overlay = Overlays[static_cast<size_t>(SelectedOverlay)];
        if (overlay->onHandleMove(SelectedHandle, mouse_pos - HandleGrabOffset)) redraw = true;
        if (overlay->onHandleRelease(SelectedHandle)) redraw = true;
        emit overlayChanged(SelectedOverlay, SelectedHandle);
    }
    HandleIsMoved = false;

    if (event->button() == Qt::LeftButton) {
        emit mouseLeftReleased(event->x(), event->y());
    }
    QFrame::mouseReleaseEvent(event);

    if (redraw) {
        emit delayedUpdate();
    }
}

void ImageFrame::mouseMoveEvent(QMouseEvent* event)
{
    bool redraw = false;
    if (HandleIsMoved && SelectedOverlay >= 0 && SelectedHandle >= 0) {
        QPointF mouse_pos(event->x(), event->y());
        ImageFrameOverlayBase* overlay = Overlays[static_cast<size_t>(SelectedOverlay)];
        if (overlay->onHandleMove(SelectedHandle, mouse_pos - HandleGrabOffset)) redraw = true;
        emit overlayChanged(SelectedOverlay, SelectedHandle);
    }

    emit mouseMove(event->x(), event->y());
    QFrame::mouseMoveEvent(event);

    if (redraw) {
        emit delayedUpdate();
    }
}

bool ImageFrame::FindNearestOverlayHandle(const QPointF& frm_pos, int& overlay_id, int& handle_id)
{
    overlay_id = -1;
    handle_id = -1;
    if (Image.isNull()) return false;

    const double max_dist_sq = HandleGrabMaxDistance * HandleGrabMaxDistance;
    double dist_sq, min_dist_sq = 10000000.0;

    for (size_t i = 0; i < Overlays.size(); i ++) {
        ImageFrameOverlayBase* overlay = Overlays[i];
        if (overlay->visible == false || overlay->pickable == false) continue;

        QVector<QPointF> handles = overlay->HandlePositions();
        for (int j = 0; j < handles.size(); j ++) {
            QPointF d = frm_pos - overlay->ToFrameCoord(handles[j]);
            dist_sq = d.x() * d.x() + d.y() * d.y();
            if (dist_sq <= max_dist_sq && dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                overlay_id = static_cast<int>(i);
                handle_id = j;
            }
        }
    }

    return (overlay_id >= 0);
}
