#include "overlays_dialog.h"
#include "ui_overlays_dialog.h"

#include "QAbstractButton"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "qt/image_frame/if_overlay.h"


OverlaysDialog::OverlaysDialog(QWidget* parent, const QString& file_path) :
    QDialog(parent),
    Ui(new Ui::OverlaysDialog)
{
    Ui->setupUi(this);

    connect(Ui->buttonBox, &QDialogButtonBox::accepted, this, &OverlaysDialog::accept);
    connect(Ui->buttonBox, &QDialogButtonBox::rejected, this, &OverlaysDialog::reject);
    connect(Ui->image_frame, &ImageFrame::overlayChanged, this, &OverlaysDialog::onOverlayChanged);

    cv::Mat image = cv::imread(file_path.toStdString());
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    QImage q_image(image.data, image.cols, image.rows, static_cast<int>(image.step[0]), QImage::Format_RGB888);
    Ui->image_frame->SetImage(q_image);

    Point = new IfOverlayMarker(QPointF(image.cols / 2, image.rows / 3), QColor(Qt::blue));
    Point->shape = IfOverlayMarker::TRIANGLE;
    Point->selected_color = Point->color;
    Point->size = 10;
    Point->pickable = true;
    Point->editable = true;
    Ui->image_frame->AddOverlay(Point);

    IfOverlayMarkerSet* Points = new IfOverlayMarkerSet(Qt::green);
    Points->selected_color = Qt::darkGreen;
    Points->markers.resize(3);
    Points->markers[0].pos = QPointF(200, 200);
    Points->markers[1].pos = QPointF(300, 300);
    Points->markers[2].pos = QPointF(400, 400);
    Points->shape = IfOverlayMarker::STAR;
    Points->size = 10;
    Points->pickable = true;
    Points->editable = true;
    Ui->image_frame->AddOverlay(Points);

    IfOverlayRectangle* rect = new IfOverlayRectangle(QRectF(100, 100, image.cols - 200, image.rows - 200), QColor(Qt::red), 3);
    rect->draw_handles = true;
    rect->handles_size = 10;
    rect->handles_color = Qt::yellow;
    rect->pickable = true;
    rect->editable = true;
    Ui->image_frame->AddOverlay(rect);

    Timer.start(25);

    connect(&Timer, &QTimer::timeout, this, &OverlaysDialog::onAnimate);
}

OverlaysDialog::~OverlaysDialog()
{
    delete Ui;
}

void OverlaysDialog::onAnimate()
{
    Point->angle += 4.0f;
    Ui->image_frame->delayedUpdate();
}

void OverlaysDialog::onOverlayChanged(int overlay_id, int handle_id)
{
    if (overlay_id == 0) {
        Ui->x_line_edit->setText(QString::number(Point->pos.x()));
        Ui->y_line_edit->setText(QString::number(Point->pos.y()));
    }
}
