#ifndef OVERLAYS_DIALOG_H
#define OVERLAYS_DIALOG_H

#include <ui_overlays_dialog.h>

#include <QDialog>
#include <QTimer>

#include <opencv2/core.hpp>

class IfOverlayMarker;


class OverlaysDialog : public QDialog
{
    Q_OBJECT

public:
    OverlaysDialog(QWidget* parent, const QString& file_path);
    ~OverlaysDialog();

protected slots:
    void onAnimate();
    void onOverlayChanged(int overlay_id, int handle_id);

protected:
    Ui::OverlaysDialog* Ui;

    IfOverlayMarker* Point;
    QTimer Timer;
};

#endif // OVERLAYS_DIALOG_H
