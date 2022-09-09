#include "my_window.h"
#include "ui_my_window.h"

#include "overlays_dialog.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QStandardPaths>


MyWindow::MyWindow(QWidget *parent) :
    QMainWindow(parent),
    Ui(new Ui::MyWindow)
{
    Ui->setupUi(this);

    connect(Ui->open_push_button, &QPushButton::clicked, this, &MyWindow::onOpenClicked);
    connect(Ui->overlays_push_button, &QPushButton::clicked, this, &MyWindow::onOverlaysClicked);
    connect(Ui->test_double_spin_box, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &MyWindow::onSpinBoxValueChanged);
    connect(this, &MyWindow::mySignal, this, &MyWindow::onMySignal);
    connect(Ui->buttonBox->button(QDialogButtonBox::Close), &QAbstractButton::clicked, this, &MyWindow::close);
    connect(Ui->buttonBox->button(QDialogButtonBox::Apply), &QAbstractButton::clicked, this, &MyWindow::onApply);
    connect(Ui->options_button_group, static_cast<void (QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked), this, &MyWindow::onOptionsChanged);

    Ui->options_button_group->setId(Ui->option1_radio_button, 1);
    Ui->options_button_group->setId(Ui->option2_radio_button, 2);
    Ui->options_button_group->setId(Ui->option3_radio_button, 3);
    Ui->option_line_edit->setText(QString::number(Ui->options_button_group->checkedId()));

    ProgramDir = QDir::toNativeSeparators(QStandardPaths::displayName(QStandardPaths::HomeLocation) + "/");
}

MyWindow::~MyWindow()
{
    delete Ui;
}

void MyWindow::onOpenClicked()
{
    QString qstr = QFileDialog::getOpenFileName(this, tr("Open Image File"), ProgramDir, tr("Image files (*.png *.jpg *.bmp);;Movie files (*.mp3 *.mpg *.avi)"));
    Ui->text_label->setText(qstr);
}

void MyWindow::onOverlaysClicked()
{
    OverlaysDialog dialog(this, Ui->text_label->text());
    int ret = dialog.exec();
    if (ret == QDialog::Accepted) {
        // ...
    }
}

void MyWindow::onOptionsChanged(int option)
{
    Ui->option_line_edit->setText(QString::number(option));
}

void MyWindow::onSpinBoxValueChanged(double value)
{
    Ui->test_line_edit->setText(QString::number(value));
    emit mySignal(10.0);
}

void MyWindow::onApply()
{
    QMessageBox msg_box;
    msg_box.setWindowTitle("Qt Tutorial");
    msg_box.setText("\nApply button pressed.\n");
    msg_box.setStandardButtons(QMessageBox::No|QMessageBox::Yes);
    msg_box.setDefaultButton(QMessageBox::No);
    msg_box.exec();
    if (msg_box.result() != QMessageBox::Yes) {
        // ...
    }
}

void MyWindow::onMySignal(double value)
{
    // ...
}

void MyWindow::closeEvent(QCloseEvent* event)
{
    double value = Ui->test_double_spin_box->value();

    QMainWindow::closeEvent(event);
}
