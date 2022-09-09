#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>


namespace Ui {
    class MyWindow;
}

class MyWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MyWindow(QWidget *parent = nullptr);
    ~MyWindow();

protected slots:
    void onOpenClicked();
    void onOverlaysClicked();
    void onOptionsChanged(int option);
    void onSpinBoxValueChanged(double value);
    void onApply();
    void onMySignal(double value);

signals:
    void mySignal(double value);

protected:
    virtual void closeEvent(QCloseEvent* event);

private:
    Ui::MyWindow* Ui;

    QString ProgramDir;
};

#endif // MAIN_WINDOW_H
