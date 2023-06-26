#ifndef RQT_TUTORIAL_H
#define RQT_TUTORIAL_H

#include <rqt_gui_cpp/plugin.h>

#include <ros/ros.h>


// Forward declarations
namespace Ui {
    class RQTTutorialWidget;
}

namespace balazs_tutorials {

class RQTTutorial : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public:
    RQTTutorial();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();

protected slots:
    void onOptionsChanged(int option);

// ROS callbacks
protected:
    void SubscriberCallback();

protected:
    Ui::RQTTutorialWidget* Ui;
    QWidget* Widget;

    ros::Subscriber Sub;
};

} // balazs_tutorials

#endif // RQT_TUTORIAL_H
