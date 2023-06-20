#include "../include/rqt_tutorial.h"

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <ros/package.h>


namespace balazs_tutorials {

RQTTutorial::RQTTutorial() :
    rqt_gui_cpp::Plugin(),
    Widget(nullptr)
{
    setObjectName("RQTTutorial");
}

void RQTTutorial::initPlugin(qt_gui_cpp::PluginContext& context)
{
    Widget = new QWidget();
    Ui_.setupUi(Widget);

    if (context.serialNumber() > 1) {
        Widget->setWindowTitle(Widget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    context.addWidget(Widget);

    const std::string package_directory_ = ros::package::getPath("balazs_tutorials");

    ros::NodeHandle& node = getPrivateNodeHandle();
}

void RQTTutorial::shutdownPlugin()
{
}

} // balazs_tutorials

PLUGINLIB_EXPORT_CLASS(balazs_tutorials::RQTTutorial, rqt_gui_cpp::Plugin)
