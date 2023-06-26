#include "../include/rqt_tutorial.h"
#include <ui_rqt_tutorial.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>


namespace balazs_tutorials {

RQTTutorial::RQTTutorial() :
    rqt_gui_cpp::Plugin(),
    Ui(new Ui::RQTTutorialWidget),
    Widget(nullptr)
{
    setObjectName("RQTTutorial");
}

void RQTTutorial::initPlugin(qt_gui_cpp::PluginContext& context)
{
    Widget = new QWidget();
    Ui->setupUi(Widget);

    if (context.serialNumber() > 1) {
        Widget->setWindowTitle(Widget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    context.addWidget(Widget);

    connect(Ui->options_button_group, static_cast<void (QButtonGroup::*)(int)>(&QButtonGroup::buttonClicked), this, &RQTTutorial::onOptionsChanged);

    Ui->options_button_group->setId(Ui->option1_radio_button, 1);
    Ui->options_button_group->setId(Ui->option2_radio_button, 2);
    Ui->options_button_group->setId(Ui->option3_radio_button, 3);

    const std::string package_directory = ros::package::getPath("balazs_tutorials");

    ros::NodeHandle& node = getPrivateNodeHandle();

    Sub = node.subscribe<std_msgs::Empty>("topic_name", 1, boost::bind(&RQTTutorial::SubscriberCallback, this));
}

void RQTTutorial::shutdownPlugin()
{
    Sub.shutdown();
}

void RQTTutorial::onOptionsChanged(int option)
{
    Ui->option_line_edit->setText(QString::number(option));
}

void RQTTutorial::SubscriberCallback()
{
}

} // balazs_tutorials

PLUGINLIB_EXPORT_CLASS(balazs_tutorials::RQTTutorial, rqt_gui_cpp::Plugin)
