#ifndef RQT_TUTORIAL_H
#define RQT_TUTORIAL_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_rqt_tutorial.h>


namespace balazs_tutorials {

class RQTTutorial : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public:
    RQTTutorial();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();

protected:
    Ui::RQTTutorialWidget Ui_;
    QWidget* Widget;
};

} // balazs_tutorials

#endif // RQT_TUTORIAL_H
