#ifndef ROS_GUI_TRACKER_H
#define ROS_GUI_TRACKER_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_ros_gui_tracker.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core.hpp>

#include <QWaitCondition>


namespace mt {
    class MarkerTracker;
}

namespace marker_tracker {

class ROSGUITracker;

class TrackingWorker : public QObject
{
    Q_OBJECT

public:
    TrackingWorker(ROSGUITracker* parent);
    ~TrackingWorker();

public slots:
    void process();

signals:
    void frame_processed();
    void finished();
    void error(QString err);

private:
    ROSGUITracker* Parent;
};

class ROSGUITracker : public rqt_gui_cpp::Plugin
{
    friend class TrackingWorker;

    Q_OBJECT

public:
    ROSGUITracker();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();

protected slots:
    virtual void updateTopicList();
    virtual void onTopicChanged(int index);
    virtual void onStartPressed();
    virtual void onConfigPressed();
    virtual void onShowImageStateChanged(int state);
    virtual void onFrameProcessed();
    virtual void onError(QString err);

protected:
    virtual QSet<QString> getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports);
    virtual void selectTopic(const QString& topic);
    virtual void callbackImage(const sensor_msgs::Image::ConstPtr& msg);

protected:
    Ui::ROSGUITrackerWidget Ui_;
    QWidget* Widget;

    const std::string ReferenceFrame;
    const std::string CameraFrame;
    const std::string MarkerFrame;
    const std::string ResultsTopic;

    image_transport::Subscriber ImageSubscriber;
    ros::Publisher TrackingResultsPublisher;
    tf::TransformBroadcaster TFPublisher;

    bool ImageCaptureInProgress;
    QMutex InputsMutex;
    QMutex ResultsMutex;
    QWaitCondition VideoFrameUpdated;
    cv::Mat ImageCaptureBuffer;
    cv::Mat VideoFrame;
    ros::Time VideoFrameTimestamp;

    QString RecentConfigDir;

    int64 StartTime;
    int StartFrame;

    QString FpsLabelText;
    QString FramesLabelText;
    QString SuccessRateLabelText;
    cv::Mat ResultsImage;
    cv::Mat DisplayImage;

    mt::MarkerTracker* Tracker;

    bool ProcessingActive;
    bool ShowResultsStateBackup;
    bool ShowMarkersStateBackup;

private:
    QString ArgTopicName;
};

} // marker_tracker

#endif // ROS_GUI_TRACKER_H
