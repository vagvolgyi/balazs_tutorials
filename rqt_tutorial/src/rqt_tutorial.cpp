#include "../include/ros_gui_tracker.h"

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tfMessage.h>
#include <std_msgs/Float64MultiArray.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QSet>
#include <QThread>

#include <mtMarkerTracker.h>


namespace marker_tracker {

TrackingWorker::TrackingWorker(ROSGUITracker* parent) :
    Parent(parent)
{
}

TrackingWorker::~TrackingWorker()
{
}

void TrackingWorker::process()
{
    cv::Mat video_frame, results_image;
    ros::Time timestamp, prev_timestamp;

    while (Parent->ProcessingActive) {
        QMutexLocker ml_inputs(&Parent->InputsMutex);
        if (timestamp.toNSec() == Parent->VideoFrameTimestamp.toNSec()) {
            Parent->VideoFrameUpdated.wait(&Parent->InputsMutex);
        }
        timestamp = Parent->VideoFrameTimestamp;
        Parent->VideoFrame.copyTo(video_frame);
        ml_inputs.unlock();

        if (timestamp.toNSec() == prev_timestamp.toNSec() || video_frame.empty()) continue;
        prev_timestamp = timestamp;

        bool target_found;
        if (Parent->Tracker->ProcessFrame(video_frame, target_found) == false) {
            emit error("\nFailed to process video frame.\n");
            break;
        }

        if (Parent->Ui_.show_image_check_box->isChecked()) {
            const bool show_markers = Parent->Ui_.show_markers_check_box->isChecked();
            const bool show_results = Parent->Ui_.show_results_check_box->isChecked();
            if (show_markers || show_results) {
                Parent->Tracker->DrawMaskOnRectifiedImage();
            }
            if (show_markers) {
                Parent->Tracker->DrawObservationsOnRectifiedImage();
                //Tracker->DrawPredictedObservationsOnRectifiedImage();
            }
            if (show_results && target_found) {
                Parent->Tracker->DrawTrackingResultsOnRectifiedImage(mt::Vector3uc(255, 0, 0), mt::Vector3uc(0, 255, 0), mt::Vector3uc(255, 255, 255));
            }
            if (show_markers || show_results) {
                Parent->Tracker->GetRectifiedImage().copyTo(results_image);
            } else {
                video_frame.copyTo(results_image);
            }
        }

        float success_code = Parent->Tracker->GetSuccessCode();
        mt::Vector3d trans_cam_wrt_ref, trans_mark_wrt_ref;
        mt::Quaternion rot_cam_wrt_ref, rot_mark_wrt_ref;
        Parent->Tracker->GetCameraPoseWrtRef(trans_cam_wrt_ref, rot_cam_wrt_ref);
        Parent->Tracker->GetMarkerPoseWrtRef(trans_mark_wrt_ref, rot_mark_wrt_ref);
        mt::Vector2d cluster_pos_on_world_plane = Parent->Tracker->GetClusterPositionOnWorldPlane();

        // Publish TF frames
        tf::StampedTransform frame;
        frame.setOrigin(tf::Vector3(trans_cam_wrt_ref.x(), trans_cam_wrt_ref.y(), trans_cam_wrt_ref.z()));
        frame.setRotation(tf::Quaternion(rot_cam_wrt_ref.x(), rot_cam_wrt_ref.y(), rot_cam_wrt_ref.z(), rot_cam_wrt_ref.w()));
        frame.stamp_ = timestamp;
        frame.child_frame_id_ = Parent->CameraFrame;
        frame.frame_id_ = Parent->ReferenceFrame;
        Parent->TFPublisher.sendTransform(frame);
        if (target_found) {
            const mt::Matrix4d& marker_pose_wrt_cam = Parent->Tracker->GetMarkerPoseWrtCamera();
            mt::Vector3d trans_mark_wrt_cam(marker_pose_wrt_cam.GetVector<0, 3, mt::Vertical, 3>());
            mt::Quaternion rot_mark_wrt_cam(mt::RotMat(marker_pose_wrt_cam.GetSubMat<0, 0, 3, 3>()));
            frame.setOrigin(tf::Vector3(trans_mark_wrt_cam.x(), trans_mark_wrt_cam.y(), trans_mark_wrt_cam.z()));
            frame.setRotation(tf::Quaternion(rot_mark_wrt_cam.x(), rot_mark_wrt_cam.y(), rot_mark_wrt_cam.z(), rot_mark_wrt_cam.w()));
            frame.child_frame_id_ = Parent->MarkerFrame;
            frame.frame_id_ = Parent->CameraFrame;
            Parent->TFPublisher.sendTransform(frame);
        }

        // Publish tracking results
        std_msgs::Float64MultiArray tracking_results_array;
        tracking_results_array.data.clear();
        tracking_results_array.data.push_back(success_code);
        tracking_results_array.data.push_back(static_cast<float>(trans_cam_wrt_ref.x()));
        tracking_results_array.data.push_back(static_cast<float>(trans_cam_wrt_ref.y()));
        tracking_results_array.data.push_back(static_cast<float>(trans_cam_wrt_ref.z()));
        tracking_results_array.data.push_back(static_cast<float>(rot_cam_wrt_ref.x()));
        tracking_results_array.data.push_back(static_cast<float>(rot_cam_wrt_ref.y()));
        tracking_results_array.data.push_back(static_cast<float>(rot_cam_wrt_ref.z()));
        tracking_results_array.data.push_back(static_cast<float>(rot_cam_wrt_ref.w()));
        tracking_results_array.data.push_back(static_cast<float>(trans_mark_wrt_ref.x()));
        tracking_results_array.data.push_back(static_cast<float>(trans_mark_wrt_ref.y()));
        tracking_results_array.data.push_back(static_cast<float>(trans_mark_wrt_ref.z()));
        tracking_results_array.data.push_back(static_cast<float>(rot_mark_wrt_ref.x()));
        tracking_results_array.data.push_back(static_cast<float>(rot_mark_wrt_ref.y()));
        tracking_results_array.data.push_back(static_cast<float>(rot_mark_wrt_ref.z()));
        tracking_results_array.data.push_back(static_cast<float>(rot_mark_wrt_ref.w()));
        tracking_results_array.data.push_back(static_cast<float>(cluster_pos_on_world_plane.x()));
        tracking_results_array.data.push_back(static_cast<float>(cluster_pos_on_world_plane.y()));
        tracking_results_array.data.push_back(timestamp.toNSec());
        Parent->TrackingResultsPublisher.publish(tracking_results_array);

        QString fps_text, frames_text, success_rate_text;
        fps_text.sprintf("%.2f fps", 1.0 / (static_cast<double>(cv::getTickCount() - Parent->StartTime)
                                            / cv::getTickFrequency()
                                            / (Parent->Tracker->GetTotalFrameCount() - Parent->StartFrame)));
        frames_text.sprintf("%d frames", Parent->Tracker->GetTotalFrameCount() - Parent->StartFrame);
        success_rate_text.sprintf("%.2f%%", static_cast<double>(Parent->Tracker->GetSuccessFrameCount()) / Parent->Tracker->GetTotalFrameCount() * 100.0);

        QMutexLocker ml_results(&Parent->ResultsMutex);
        Parent->FpsLabelText = fps_text;
        Parent->FramesLabelText = frames_text;
        Parent->SuccessRateLabelText = success_rate_text;
        results_image.copyTo(Parent->ResultsImage);
        ml_results.unlock();

        frame_processed();
    }

    emit finished();
}

ROSGUITracker::ROSGUITracker() :
    rqt_gui_cpp::Plugin(),
    Widget(nullptr),
    ReferenceFrame("/reference"),
    CameraFrame("/camera"),
    MarkerFrame("/marker"),
    ResultsTopic("/marker_tracker_results"),
    ImageCaptureInProgress(false),
    Tracker(nullptr),
    ProcessingActive(false)
{
    setObjectName("ROSGUITracker");
}

void ROSGUITracker::initPlugin(qt_gui_cpp::PluginContext& context)
{
    Widget = new QWidget();
    Ui_.setupUi(Widget);

    if (context.serialNumber() > 1) {
        Widget->setWindowTitle(Widget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    context.addWidget(Widget);

    const std::string package_directory_ = ros::package::getPath("marker_tracker");

    updateTopicList();

    Ui_.topics_combo_box->setCurrentIndex(Ui_.topics_combo_box->findText(""));
    connect(Ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

    Ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
    connect(Ui_.refresh_topics_push_button, SIGNAL(clicked()), this, SLOT(updateTopicList()));

    connect(Ui_.start_push_button, SIGNAL(clicked()), this, SLOT(onStartPressed()));
    connect(Ui_.config_push_button, SIGNAL(clicked()), this, SLOT(onConfigPressed()));
    connect(Ui_.show_image_check_box, SIGNAL(stateChanged(int)), this, SLOT(onShowImageStateChanged(int)));

    RecentConfigDir = QDir::homePath();

    ShowResultsStateBackup = Ui_.show_results_check_box->isChecked();
    ShowMarkersStateBackup = Ui_.show_markers_check_box->isChecked();

    ros::NodeHandle& node = getPrivateNodeHandle();
    TrackingResultsPublisher = node.advertise<std_msgs::Float64MultiArray>(ResultsTopic, 100);
}

void ROSGUITracker::shutdownPlugin()
{
    ProcessingActive = false;

    QMutexLocker ml_inputs(&InputsMutex);
    VideoFrameUpdated.wakeAll();
    ml_inputs.unlock();

    QThread::sleep(1);
    ImageSubscriber.shutdown();
    delete Tracker;
}

void ROSGUITracker::updateTopicList()
{
    QSet<QString> message_types;
    message_types.insert("sensor_msgs/Image");
    QSet<QString> message_sub_types;
    message_sub_types.insert("sensor_msgs/CompressedImage");

    // get declared transports
    QList<QString> transports;
    image_transport::ImageTransport it(getNodeHandle());
    std::vector<std::string> declared = it.getDeclaredTransports();
    for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it ++) {
        QString transport = it->c_str();

        // strip prefix from transport name
        QString prefix = "image_transport/";
        if (transport.startsWith(prefix)) {
            transport = transport.mid(prefix.length());
        }
        transports.append(transport);
    }

    QString selected = Ui_.topics_combo_box->currentText();

    // fill combo box
    QList<QString> topics = getTopics(message_types, message_sub_types, transports).values();
    topics.append("");
    qSort(topics);
    Ui_.topics_combo_box->clear();
    for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it ++) {
        QString label(*it);
        label.replace(" ", "/");
        Ui_.topics_combo_box->addItem(label, QVariant(*it));
    }

    // restore previous selection
    selectTopic(selected);
}

void ROSGUITracker::onTopicChanged(int index)
{
    ImageSubscriber.shutdown();

    // reset image on topic change
    Ui_.image_frame->SetImage(QImage());

    QStringList parts = Ui_.topics_combo_box->itemData(index).toString().split(" ");
    QString topic = parts.first();
    QString transport = parts.length() == 2 ? parts.last() : "raw";

    if (!topic.isEmpty()) {
        image_transport::ImageTransport it(getNodeHandle());
        image_transport::TransportHints hints(transport.toStdString());
        try {
            ImageSubscriber = it.subscribe(topic.toStdString(), 1, &ROSGUITracker::callbackImage, this, hints);
        } catch (image_transport::TransportLoadException& e) {
            QMessageBox::warning(Widget, tr("Loading image transport plugin failed"), e.what());
        }
    }

    ImageCaptureInProgress = false;
}

QSet<QString> ROSGUITracker::getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports)
{
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);

    QSet<QString> all_topics;
    for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it ++) {
        all_topics.insert(it->name.c_str());
    }

    QSet<QString> topics;
    for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it ++) {
        if (message_types.contains(it->datatype.c_str())) {
            QString topic = it->name.c_str();

            // add raw topic
            topics.insert(topic);

            // add transport specific sub-topics
            for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt ++) {
                if (all_topics.contains(topic + "/" + *jt)) {
                    QString sub = topic + " " + *jt;
                    topics.insert(sub);
                }
            }
        }
        if (message_sub_types.contains(it->datatype.c_str())) {
            QString topic = it->name.c_str();
            int index = topic.lastIndexOf("/");
            if (index != -1) {
                topic.replace(index, 1, " ");
                topics.insert(topic);
            }
        }
    }
    return topics;
}

void ROSGUITracker::selectTopic(const QString& topic)
{
    int index = Ui_.topics_combo_box->findText(topic);
    if (index == -1) {
        index = Ui_.topics_combo_box->findText("");
    }
    Ui_.topics_combo_box->setCurrentIndex(index);
}

void ROSGUITracker::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
    try {
        // First let cv_bridge do its magic
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        ImageCaptureBuffer = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e) {
        try {
            // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
            if (msg->encoding == "CV_8UC3") {
                // assuming it is rgb
                ImageCaptureBuffer = cv_ptr->image;
            } else if (msg->encoding == "8UC1") {
                // convert gray to rgb
                cv::cvtColor(cv_ptr->image, ImageCaptureBuffer, CV_GRAY2RGB);
            } else {
                qWarning("ROSGUITracker::callbackImage() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
                Ui_.image_frame->SetImage(QImage());
                return;
            }
        } catch (cv_bridge::Exception& e) {
            qWarning("ROSGUITracker::callbackImage() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
            return;
        }
    }
    if (ImageCaptureBuffer.empty()) return;

    if (ProcessingActive) {
        QMutexLocker ml_inputs(&InputsMutex);
        ImageCaptureBuffer.copyTo(VideoFrame);
        VideoFrameTimestamp = msg->header.stamp;
        VideoFrameUpdated.wakeAll();
        ml_inputs.unlock();
    } else {
        QImage image(ImageCaptureBuffer.data, ImageCaptureBuffer.cols, ImageCaptureBuffer.rows, static_cast<int>(ImageCaptureBuffer.step[0]), QImage::Format_RGB888);
        Ui_.image_frame->SetImage(image);

        ImageCaptureInProgress = true;
    }
}

void ROSGUITracker::onStartPressed()
{
    if (!ImageCaptureInProgress) return;

    if (ProcessingActive) {
        ProcessingActive = false;
        Ui_.start_push_button->setText("Start");
        Ui_.config_push_button->setEnabled(true);
        Ui_.topics_combo_box->setEnabled(true);
        Ui_.refresh_topics_push_button->setEnabled(true);
        Ui_.fps_label->setText("--.-- fps");

        QMutexLocker ml_inputs(&InputsMutex);
        VideoFrameUpdated.wakeAll();
        ml_inputs.unlock();
    } else {
        ProcessingActive = true;
        Ui_.start_push_button->setText("Stop");
        Ui_.config_push_button->setEnabled(false);
        Ui_.topics_combo_box->setEnabled(false);
        Ui_.refresh_topics_push_button->setEnabled(false);
        StartTime = cv::getTickCount();
        StartFrame = 0;
        Tracker->ResetStats();

        QThread* thread = new QThread();
        TrackingWorker* worker = new TrackingWorker(this);
        worker->moveToThread(thread);
        connect(thread, &QThread::started, worker, &TrackingWorker::process);
        connect(worker, &TrackingWorker::frame_processed, this, &ROSGUITracker::onFrameProcessed);
        connect(worker, &TrackingWorker::finished, thread, &QThread::quit);
        connect(worker, &TrackingWorker::finished, worker, &TrackingWorker::deleteLater);
        connect(worker, &TrackingWorker::error, this, &ROSGUITracker::onError);
        connect(thread, &QThread::finished, thread, &QThread::deleteLater);
        thread->start();
    }
}

void ROSGUITracker::onConfigPressed()
{
    Ui_.config_push_button->setDown(false);

    delete Tracker;
    Tracker = nullptr;

    QString config_dir = QFileDialog::getExistingDirectory(Widget, tr("Open Configuration Directory"), RecentConfigDir, QFileDialog::ShowDirsOnly|QFileDialog::DontResolveSymlinks);
    if (config_dir.isEmpty()) {
        Ui_.config_label->setText("<select-config-directory>");
        Ui_.start_push_button->setEnabled(false);

        QMessageBox msg_box;
        msg_box.setWindowTitle("Marker Tracker");
        msg_box.setText("\nSelected directory does not exist.\n");
        msg_box.setStandardButtons(QMessageBox::Ok);
        msg_box.exec();
        return;
    }
    RecentConfigDir = config_dir;

    Tracker = new mt::MarkerTracker;

    if (Tracker->LoadConfiguration(config_dir.toStdString()) == false) {
        delete Tracker;
        Tracker = nullptr;

        Ui_.config_label->setText("<select-config-directory>");
        Ui_.start_push_button->setEnabled(false);

        QMessageBox msg_box;
        msg_box.setWindowTitle("Marker Tracker");
        msg_box.setText("\nFailed to open configuration files in selected directory.\n");
        msg_box.setStandardButtons(QMessageBox::Ok);
        msg_box.exec();
        return;
    }

    Ui_.config_label->setText(config_dir);

    if (Tracker) {
        Tracker->Release();
        if (Tracker->Initialize() == false) {
            delete Tracker;
            Tracker = nullptr;

            Ui_.start_push_button->setEnabled(false);

            QMessageBox msg_box;
            msg_box.setWindowTitle("Marker Tracker");
            msg_box.setText("\nFailed to initialize marker tracker.\nTry selecting a different configuration directory.\n");
            msg_box.setStandardButtons(QMessageBox::Ok);
            msg_box.exec();
            return;
        }

        Ui_.start_push_button->setEnabled(true);
    }
}

void ROSGUITracker::onShowImageStateChanged(int state)
{
    if (state == Qt::Checked) {
        Ui_.show_results_check_box->setChecked(ShowResultsStateBackup);
        Ui_.show_results_check_box->setEnabled(true);
        Ui_.show_markers_check_box->setChecked(ShowMarkersStateBackup);
        Ui_.show_markers_check_box->setEnabled(true);
    } else {
        ShowResultsStateBackup = Ui_.show_results_check_box->isChecked();
        Ui_.show_results_check_box->setChecked(false);
        Ui_.show_results_check_box->setEnabled(false);
        ShowMarkersStateBackup = Ui_.show_markers_check_box->isChecked();
        Ui_.show_markers_check_box->setChecked(false);
        Ui_.show_markers_check_box->setEnabled(false);
    }
}

void ROSGUITracker::onFrameProcessed()
{
    QString fps_text, frames_text, success_rate_text;
    QMutexLocker ml_results(&ResultsMutex);
    fps_text = FpsLabelText;
    frames_text = FramesLabelText;
    success_rate_text = SuccessRateLabelText;
    ResultsImage.copyTo(DisplayImage);
    ml_results.unlock();

    Ui_.fps_label->setText(fps_text);
    Ui_.frames_label->setText(frames_text);
    Ui_.success_rate_label->setText(success_rate_text);

    QImage image(DisplayImage.data, DisplayImage.cols, DisplayImage.rows, static_cast<int>(DisplayImage.step[0]), QImage::Format_RGB888);
    Ui_.image_frame->SetImage(image);
}

void ROSGUITracker::onError(QString err)
{
    QMessageBox msg_box;
    msg_box.setWindowTitle("Marker Tracker");
    msg_box.setText(err);
    msg_box.setStandardButtons(QMessageBox::Ok);
    msg_box.exec();

    ProcessingActive = false;
    Ui_.start_push_button->setText("Start");
    Ui_.config_push_button->setEnabled(true);
    Ui_.topics_combo_box->setEnabled(true);
    Ui_.refresh_topics_push_button->setEnabled(true);
    Ui_.fps_label->setText("--.-- fps");
}

} // marker_tracker

PLUGINLIB_EXPORT_CLASS(marker_tracker::ROSGUITracker, rqt_gui_cpp::Plugin)
