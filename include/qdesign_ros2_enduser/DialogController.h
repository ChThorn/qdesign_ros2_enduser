#ifndef DIALOGCONTROLLER_H
#define DIALOGCONTROLLER_H

#include <QDialog>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "DialogTest.h"

class DialogController : public QDialog, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit DialogController(QWidget *parent = nullptr);
    ~DialogController();

private slots:
    void onCancelButtonClicked();
    void onConfirmButtonClicked();
    void processFrame();

private:
    Ui::Dialog *ui;
    QTimer *timer;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    bool cameraActive;
    cv::Mat latest_frame;
    std::mutex frame_mutex;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void startCamera();
    void stopCamera();
    void updateFrame(const cv::Mat& frame);
};

#endif // DIALOGCONTROLLER_H