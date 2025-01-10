#include "qdesign_ros2_enduser/DialogController.h"
#include <QPixmap>
#include <QImage>
#include <QPainter>
#include <opencv2/opencv.hpp>

DialogController::DialogController(QWidget *parent)
    : QDialog(parent), Node("qt_realsense_node"), ui(new Ui::Dialog), cameraActive(false)
{
    ui->setupUi(this);

    // Set fixed window size
    setFixedSize(1024, 768);

    connect(ui->CancelBTN, &QPushButton::clicked, this, &DialogController::onCancelButtonClicked);
    connect(ui->ConfirmBTN, &QPushButton::clicked, this, &DialogController::onConfirmButtonClicked);

    ui->ConfirmBTN->setText("Start Camera");
    ui->CancelBTN->setText("Stop Camera");
    ui->CancelBTN->setEnabled(false);

    // Set initial label text and styling
    ui->label->setText("No Camera Feed");
    ui->label->setStyleSheet("QLabel { background-color: black; color: white; }");

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &DialogController::processFrame);
    timer->start(33); // ~30 FPS
}

DialogController::~DialogController()
{
    stopCamera();
    delete ui;
}

void DialogController::onConfirmButtonClicked()
{
    if (!cameraActive) {
        startCamera();
    }
}

void DialogController::onCancelButtonClicked()
{
    if (cameraActive) {
        stopCamera();
    }
}

void DialogController::startCamera()
{
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", 10,
        std::bind(&DialogController::imageCallback, this, std::placeholders::_1));

    cameraActive = true;
    ui->ConfirmBTN->setEnabled(false);
    ui->CancelBTN->setEnabled(true);
}

void DialogController::stopCamera()
{
    image_sub_.reset();
    cameraActive = false;
    ui->ConfirmBTN->setEnabled(true);
    ui->CancelBTN->setEnabled(false);

    // Clear the label and reset to default state
    std::lock_guard<std::mutex> lock(frame_mutex);
    latest_frame = cv::Mat();
    ui->label->clear();
    ui->label->setText("No Camera Feed");
}

void DialogController::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        std::lock_guard<std::mutex> lock(frame_mutex);
        latest_frame = cv_ptr->image.clone();
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void DialogController::processFrame()
{
    if (!cameraActive) return;

    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        if (latest_frame.empty()) return;
        frame = latest_frame.clone();
    }

    updateFrame(frame);
}

void DialogController::updateFrame(const cv::Mat& frame)
{
    // Get the label's size
    QSize labelSize = ui->label->size();
    
    // Create QImage from the OpenCV frame
    QImage qimg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_BGR888);
    
    // Scale the image to fit the label while maintaining aspect ratio
    QPixmap pixmap = QPixmap::fromImage(qimg)
        .scaled(labelSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    
    // Calculate position to center the image in the label
    int x = (labelSize.width() - pixmap.width()) / 2;
    int y = (labelSize.height() - pixmap.height()) / 2;
    
    // Create a black background pixmap
    QPixmap background(labelSize);
    background.fill(Qt::black);
    
    // Create a painter to draw on the background
    QPainter painter(&background);
    painter.drawPixmap(x, y, pixmap);
    painter.end();
    
    // Set the final pixmap to the label
    ui->label->setPixmap(background);
}
