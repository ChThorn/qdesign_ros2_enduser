#include <QApplication>
#include "qdesign_ros2_enduser/DialogController.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    DialogController dialog;
    dialog.show();

    while (rclcpp::ok() && dialog.isVisible()) {
        app.processEvents();
        rclcpp::spin_some(dialog.get_node_base_interface());
    }

    rclcpp::shutdown();
    return 0;
}
