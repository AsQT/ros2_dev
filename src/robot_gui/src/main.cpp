#include <memory>
#include <atomic>
#include <thread>
#include <vector>

#include <QApplication>
#include <QCoreApplication>
#include <QMetaType>
#include <QMetaObject>
#include <QObject>

#include "rclcpp/rclcpp.hpp"
#include "robot_gui/main_window.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  qRegisterMetaType<std::vector<uint32_t>>("std::vector<uint32_t>");

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<robot_gui::RobotGuiNode>(node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  std::atomic_bool quit_requested{false};
  rclcpp::on_shutdown([&app, &quit_requested]() {
    if (!quit_requested.exchange(true)) {
      QMetaObject::invokeMethod(&app, "quit", Qt::QueuedConnection);
    }
  });
  QObject::connect(&app, &QCoreApplication::aboutToQuit, [&executor, &quit_requested]() {
    quit_requested.store(true);
    executor.cancel();
  });

  auto window = std::make_unique<robot_gui::MainWindow>(&app, node);
  window->show();
  const int result = app.exec();

  window.reset();
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return result;
}
