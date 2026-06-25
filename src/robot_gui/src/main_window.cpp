#include "robot_gui/main_window.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <utility>

#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QFrame>
#include <QFont>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QPixmap>
#include <QPushButton>
#include <QStackedWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/logging.hpp"
#include "robot_gui/rviz_panel.hpp"
#include "robot_gui/status_led.hpp"
#include "ui_robot_gui.h"

namespace robot_gui
{
namespace
{
constexpr int kJointCount = 6;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr uint32_t ERROR_ALL = 0x00000001;
constexpr uint32_t SOF_LIMIT_P = 0x00000008;
constexpr uint32_t SOF_LIMIT_M = 0x00000010;
constexpr uint32_t EMG = 0x00010000;
constexpr uint32_t SERVO_ON = 0x00100000;
constexpr uint32_t ALARM = 0x00200000;
constexpr uint32_t ORG_SET_OK = 0x02000000;
constexpr uint32_t RUNNING = 0x08000000;
constexpr const char * kNormalInactive = "#808080";
constexpr const char * kNormalActive = "#00cc66";
constexpr const char * kFaultInactive = "#95c7ea";
constexpr const char * kFaultActive = "#ff3333";

QString timestamp()
{
  const auto now = std::time(nullptr);
  char buffer[32]{};
  std::strftime(buffer, sizeof(buffer), "%H:%M:%S", std::localtime(&now));
  return QString::fromLatin1(buffer);
}

bool ros_image_to_qimage(const sensor_msgs::msg::Image & msg, QImage * image)
{
  if (image == nullptr || msg.width == 0 || msg.height == 0 || msg.step == 0) {
    return false;
  }

  const auto required_size = static_cast<size_t>(msg.step) * static_cast<size_t>(msg.height);
  if (msg.data.size() < required_size) {
    return false;
  }

  const auto width = static_cast<int>(msg.width);
  const auto height = static_cast<int>(msg.height);
  const auto step = static_cast<int>(msg.step);
  const auto * data = msg.data.data();

  if (msg.encoding == "rgb8") {
    *image = QImage(data, width, height, step, QImage::Format_RGB888).copy();
    return true;
  }
  if (msg.encoding == "bgr8") {
    *image = QImage(data, width, height, step, QImage::Format_RGB888).rgbSwapped();
    return true;
  }
  if (msg.encoding == "mono8") {
    *image = QImage(data, width, height, step, QImage::Format_Grayscale8).copy();
    return true;
  }
  if (msg.encoding == "rgba8") {
    *image = QImage(data, width, height, step, QImage::Format_RGBA8888).copy();
    return true;
  }
  if (msg.encoding == "bgra8") {
    *image = QImage(data, width, height, step, QImage::Format_ARGB32).copy();
    return true;
  }
  return false;
}
}  // namespace

MainWindow::MainWindow(
  QApplication * app,
  const std::shared_ptr<RobotGuiNode> & node,
  QWidget * parent)
: QMainWindow(parent), ui_(std::make_unique<Ui::RobotGUI_MainWindow>()), node_(node)
{
  const auto share_dir =
    QString::fromStdString(ament_index_cpp::get_package_share_directory("robot_gui"));
  const auto ui_dir = share_dir + "/ui";
  const QDir old_dir = QDir::current();
  QDir::setCurrent(ui_dir);
  ui_->setupUi(this);
  QDir::setCurrent(old_dir.absolutePath());
  qRegisterMetaType<std::vector<uint32_t>>("std::vector<uint32_t>");
  qRegisterMetaType<QStringList>("QStringList");
  qRegisterMetaType<QVector<double>>("QVector<double>");
  qRegisterMetaType<QImage>("QImage");
  connect(this, &MainWindow::flags_received, this, &MainWindow::update_axis_flags);
  connect(this, &MainWindow::joint_state_received, this, &MainWindow::update_joint_state_display);
  connect(this, &MainWindow::ros_log_received, this, &MainWindow::append_ros_log);
  connect(this, &MainWindow::raw_image_received, this, &MainWindow::update_raw_image);
  connect(this, &MainWindow::detection_image_received, this, &MainWindow::update_detection_image);
  connect(this, &MainWindow::yolo_image_received, this, &MainWindow::update_yolo_image);

  node_->set_flag_callback([this](const std::vector<uint32_t> & flags) {
    Q_EMIT flags_received(flags);
  });
  node_->set_joint_state_callback(
    [this](
      const std::vector<std::string> & names,
      const std::vector<double> & positions_rad,
      const std::vector<double> & velocities_rad_s) {
      QStringList qt_names;
      qt_names.reserve(static_cast<int>(names.size()));
      for (const auto & name : names) {
        qt_names.push_back(QString::fromStdString(name));
      }
      QVector<double> qt_positions;
      qt_positions.reserve(static_cast<int>(positions_rad.size()));
      for (const double position : positions_rad) {
        qt_positions.push_back(position);
      }
      QVector<double> qt_velocities;
      qt_velocities.reserve(static_cast<int>(velocities_rad_s.size()));
      for (const double velocity : velocities_rad_s) {
        qt_velocities.push_back(velocity);
      }
      Q_EMIT joint_state_received(qt_names, qt_positions, qt_velocities);
    });
  node_->set_log_callback([this](const std::string & message) {
    Q_EMIT ros_log_received(QString::fromStdString(message));
  });
  node_->set_raw_image_callback([this](const sensor_msgs::msg::Image::SharedPtr & msg) {
    emit_image(msg, "raw");
  });
  node_->set_detection_image_callback([this](const sensor_msgs::msg::Image::SharedPtr & msg) {
    emit_image(msg, "detection");
  });
  node_->set_yolo_image_callback([this](const sensor_msgs::msg::Image::SharedPtr & msg) {
    emit_image(msg, "yolo");
  });

  setup_defaults();
  setup_image_display();
  setup_navigation();
  setup_robot_controls();
  setup_axis_controls();
  setup_logs();
  log_robot_tab_widget_mapping();
  log_joint_state_widget_mapping();
  setup_rviz(app);
}

MainWindow::~MainWindow() = default;

void MainWindow::setup_defaults()
{
  set_label_text("ConnectStatus_label", "Disconnected");
  set_label_text("PingStatus_value", "-- ms");
  set_label_text("PlannerStatus_value", "RL|Cartesian");
  set_status_led(label("ConnectStatus_led"), false);

  if (auto * enable = button("btnRobotEnable")) {
    robot_enable_default_style_ = enable->styleSheet();
  }
  if (auto * disable = button("btnRobotDisable")) {
    robot_disable_default_style_ = disable->styleSheet();
  }

  for (int axis = 1; axis <= kJointCount; ++axis) {
    set_label_text(QString("lblAxis%1Name").arg(axis), QString("Joint %1").arg(axis));
    set_label_text(QString("lblAxis%1ActualPosUnit").arg(axis), "deg");
    set_label_text(QString("lblAxis%1ActualVelUnit").arg(axis), "deg/s");
    set_label_text(QString("lblAxis%1CommandPosUnit").arg(axis), "deg");
    set_label_text(QString("lblAxis%1CommandVelUnit").arg(axis), "deg/s");
    if (auto * actual_pos = line_edit(QString("txtAxis%1ActualPos").arg(axis))) {
      actual_pos->setReadOnly(true);
      actual_pos->setText("0.000");
    }
    if (auto * actual_vel = line_edit(QString("txtAxis%1ActualVel").arg(axis))) {
      actual_vel->setReadOnly(true);
      actual_vel->setText("0.000");
    }
    if (auto * command_pos = line_edit(QString("txtAxis%1CommandPos").arg(axis))) {
      command_pos->setText("0.000");
    }
    if (auto * command_vel = line_edit(QString("txtAxis%1CommandVel").arg(axis))) {
      command_vel->setText("1.000");
    }
    set_axis_leds(axis, 0);
  }
  update_robot_enable_button();
}

void MainWindow::setup_image_display()
{
  set_status_led(label("ledCameraStatus"), false);

  set_image_placeholder(label("rawImageView"), "Raw Image", node_->raw_image_topic());
  set_image_placeholder(label("detectionImageView"), "Detection Image", node_->detection_image_topic());
  set_image_placeholder(label("yoloPreviewWidget"), "YOLO Image", node_->yolo_image_topic());

  if (auto * combo = findChild<QComboBox *>("cbImageTopic")) {
    combo->clear();
    const std::array<std::string, 3> topics = {
      node_->raw_image_topic(), node_->detection_image_topic(), node_->yolo_image_topic()};
    for (const auto & topic : topics) {
      if (!topic.empty()) {
        combo->addItem(QString::fromStdString(topic));
      }
    }
    if (combo->count() == 0) {
      combo->addItem("No topic configured");
    }
  }

  append_ros_log(
    QString("Image topics: raw=%1, detection=%2, yolo=%3")
      .arg(node_->raw_image_topic().empty() ? "No topic configured" :
        QString::fromStdString(node_->raw_image_topic()))
      .arg(node_->detection_image_topic().empty() ? "No topic configured" :
        QString::fromStdString(node_->detection_image_topic()))
      .arg(node_->yolo_image_topic().empty() ? "No topic configured" :
        QString::fromStdString(node_->yolo_image_topic())));
}

void MainWindow::setup_navigation()
{
  const std::vector<std::pair<QString, int>> pages = {
    {"btnHome", 0}, {"btnMain", 1}, {"btnRobot", 2},
    {"btnVision", 3}, {"btnSetting", 4}, {"btnLog", 5}};
  for (const auto & [name, index] : pages) {
    if (auto * btn = button(name)) {
      btn->setCheckable(true);
      connect(btn, &QPushButton::clicked, this, [this, index]() {set_current_page(index);});
    }
  }
  if (ui_->stackedWidget_MainPages != nullptr) {
    connect(
      ui_->stackedWidget_MainPages, &QStackedWidget::currentChanged,
      this, &MainWindow::update_navigation_buttons);
    const int initial = node_->initial_page();
    if (initial >= 0 && initial < ui_->stackedWidget_MainPages->count()) {
      ui_->stackedWidget_MainPages->setCurrentIndex(initial);
    }
    update_navigation_buttons(ui_->stackedWidget_MainPages->currentIndex());
  }
}

void MainWindow::setup_robot_controls()
{
  if (auto * btn = button("btnRobotEnable")) {
    connect(btn, &QPushButton::clicked, this, [this]() {
      node_->set_servo_all(!robot_servo_on_);
    });
  }
  if (auto * btn = button("btnRobotDisable")) {
    connect(btn, &QPushButton::clicked, this, [this]() {node_->set_servo_all(false);});
  }
  if (auto * btn = button("btnStopTask")) {
    connect(btn, &QPushButton::clicked, this, [this]() {node_->stop_all();});
  }
  if (auto * btn = button("btnStartTask")) {
    connect(btn, &QPushButton::clicked, this, [this]() {append_ros_log("Start task requested");});
  }
  if (auto * btn = button("btnResetTask")) {
    connect(btn, &QPushButton::clicked, this, [this]() {append_ros_log("Reset task requested");});
  }
  if (auto * btn = button("btnGripperOpen")) {
    connect(btn, &QPushButton::clicked, this, [this]() {node_->publish_gripper(0.04);});
  }
  if (auto * btn = button("btnGripperClose")) {
    connect(btn, &QPushButton::clicked, this, [this]() {
      node_->publish_gripper(line_text("txtGripperPosition", "0.0").toDouble());
    });
  }
  if (auto * btn = button("btnConnectRobot")) {
    connect(btn, &QPushButton::clicked, this, [this]() {
      set_label_text("ConnectStatus_label", "ROS mode");
      set_status_led(label("ConnectStatus_led"), true);
    });
  }
  if (auto * btn = button("btnDisconnectRobot")) {
    connect(btn, &QPushButton::clicked, this, [this]() {
      set_label_text("ConnectStatus_label", "Disconnected");
      set_status_led(label("ConnectStatus_led"), false);
    });
  }
  if (auto * btn = button("btnPingRobot")) {
    connect(btn, &QPushButton::clicked, this, [this]() {
      set_label_text("PingStatus_value", "ROS OK");
    });
  }
}

void MainWindow::setup_axis_controls()
{
  for (int axis = 1; axis <= kJointCount; ++axis) {
    if (auto * btn = button(QString("btnAxis%1Enable").arg(axis))) {
      connect(btn, &QPushButton::clicked, this, [this]() {node_->set_servo_all(true);});
    }
    if (auto * btn = button(QString("btnAxis%1Home").arg(axis))) {
      connect(btn, &QPushButton::clicked, this, [this, axis]() {node_->home_axis(axis);});
    }
    if (auto * btn = button(QString("btnAxis%1AlarmReset").arg(axis))) {
      connect(btn, &QPushButton::clicked, this, [this, axis]() {
        append_ros_log(QString("Alarm reset for Joint %1 is not wired to a ROS service").arg(axis));
      });
    }
    if (auto * btn = button(QString("btnAxis%1Run").arg(axis))) {
      connect(btn, &QPushButton::clicked, this, [this, axis]() {run_axis(axis);});
    }
    if (auto * btn = button(QString("btnAxis%1Stop").arg(axis))) {
      connect(btn, &QPushButton::clicked, this, [this, axis]() {node_->stop_axis(axis);});
    }
    if (auto * btn = button(QString("btnAxis%1JogPositive").arg(axis))) {
      connect(btn, &QPushButton::pressed, this, [this, axis]() {
        node_->jog_axis(axis, 1, axis_velocity_deg_s(axis));
      });
      connect(btn, &QPushButton::released, this, [this, axis]() {node_->stop_axis(axis);});
    }
    if (auto * btn = button(QString("btnAxis%1JogNegative").arg(axis))) {
      connect(btn, &QPushButton::pressed, this, [this, axis]() {
        node_->jog_axis(axis, -1, axis_velocity_deg_s(axis));
      });
      connect(btn, &QPushButton::released, this, [this, axis]() {node_->stop_axis(axis);});
    }
  }
}

void MainWindow::setup_logs()
{
  const auto clear = [this]() {
    if (auto * log = text_edit("txtSystemLog")) {log->clear();}
    if (auto * log = text_edit("txtHardwareLog")) {log->clear();}
    if (auto * log = text_edit("txtROS2Log")) {log->clear();}
  };
  if (auto * btn = button("btnClearLog")) {
    connect(btn, &QPushButton::clicked, this, clear);
  }
}

void MainWindow::log_robot_tab_widget_mapping()
{
  const auto log_found = [this](const QString & label, const QString & object_name, bool found) {
    const QString message = QString("Robot tab widget mapping: %1 (%2): found %3")
      .arg(label, object_name, found ? "yes" : "no");
    if (!found) {
      RCLCPP_WARN(node_->get_logger(), "%s", message.toStdString().c_str());
    } else {
      RCLCPP_INFO(node_->get_logger(), "%s", message.toStdString().c_str());
    }
    append_ros_log(message);
  };

  log_found("btnRobotEnable", "btnRobotEnable", button("btnRobotEnable") != nullptr);
  log_found("btnRobotDisable", "btnRobotDisable", button("btnRobotDisable") != nullptr);
  log_found("btnStopTask", "btnStopTask", button("btnStopTask") != nullptr);

  for (int axis = 1; axis <= kJointCount; ++axis) {
    const std::vector<std::pair<QString, bool>> widgets = {
      {QString("btnAxis%1Enable").arg(axis), button(QString("btnAxis%1Enable").arg(axis)) != nullptr},
      {QString("btnAxis%1Home").arg(axis), button(QString("btnAxis%1Home").arg(axis)) != nullptr},
      {QString("btnAxis%1Run").arg(axis), button(QString("btnAxis%1Run").arg(axis)) != nullptr},
      {QString("btnAxis%1Stop").arg(axis), button(QString("btnAxis%1Stop").arg(axis)) != nullptr},
      {QString("btnAxis%1JogPositive").arg(axis), button(QString("btnAxis%1JogPositive").arg(axis)) != nullptr},
      {QString("btnAxis%1JogNegative").arg(axis), button(QString("btnAxis%1JogNegative").arg(axis)) != nullptr},
      {QString("txtAxis%1CommandPos").arg(axis), line_edit(QString("txtAxis%1CommandPos").arg(axis)) != nullptr},
      {QString("txtAxis%1CommandVel").arg(axis), line_edit(QString("txtAxis%1CommandVel").arg(axis)) != nullptr},
      {QString("ledAxis%1ServoOn").arg(axis), label(QString("ledAxis%1ServoOn").arg(axis)) != nullptr},
      {QString("ledAxis%1Running").arg(axis), label(QString("ledAxis%1Running").arg(axis)) != nullptr},
      {QString("ledAxis%1OrgOK").arg(axis), label(QString("ledAxis%1OrgOK").arg(axis)) != nullptr},
      {QString("ledAxis%1LimitPositive").arg(axis), label(QString("ledAxis%1LimitPositive").arg(axis)) != nullptr},
      {QString("ledAxis%1LimitNegative").arg(axis), label(QString("ledAxis%1LimitNegative").arg(axis)) != nullptr},
      {QString("ledAxis%1Alarm").arg(axis), label(QString("ledAxis%1Alarm").arg(axis)) != nullptr},
      {QString("ledAxis%1EMG").arg(axis), label(QString("ledAxis%1EMG").arg(axis)) != nullptr},
      {QString("ledAxis%1ErrorAll").arg(axis), label(QString("ledAxis%1ErrorAll").arg(axis)) != nullptr},
    };
    for (const auto & [object_name, found] : widgets) {
      log_found(QString("Axis%1").arg(axis), object_name, found);
    }
  }
}

void MainWindow::log_joint_state_widget_mapping()
{
  for (int axis = 1; axis <= kJointCount; ++axis) {
    const QString position_name = QString("txtAxis%1ActualPos").arg(axis);
    const QString velocity_name = QString("txtAxis%1ActualVel").arg(axis);
    const bool position_found = line_edit(position_name) != nullptr;
    const bool velocity_found = line_edit(velocity_name) != nullptr;
    const QString position_message =
      QString("Joint state display widget: Axis%1 position (%2): found %3")
        .arg(axis)
        .arg(position_name)
        .arg(position_found ? "yes" : "no");
    const QString velocity_message =
      QString("Joint state display widget: Axis%1 velocity (%2): found %3")
        .arg(axis)
        .arg(velocity_name)
        .arg(velocity_found ? "yes" : "no");
    if (position_found) {
      RCLCPP_INFO(node_->get_logger(), "%s", position_message.toStdString().c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "%s", position_message.toStdString().c_str());
    }
    if (velocity_found) {
      RCLCPP_INFO(node_->get_logger(), "%s", velocity_message.toStdString().c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "%s", velocity_message.toStdString().c_str());
    }
    append_ros_log(position_message);
    append_ros_log(velocity_message);
  }
}

void MainWindow::setup_rviz(QApplication * app)
{
  if (!node_->embed_rviz()) {
    if (ui_->labelRvizPlaceholder) {
      ui_->labelRvizPlaceholder->setText("RViz disabled");
    }
    return;
  }
  if (ui_->embeddedRvizWidget == nullptr) {
    append_ros_log("embeddedRvizWidget not found");
    return;
  }
  auto * layout = ui_->embeddedRvizWidget->layout();
  if (layout == nullptr) {
    layout = new QVBoxLayout(ui_->embeddedRvizWidget);
  }
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  if (ui_->labelRvizPlaceholder) {
    ui_->labelRvizPlaceholder->hide();
  }
  QTimer::singleShot(1500, this, [this]() {
    const bool robot_description = node_->has_node_name("robot_state_publisher");
    const bool robot_description_semantic = node_->has_node_name("move_group");
    const bool joint_states = node_->has_topic_name("/joint_states");
    RCLCPP_INFO(
      node_->get_logger(),
      "RViz inputs: robot_description available: %s, robot_description_semantic available: %s, "
      "joint_states available: %s",
      robot_description ? "yes" : "no",
      robot_description_semantic ? "yes" : "no",
      joint_states ? "yes" : "no");
    append_ros_log(
      QString("RViz inputs: robot_description=%1, robot_description_semantic=%2, joint_states=%3")
        .arg(robot_description ? "yes" : "no")
        .arg(robot_description_semantic ? "yes" : "no")
        .arg(joint_states ? "yes" : "no"));
    if (!robot_description && !robot_description_semantic) {
      const QString warning =
        "RViz enabled but robot_description provider was not detected. "
        "Run robot_moveit moveit_gui.launch.py or robot_bringup real.launch.py launch_gui:=true "
        "for a complete robot model setup.";
      RCLCPP_WARN(node_->get_logger(), "%s", warning.toStdString().c_str());
      append_ros_log(warning);
    }
  });
  rviz_panel_ = std::make_unique<RvizPanel>(app, node_, ui_->embeddedRvizWidget);
  if (!rviz_panel_->initialize(node_->rviz_config_package(), node_->rviz_config_relative_path())) {
    append_ros_log(rviz_panel_->last_error());
    if (ui_->labelRvizPlaceholder) {
      ui_->labelRvizPlaceholder->setText(rviz_panel_->last_error());
      ui_->labelRvizPlaceholder->show();
    }
  } else {
    layout->addWidget(rviz_panel_->widget());
    append_ros_log(
      QString("Embedded RViz native initialized. Config: %1, Fixed Frame: %2")
        .arg(rviz_panel_->config_path())
        .arg(rviz_panel_->fixed_frame().isEmpty() ? "<unknown>" : rviz_panel_->fixed_frame()));
  }
}

void MainWindow::set_current_page(int index)
{
  if (ui_->stackedWidget_MainPages && index >= 0 && index < ui_->stackedWidget_MainPages->count()) {
    ui_->stackedWidget_MainPages->setCurrentIndex(index);
  }
}

void MainWindow::update_navigation_buttons(int current_index)
{
  const std::vector<std::pair<QString, int>> pages = {
    {"btnHome", 0}, {"btnMain", 1}, {"btnRobot", 2},
    {"btnVision", 3}, {"btnSetting", 4}, {"btnLog", 5}};
  for (const auto & [name, index] : pages) {
    if (auto * btn = button(name)) {
      btn->setChecked(index == current_index);
    }
  }
}

void MainWindow::update_axis_flags(std::vector<uint32_t> flags)
{
  const int count = std::min<int>(kJointCount, flags.size());
  for (int i = 0; i < count; ++i) {
    last_flags_[i] = flags[i];
    set_axis_leds(i + 1, flags[i]);
  }
  for (int i = count; i < kJointCount; ++i) {
    last_flags_[i] = 0;
    set_axis_leds(i + 1, 0);
  }
  if (count < kJointCount) {
    append_ros_log(
      QString("/robot_hw/flags has %1 axes; expected %2. Missing axes set inactive.")
        .arg(count)
        .arg(kJointCount));
  }
  robot_servo_on_ = count > 0 &&
    std::all_of(last_flags_.begin(), last_flags_.end(), [](uint32_t status) {
      return (status & SERVO_ON) != 0;
    });
  update_robot_enable_button();
}

void MainWindow::update_robot_enable_button()
{
  set_status_led(label("RobotEnableStatus_led"), robot_servo_on_);
  if (auto * enable = button("btnRobotEnable")) {
    enable->setText(robot_servo_on_ ? "Disable" : "Enable");
    enable->setStyleSheet(robot_servo_on_ ? robot_disable_default_style_ : robot_enable_default_style_);
  }
  if (auto * disable = button("btnRobotDisable")) {
    disable->setStyleSheet(robot_disable_default_style_);
  }
}

void MainWindow::update_joint_state_display(
  QStringList names,
  QVector<double> positions_rad,
  QVector<double> velocities_rad_s)
{
  for (int axis = 1; axis <= kJointCount; ++axis) {
    const auto & joint_names = node_->joint_names();
    const QString joint_name = axis - 1 < static_cast<int>(joint_names.size()) ?
      QString::fromStdString(joint_names[axis - 1]) :
      QString("joint_%1").arg(axis);
    const int msg_index = names.indexOf(joint_name);
    if (msg_index < 0 || msg_index >= positions_rad.size()) {
      continue;
    }

    const double position_deg = positions_rad[msg_index] * kRadToDeg;
    if (auto * actual_pos = line_edit(QString("txtAxis%1ActualPos").arg(axis))) {
      actual_pos->setText(QString::number(position_deg, 'f', 3));
    }
    commanded_deg_[axis - 1] = position_deg;

    if (auto * actual_vel = line_edit(QString("txtAxis%1ActualVel").arg(axis))) {
      if (msg_index < velocities_rad_s.size()) {
        actual_vel->setText(QString::number(velocities_rad_s[msg_index] * kRadToDeg, 'f', 3));
      }
    }
  }
}

void MainWindow::update_raw_image(QImage image)
{
  set_image_pixmap(label("rawImageView"), image);
  set_status_led(label("ledCameraStatus"), true);
  if (!raw_image_seen_) {
    raw_image_seen_ = true;
    const QString message = QString("Raw Image received: %1x%2 from %3")
      .arg(image.width())
      .arg(image.height())
      .arg(QString::fromStdString(node_->raw_image_topic()));
    RCLCPP_INFO(node_->get_logger(), "%s", message.toStdString().c_str());
    append_ros_log(message);
  }
}

void MainWindow::update_detection_image(QImage image)
{
  set_image_pixmap(label("detectionImageView"), image);
  set_status_led(label("ledCameraStatus"), true);
  if (!detection_image_seen_) {
    detection_image_seen_ = true;
    const QString message = QString("Detection Image received: %1x%2 from %3")
      .arg(image.width())
      .arg(image.height())
      .arg(QString::fromStdString(node_->detection_image_topic()));
    RCLCPP_INFO(node_->get_logger(), "%s", message.toStdString().c_str());
    append_ros_log(message);
  }
}

void MainWindow::update_yolo_image(QImage image)
{
  set_image_pixmap(label("yoloPreviewWidget"), image);
  set_status_led(label("ledCameraStatus"), true);
  if (!yolo_image_seen_) {
    yolo_image_seen_ = true;
    const QString message = QString("YOLO Image received: %1x%2 from %3")
      .arg(image.width())
      .arg(image.height())
      .arg(QString::fromStdString(node_->yolo_image_topic()));
    RCLCPP_INFO(node_->get_logger(), "%s", message.toStdString().c_str());
    append_ros_log(message);
  }
}

void MainWindow::set_axis_leds(int axis, uint32_t status)
{
  set_led(label(QString("ledAxis%1ServoOn").arg(axis)), status & SERVO_ON, kNormalInactive, kNormalActive);
  set_led(label(QString("ledAxis%1Running").arg(axis)), status & RUNNING, kNormalInactive, kNormalActive);
  set_led(label(QString("ledAxis%1OrgOK").arg(axis)), status & ORG_SET_OK, kNormalInactive, kNormalActive);
  set_led(label(QString("ledAxis%1LimitPositive").arg(axis)), status & SOF_LIMIT_P, kNormalInactive, kNormalActive);
  set_led(label(QString("ledAxis%1LimitNegative").arg(axis)), status & SOF_LIMIT_M, kNormalInactive, kNormalActive);
  set_led(label(QString("ledAxis%1Alarm").arg(axis)), status & ALARM, kFaultInactive, kFaultActive);
  set_led(label(QString("ledAxis%1EMG").arg(axis)), status & EMG, kFaultInactive, kFaultActive);
  set_led(label(QString("ledAxis%1ErrorAll").arg(axis)), status & ERROR_ALL, kFaultInactive, kFaultActive);
}

void MainWindow::set_label_text(const QString & object_name, const QString & text)
{
  if (auto * object = findChild<QLabel *>(object_name)) {
    object->setText(text);
  }
}

void MainWindow::set_image_placeholder(QLabel * label, const QString & title, const std::string & topic)
{
  if (label == nullptr) {
    return;
  }
  QFont font = label->font();
  font.setPointSize(12);
  label->setFont(font);
  label->setWordWrap(true);
  label->setAlignment(Qt::AlignCenter);
  label->setScaledContents(false);
  label->clear();
  if (topic.empty()) {
    label->setText(QString("%1\nNo topic configured").arg(title));
  } else {
    label->setText(
      QString("%1\n%2\nWaiting for image...")
        .arg(title)
        .arg(QString::fromStdString(topic)));
  }
}

void MainWindow::set_image_pixmap(QLabel * label, const QImage & image)
{
  if (label == nullptr || image.isNull()) {
    return;
  }
  label->setText(QString());
  label->setAlignment(Qt::AlignCenter);
  label->setScaledContents(false);
  const QPixmap pixmap = QPixmap::fromImage(image);
  const QSize target_size = label->size();
  label->setPixmap(
    target_size.isEmpty() ? pixmap :
    pixmap.scaled(target_size, Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

bool MainWindow::emit_image(const sensor_msgs::msg::Image::SharedPtr & msg, const char * panel_name)
{
  if (!msg) {
    return false;
  }

  QImage image;
  if (!ros_image_to_qimage(*msg, &image)) {
    const QString message =
      QString("Unsupported or invalid %1 image: encoding=%2, size=%3x%4, step=%5")
        .arg(panel_name)
        .arg(QString::fromStdString(msg->encoding))
        .arg(msg->width)
        .arg(msg->height)
        .arg(msg->step);
    RCLCPP_WARN(node_->get_logger(), "%s", message.toStdString().c_str());
    Q_EMIT ros_log_received(message);
    return false;
  }

  const QString panel = QString::fromLatin1(panel_name);
  if (panel == "raw") {
    Q_EMIT raw_image_received(image);
  } else if (panel == "detection") {
    Q_EMIT detection_image_received(image);
  } else if (panel == "yolo") {
    Q_EMIT yolo_image_received(image);
  }
  return true;
}

QString MainWindow::line_text(const QString & object_name, const QString & fallback) const
{
  if (auto * object = findChild<QLineEdit *>(object_name)) {
    return object->text();
  }
  return fallback;
}

QPushButton * MainWindow::button(const QString & object_name) const
{
  return findChild<QPushButton *>(object_name);
}

QLineEdit * MainWindow::line_edit(const QString & object_name) const
{
  return findChild<QLineEdit *>(object_name);
}

QLabel * MainWindow::label(const QString & object_name) const
{
  return findChild<QLabel *>(object_name);
}

QTextEdit * MainWindow::text_edit(const QString & object_name) const
{
  return findChild<QTextEdit *>(object_name);
}

double MainWindow::axis_velocity_deg_s(int axis) const
{
  bool ok = false;
  const double value = line_text(QString("txtAxis%1CommandVel").arg(axis), "1.0").toDouble(&ok);
  return ok ? value : 1.0;
}

void MainWindow::run_axis(int axis)
{
  bool ok = false;
  const double position = line_text(QString("txtAxis%1CommandPos").arg(axis), "0.0").toDouble(&ok);
  const double target = ok ? position : 0.0;
  commanded_deg_[axis - 1] = target;
  node_->run_axis(axis, target, axis_velocity_deg_s(axis));
  node_->publish_joint_trajectory(commanded_deg_);
}

void MainWindow::append_ros_log(QString message)
{
  const QString line = QString("[%1] %2").arg(timestamp(), std::move(message));
  if (auto * log = text_edit("txtROS2Log")) {
    log->append(line);
  }
  if (auto * short_log = findChild<QLabel *>("txtMainLog")) {
    short_log->setText(line);
  }
}

}  // namespace robot_gui
