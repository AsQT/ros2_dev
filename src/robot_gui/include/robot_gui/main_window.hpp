#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include <QMainWindow>
#include <QImage>
#include <QStringList>
#include <QVector>

#include "robot_gui/robot_gui_node.hpp"

class QApplication;
class QLabel;
class QPushButton;
class QLineEdit;
class QTextEdit;

QT_BEGIN_NAMESPACE
namespace Ui
{
class RobotGUI_MainWindow;
}
QT_END_NAMESPACE

namespace robot_gui
{

class RvizPanel;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(
    QApplication * app,
    const std::shared_ptr<RobotGuiNode> & node,
    QWidget * parent = nullptr);
  ~MainWindow() override;

Q_SIGNALS:
  void flags_received(std::vector<uint32_t> flags);
  void joint_state_received(QStringList names, QVector<double> positions_rad, QVector<double> velocities_rad_s);
  void ros_log_received(QString message);
  void raw_image_received(QImage image);
  void detection_image_received(QImage image);
  void yolo_image_received(QImage image);

private Q_SLOTS:
  void update_axis_flags(std::vector<uint32_t> flags);
  void update_joint_state_display(QStringList names, QVector<double> positions_rad, QVector<double> velocities_rad_s);
  void append_ros_log(QString message);
  void update_raw_image(QImage image);
  void update_detection_image(QImage image);
  void update_yolo_image(QImage image);

private:
  void setup_defaults();
  void setup_image_display();
  void setup_navigation();
  void setup_robot_controls();
  void setup_axis_controls();
  void setup_logs();
  void log_robot_tab_widget_mapping();
  void log_joint_state_widget_mapping();
  void setup_rviz(QApplication * app);
  void set_current_page(int index);
  void update_navigation_buttons(int current_index);
  void update_robot_enable_button();
  void set_axis_leds(int axis, uint32_t status);
  void set_label_text(const QString & object_name, const QString & text);
  void set_image_placeholder(QLabel * label, const QString & title, const std::string & topic);
  void set_image_pixmap(QLabel * label, const QImage & image);
  bool emit_image(const sensor_msgs::msg::Image::SharedPtr & msg, const char * panel_name);
  QString line_text(const QString & object_name, const QString & fallback = QString()) const;
  QPushButton * button(const QString & object_name) const;
  QLineEdit * line_edit(const QString & object_name) const;
  QLabel * label(const QString & object_name) const;
  QTextEdit * text_edit(const QString & object_name) const;
  double axis_velocity_deg_s(int axis) const;
  void run_axis(int axis);

  std::unique_ptr<Ui::RobotGUI_MainWindow> ui_;
  std::shared_ptr<RobotGuiNode> node_;
  std::unique_ptr<RvizPanel> rviz_panel_;
  std::array<double, 6> commanded_deg_{};
  std::array<uint32_t, 6> last_flags_{};
  bool robot_servo_on_{false};
  bool raw_image_seen_{false};
  bool detection_image_seen_{false};
  bool yolo_image_seen_{false};
  QString robot_enable_default_style_;
  QString robot_disable_default_style_;
};

}  // namespace robot_gui
