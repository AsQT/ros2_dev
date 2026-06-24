#include "robot_gui/status_led.hpp"

#include <QLabel>

namespace robot_gui
{

void set_led(QLabel * label, bool active, const QString & inactive, const QString & active_color)
{
  if (label == nullptr) {
    return;
  }
  const QString color = active ? active_color : inactive;
  const QString border = active ? "#1f7a45" : "#3a3a3a";
  label->setText("");
  label->setVisible(true);
  label->setMinimumSize(14, 14);
  label->setMaximumSize(18, 18);
  label->setStyleSheet(
    QString("background-color: %1; border: 1px solid %2; border-radius: 7px;")
      .arg(color, border));
}

void set_status_led(QLabel * label, bool active)
{
  set_led(label, active, "#d50000", "#00c853");
}

}  // namespace robot_gui
