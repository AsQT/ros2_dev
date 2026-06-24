#pragma once

#include <QString>

class QLabel;

namespace robot_gui
{

void set_led(QLabel * label, bool active, const QString & inactive, const QString & active_color);
void set_status_led(QLabel * label, bool active);

}  // namespace robot_gui
