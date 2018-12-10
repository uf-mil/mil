#ifndef _UI_H
#define _UI_H

#include <nav_msgs/Odometry.h>
#include <navigator_msgs/ScanTheCode.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <QTimer>
#include "ui_UI.h"

#include <ctime>
#include <iostream>
#include <random>

class UI : public QMainWindow
{
  Q_OBJECT
public:
  UI();
  ~UI() override;
  QTimer* timer;
public slots:
  void slotTimer();

public:
  ros::NodeHandle* nh;
  ros::Subscriber sub;
  void scanTheCodeCallback(const navigator_msgs::ScanTheCode& scan_the_code);
  std::string color_pattern = "   ";

private:
  Ui::UI widget;
};

#endif /* _UI_H */
