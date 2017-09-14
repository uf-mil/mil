/*
 * File:   UI.cpp
 * Author: darkknight
 *
 * Created on September 9, 2016, 10:01 AM
 */

#include "UI.h"
#include <map>
#include <sstream>
#include <string>

////////////////////////////////////////////////////////////
/// \brief ?
///
/// \param ?
/// \param ?
////////////////////////////////////////////////////////////
UI::UI()
  : engine(time(0))
  , randomColor(1, 3)
  , imageCircle("/usr/local/images/circle.png")
  , imageTriangle("/usr/local/images/triangle.png")
  , imageCruciform("/usr/local/images/cruciform.png")
  , nullPixmap("/usr/local/images/question.png")
{
  // Start ROS
  std::map<std::string, std::string> args;

  // Start node
  nh = new ros::NodeHandle;
  // sub = nh->subscribe("/odom", 1, &UI::odomCallBack, this);

  // Setup QT
  widget.setupUi(this);

  widget.label_ScanCode->setAlignment(Qt::AlignCenter);
  widget.label_FindBreak->setAlignment(Qt::AlignCenter);
  widget.label_CoralSurvey->setAlignment(Qt::AlignCenter);

  this->setStyleSheet("background-color: gray");

  QPalette white(palette());
  white.setColor(QPalette::Background, Qt::white);

  widget.color1->setStyleSheet("background-color: white");
  widget.color2->setStyleSheet("background-color: white");
  widget.color3->setStyleSheet("background-color: white");

  widget.frameCoral->setStyleSheet("background-color: white");
  widget.frameBreak->setStyleSheet("background-color: white");
  widget.frameCode->setStyleSheet("background-color: white");

  timer = new QTimer();
  QObject::connect(timer, SIGNAL(timeout()), this, SLOT(slotTimer()));
  timer->start(1000);

  widget.statusbar->setStyleSheet("font: bold 18px");
  widget.statusbar->showMessage(tr("ROS is alive..."));

  widget.graphics_Left->setPixmap(nullPixmap.scaled(125, 125));
  widget.graphics_Right->setPixmap(nullPixmap.scaled(125, 125));
}

////////////////////////////////////////////////////////////
/// \brief ?
///
/// \param ?
/// \param ?
////////////////////////////////////////////////////////////
UI::~UI()
{
}

////////////////////////////////////////////////////////////
/// \brief ?
///
/// \param ?
/// \param ?
////////////////////////////////////////////////////////////
/*
void UI::odomCallBack(const nav_msgs::OdometryConstPtr &odom) {
    //ROS_INFO("cb_odom...");
    boatPose_enu = odom->pose.pose;
    boatTwist_enu = odom->twist.twist;
    std::stringstream ss;
    ss << "Boat connected: " << boatPose_enu.position.x << "," << boatPose_enu.position.y << "," <<
boatPose_enu.position.z;
    widget.statusbar->showMessage(ss.str().c_str());
}*/

////////////////////////////////////////////////////////////
/// \brief ?
///
/// \param ?
/// \param ?
////////////////////////////////////////////////////////////
void UI::slotTimer()
{
  // Let ROS update
  ros::spinOnce();

  // Scan the code
  bool result;
  std::string colorCode;
  std::array<QLabel*, 3> l = { widget.label_Color1, widget.label_Color2, widget.label_Color3 };
  std::array<QFrame*, 3> w = { widget.color1, widget.color2, widget.color3 };
  for (auto ii = 1; ii <= 3; ++ii)
  {
    if (nh->getParam("/mission/scan_the_code/color" + std::to_string(ii), colorCode))
    {
      l[ii - 1]->setText(tr(colorCode.c_str()));
      if (colorCode == "GREEN")
      {
        w[ii - 1]->setStyleSheet("background-color: green");
      }
      else if (colorCode == "RED")
      {
        w[ii - 1]->setStyleSheet("background-color: red");
      }
      else if (colorCode == "BLUE")
      {
        w[ii - 1]->setStyleSheet("background-color: blue");
      }
      else if (colorCode == "YELLOW")
      {
        w[ii - 1]->setStyleSheet("background-color: yellow");
      }
    }
    else
    {
      l[ii - 1]->setText(tr(""));
      w[ii - 1]->setStyleSheet("background-color: white");
    }
  }

  // Coral Survey 1
  if (nh->getParam("/mission/coral_survey/shape1", colorCode))
  {
    widget.label->setText(tr(colorCode.c_str()));
    if (colorCode == "CIRCLE")
    {
      widget.graphics_Left->setPixmap(imageCircle.scaled(125, 125));
    }
    else if (colorCode == "TRIANGLE")
    {
      widget.graphics_Left->setPixmap(imageTriangle.scaled(125, 125));
    }
    else if (colorCode == "CROSS")
    {
      widget.graphics_Left->setPixmap(imageCruciform.scaled(125, 125));
    }
  }
  else
  {
    widget.label->setText(tr(""));
    widget.graphics_Left->setPixmap(nullPixmap.scaled(125, 125));
  }

  // Coral Survey 2
  if (nh->getParam("/mission/coral_survey/shape2", colorCode))
  {
    widget.label_2->setText(tr(colorCode.c_str()));
    if (colorCode == "CIRCLE")
    {
      widget.graphics_Right->setPixmap(imageCircle.scaled(125, 125));
    }
    else if (colorCode == "TRIANGLE")
    {
      widget.graphics_Right->setPixmap(imageTriangle.scaled(125, 125));
    }
    else if (colorCode == "CROSS")
    {
      widget.graphics_Right->setPixmap(imageCruciform.scaled(125, 125));
    }
  }
  else
  {
    widget.label_2->setText(tr(""));
    widget.graphics_Right->setPixmap(nullPixmap.scaled(125, 125));
  }

  // Check find the break
  int breaks;
  if (nh->getParam("/mission/find_the_break/markers", breaks))
  {
    widget.label_break->setText(tr(std::to_string(breaks).c_str()));
  }
  else
  {
    widget.label_break->setText(tr(""));
  }
}
