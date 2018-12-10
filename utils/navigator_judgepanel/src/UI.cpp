#include "UI.h"
#include <map>
#include <sstream>
#include <string>

UI::UI()
{
  // Start node
  nh = new ros::NodeHandle;

  sub = nh->subscribe("/scan_the_code", 1, &UI::scanTheCodeCallback, this);

  // Setup QT
  widget.setupUi(this);

  widget.label_ScanCode->setAlignment(Qt::AlignCenter);

  this->setStyleSheet("background-color: gray");

  QPalette white(palette());
  white.setColor(QPalette::Background, Qt::white);

  widget.color1->setStyleSheet("background-color: white");
  widget.color2->setStyleSheet("background-color: white");
  widget.color3->setStyleSheet("background-color: white");

  widget.frameCode->setStyleSheet("background-color: white");

  timer = new QTimer();
  QObject::connect(timer, SIGNAL(timeout()), this, SLOT(slotTimer()));
  timer->start(1000);
}

UI::~UI()
{
  delete timer;
  delete nh;
}

void UI::scanTheCodeCallback(const navigator_msgs::ScanTheCode &scan_the_code)
{
  this->color_pattern = scan_the_code.color_pattern;
}

void UI::slotTimer()
{
  // Let ROS update
  ros::spinOnce();

  // Scan the code GUI
  std::array<QLabel *, 3> l = { widget.label_Color1, widget.label_Color2, widget.label_Color3 };
  std::array<QFrame *, 3> w = { widget.color1, widget.color2, widget.color3 };

  if (this->color_pattern.length() != 3)
    return;

  for (size_t ii = 0; ii <= 2; ++ii)
  {
    switch (color_pattern.at(ii))
    {
      case 'G':
        l[ii]->setText(tr("GREEN"));
        w[ii]->setStyleSheet("background-color: green");
        break;
      case 'R':
        l[ii]->setText(tr("RED"));
        w[ii]->setStyleSheet("background-color: red");
        break;
      case 'B':
        l[ii]->setText(tr("BLUE"));
        w[ii]->setStyleSheet("background-color: blue");
        break;
      default:
        l[ii]->setText(tr(""));
        w[ii]->setStyleSheet("background-color: white");
        break;
    }
  }
}
