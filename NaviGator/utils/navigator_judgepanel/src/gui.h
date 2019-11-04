#ifndef GUI_H
#define GUI_H

#include <QMainWindow>

class GUI : public QMainWindow
{
  Q_OBJECT

public:
  GUI(QWidget *parent = 0);
  ~GUI();
};

#endif  // GUI_H
