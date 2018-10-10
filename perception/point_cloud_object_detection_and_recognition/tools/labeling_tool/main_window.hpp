#pragma once

#include <QMainWindow>
#include <QtWidgets>

#include "label_model.hpp"

#include <memory>

// #include <opencv/cv.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv/highgui.h>

namespace Ui
{
class main_window;
}

class main_window : public QMainWindow
{
  Q_OBJECT

public:
  explicit main_window(QWidget* parent = 0);
  ~main_window();

  void update_image(const QImage& image);
  void set_object_map(std::shared_ptr<id_to_labeled_object>);
  void populate_list();
  bool restart();
  bool generate_bag();

  /// Callbacks
  void combo_box_changed(const QString& string);
  void cell_changed(int row, int column);

  void restart_action();
  void save_bag_action();

private:
  Ui::main_window* ui_;
  std::shared_ptr<id_to_labeled_object> object_map_ptr_;
  bool restart_clicked_ = false;
  bool save_bag_clicked_ = false;
};
