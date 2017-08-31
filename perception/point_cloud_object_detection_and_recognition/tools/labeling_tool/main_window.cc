#include "main_window.hh"
#include <point_cloud_object_detection_and_recognition/pcodar_types.hh>
#include "ui_main_window.h"

#include <iostream>

uint get_class_index(const std::string& clss)
{
    return std::find(pcodar::classification_strings.begin(), pcodar::classification_strings.end(), clss) -
           pcodar::classification_strings.begin();
}

main_window::main_window(QWidget* parent) : QMainWindow(parent), ui_(new Ui::main_window)
{
    ui_->setupUi(this);
    ui_->list_widget->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    connect(ui_->list_widget, &QTableWidget::cellClicked, this, &main_window::cell_changed);
    connect(ui_->save_bag_action, &QAction::triggered, this, &main_window::save_bag_action);
    connect(ui_->restart_action, &QAction::triggered, this, &main_window::restart_action);
}
main_window::~main_window() { delete ui_; }
void main_window::update_image(const QImage& image)
{
    // update images
    ui_->image_view->setPixmap(QPixmap::fromImage(image));
}

void main_window::combo_box_changed(const QString& string)
{
    QComboBox* combo = qobject_cast<QComboBox*>(sender());
    if (combo)
    {
        const int row = combo->property("row").toInt();
        std::string text = ui_->list_widget->item(row, 1)->text().toStdString();
        uint16_t id = std::stoi(text);
        const std::string clss = combo->currentText().toStdString();
        object_map_ptr_->at(id).p_object.classification = clss;
    }
}

void main_window::cell_changed(int row, int column)
{
    if (column == 0)
    {
        std::string text = ui_->list_widget->item(row, 1)->text().toStdString();
        uint16_t id = std::stoi(text);

        if (ui_->list_widget->item(row, column)->checkState() == Qt::Checked)
        {
            object_map_ptr_->at(id).visualize = true;
        }
        else
        {
            object_map_ptr_->at(id).visualize = false;
        }
    }
}

void main_window::set_object_map(std::shared_ptr<id_to_labeled_object> object_map_ptr)
{
    object_map_ptr_ = object_map_ptr;
}
void main_window::populate_list()
{
    for (const auto& id_object_pair : *object_map_ptr_)
    {
        const auto& c_object = id_object_pair.second;
        int row_id = ui_->list_widget->rowCount();
        ui_->list_widget->insertRow(row_id);

        QTableWidgetItem* visible = new QTableWidgetItem("");

        visible->setCheckState(Qt::Checked);
        if (!true)
        {
            visible->setCheckState(Qt::Unchecked);
        }

        ui_->list_widget->setItem(row_id, 0, visible);

        std::string id_string = std::to_string(c_object.p_object.id);
        QTableWidgetItem* item2 = new QTableWidgetItem(id_string.c_str());
        ui_->list_widget->setItem(row_id, 1, item2);

        QStringList commands;
        for (const std::string& clss_str : pcodar::classification_strings)
        {
            commands.push_back(clss_str.c_str());
        }

        QComboBox* combo = new QComboBox();
        combo->addItems(commands);
        combo->setProperty("row", row_id);
        ui_->list_widget->setCellWidget(row_id, 2, combo);
        combo->setCurrentIndex(get_class_index(c_object.p_object.classification));
        connect(combo, &QComboBox::currentTextChanged, this, &main_window::combo_box_changed);
    }
}

void main_window::restart_action() { restart_clicked_ = true; }
void main_window::save_bag_action() { save_bag_clicked_ = true; }
bool main_window::restart()
{
    bool temp = restart_clicked_;
    restart_clicked_ = false;
    return temp;
}
bool main_window::generate_bag()
{
    bool temp = save_bag_clicked_;
    save_bag_clicked_ = false;
    return temp;
}
