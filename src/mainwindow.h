#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStandardItemModel>
#include <QMessageBox>
#include <QStandardItem>

#include "data.h"
#include "plot.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    Data *pData;
    Plot *pPlot;

    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_data_time_valueChanged(int arg1);
    void on_data_freq_valueChanged(int arg1);
    void on_btn_sim_released();

private:
    Ui::MainWindow *ui;
    QStandardItemModel *tab_mod_acc;
    QStandardItemModel *tab_mod_gyro;

    size_t _size;

    void init_param_table();

public:
    double _ab[3];
    double _aw[3];
    double _ai[3];
    double _at[3];

    double _gb[3];
    double _gw[3];
    double _gi[3];
    double _gt[3];
};

#endif // MAINWINDOW_H
