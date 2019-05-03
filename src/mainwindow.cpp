#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->data_time->setValue(1);
    ui->data_freq->setValue(10);

    init_param_table();

    setWindowTitle("SimUnit");

    pData = new Data(ui);
    pPlot = new Plot(ui);

    pData->setPeriod(ui->data_time->value());
    pData->setFreq(ui->data_freq->value());
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::init_param_table()
{
    // init the param tabel of acc
    tab_mod_acc = new QStandardItemModel(3,4);
    ui->tab_acc_param->setModel(tab_mod_acc);
    ui->tab_acc_param->horizontalHeader()->setStretchLastSection(true);
    ui->tab_acc_param->verticalHeader()->setStretchLastSection(true);
    QStringList col, row;
    col << "bias rad/s" << "ARW rad/sqrt(s)" << "BI rad/s" << "Markov Time";
    row << "accX" << "accY" << "accZ";
    tab_mod_acc->setHorizontalHeaderLabels(col);
    tab_mod_acc->setVerticalHeaderLabels(row);
//    tab_mod_acc->setItem(1,1,new QStandardItem("qwe"));

    // init the param tabel of gyro
    tab_mod_gyro = new QStandardItemModel(3,4);
    ui->tab_gyro_param->setModel(tab_mod_gyro);
    ui->tab_gyro_param->horizontalHeader()->setStretchLastSection(true);
    ui->tab_gyro_param->verticalHeader()->setStretchLastSection(true);
    row.clear();
    row << "gyroX" << "gyroY" << "gyroZ";
    col.clear();
    col << "bias m/s^2" << "VRW m/s^2/sqrt(s)" << "BI m/s^2" << "Markov Time";
    tab_mod_gyro->setHorizontalHeaderLabels(col);
    tab_mod_gyro->setVerticalHeaderLabels(row);

    for (int i=0; i<tab_mod_acc->columnCount(); ++i) {
        double Tmp[4] = {0.0, 0.075, 0.1, 10};
        for (int j=0; j<tab_mod_acc->rowCount(); ++j) {
            tab_mod_acc->setItem(j,i,new QStandardItem(QString::number(Tmp[i])));
        }
    }

    for (int i=0; i<tab_mod_gyro->columnCount(); ++i) {
        double Tmp[4] = {0.0, 0.001, 0.0003, 10};
        for (int j=0; j<tab_mod_gyro->rowCount(); ++j) {
            tab_mod_gyro->setItem(j,i,new QStandardItem(QString::number(Tmp[i])));
        }
    }

    ui->tab_gyro_param->resizeColumnsToContents();
    ui->tab_gyro_param->resizeRowsToContents();
    ui->tab_acc_param->resizeColumnsToContents();
    ui->tab_acc_param->resizeRowsToContents();
}

void MainWindow::on_data_time_valueChanged(int arg1)
{
    if (arg1 < 1) {
        QMessageBox::warning(this, tr("warn"), tr("Please set Time larger than 1s."));
        return;
    }
    pData->setPeriod(ui->data_time->value());
}

void MainWindow::on_data_freq_valueChanged(int arg1)
{
    if (arg1 < 1) {
        QMessageBox::warning(this, tr("warn"), tr("Please set Freq valid data."));
        return;
    }
    pData->setFreq(ui->data_freq->value());
}

void MainWindow::on_btn_sim_released()
{
    // set the imu noise param
     _ab[0] = tab_mod_acc->item(0,0)->text().toDouble();
     _ab[1] = tab_mod_acc->item(1,0)->text().toDouble();
     _ab[2] = tab_mod_acc->item(2,0)->text().toDouble();
     _aw[0] = tab_mod_acc->item(0,1)->text().toDouble();
     _aw[1] = tab_mod_acc->item(1,1)->text().toDouble();
     _aw[2] = tab_mod_acc->item(2,1)->text().toDouble();
     _ai[0] = tab_mod_acc->item(0,2)->text().toDouble();
     _ai[1] = tab_mod_acc->item(1,2)->text().toDouble();
     _ai[2] = tab_mod_acc->item(2,2)->text().toDouble();
     _at[0] = tab_mod_acc->item(0,3)->text().toDouble();
     _at[1] = tab_mod_acc->item(1,3)->text().toDouble();
     _at[2] = tab_mod_acc->item(2,3)->text().toDouble();
    pData->setAccParam(_ab, _aw, _ai, _at);

     _gb[0] = tab_mod_gyro->item(0,0)->text().toDouble();
     _gb[1] = tab_mod_gyro->item(1,0)->text().toDouble();
     _gb[2] = tab_mod_gyro->item(2,0)->text().toDouble();
     _gw[0] = tab_mod_gyro->item(0,1)->text().toDouble();
     _gw[1] = tab_mod_gyro->item(1,1)->text().toDouble();
     _gw[2] = tab_mod_gyro->item(2,1)->text().toDouble();
     _gi[0] = tab_mod_gyro->item(0,2)->text().toDouble();
     _gi[1] = tab_mod_gyro->item(1,2)->text().toDouble();
     _gi[2] = tab_mod_gyro->item(2,2)->text().toDouble();
     _gt[0] = tab_mod_gyro->item(0,3)->text().toDouble();
     _gt[1] = tab_mod_gyro->item(1,3)->text().toDouble();
     _gt[2] = tab_mod_gyro->item(2,3)->text().toDouble();
    pData->setGyroParam(_gb, _gw, _gi, _gt);

    QString csvPath("/tmp/imuSim.csv");
    pData->setData(csvPath);

    pPlot->setData(csvPath);
    pPlot->plotData();
}
