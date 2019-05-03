#ifndef DATA_H
#define DATA_H

#include "ui_mainwindow.h"

#include <QString>
#include <QFile>
#include <QMessageBox>
#include <QTime>
#include <QtGlobal>

#include <random>

class Data
{
public:
    Data(Ui::MainWindow *parent);

    enum DataFlagEnum
    {
        NoiseAccX = 0,
        NoiseAccY,
        NoiseAccZ,
        NoiseGyroX,
        NoiseGyroY,
        NoiseGyroZ,
        DataNum
    };
    double g = 9.80665;
    void setData(QString lhs);
    void setAccParam(double ab[3], double aw[3], double ai[3], double at[3])
    {   memcpy(_ab, ab, sizeof(double)*3);
        memcpy(_aw, aw, sizeof(double)*3);
        memcpy(_ai, ai, sizeof(double)*3);
        memcpy(_at, at, sizeof(double)*3);
    }
    void setGyroParam(double gb[3], double gw[3], double gi[3], double gt[3])
    {   memcpy(_gb, gb, sizeof(double)*3);
        memcpy(_gw, gw, sizeof(double)*3);
        memcpy(_gi, gi, sizeof(double)*3);
        memcpy(_gt, gt, sizeof(double)*3);
    }
    void setPeriod(const int &v) {period = v;}
    void setFreq(const int &v) {freq = v;}

private:
    Ui::MainWindow *ui;

    QString fileName;
//    QFile   csvFile;
    int period;
    int freq;

    //TODO: use bit flag instead
    bool DataFlag[DataNum] = {0};
    double  DataPerc[DataNum] = {0};
    double _ab[3];
    double _aw[3];
    double _ai[3];
    double _at[3];
    double _ag[3] = {0};  // random walk

    double _gb[3];
    double _gw[3];
    double _gi[3];
    double _gt[3];
    double _gg[3] = {0};  // random walk
};

#endif // DATA_H
