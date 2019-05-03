#ifndef PLOT_H
#define PLOT_H

#include "ui_mainwindow.h"

#include <QString>
#include <QFile>
#include <QMessageBox>
#include <QTime>
#include <QtGlobal>
#include <QVector>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QXYSeries>

QT_CHARTS_USE_NAMESPACE

class Plot
{
public:
    Plot(Ui::MainWindow *parent);
    void setData(QString lhs);
    void plotData();

    enum DataEnum
    {
        Timestamp = 0,
        AccX,
        AccY,
        AccZ,
        GyroX,
        GyroY,
        GyroZ,
        DataNum
    };

private:
    Ui::MainWindow *ui;

    QString fileName;

public:
    QLineSeries seriesVec[DataNum-1];
};

#endif // PLOT_H
