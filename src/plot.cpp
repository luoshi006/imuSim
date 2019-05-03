#include "plot.h"

#include <QIODevice>
#include <QTextStream>

#include <iostream>

using std::cout;
using std::endl;

Plot::Plot(Ui::MainWindow *parent)
    : ui(parent)
{
    //sth todo.
//    seriesVec.reserve(DataNum);
//    seriesVec.resize(DataNum);
}

void
Plot::setData(QString lhs)
{
    // reset state and clean data
    for (size_t i=0; i<DataNum-1; ++i) {
        seriesVec[i].clear();
    }

    fileName = lhs;
    QFile csvFile(fileName);

    if (!csvFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        ui->statusBar->showMessage("Get Data failed.",3000);
        return;
    }

    QTextStream stream(&csvFile);
    while (!stream.atEnd()) {
        QString line = stream.readLine();
        if (line.startsWith("#"))
            continue;
        line = line.simplified();
        if (line.endsWith(","))
            line.chop(1);


        QStringList values = line.split(",", QString::SkipEmptyParts);

        if (values.size() != DataNum) {
            ui->statusBar->showMessage("Data col mismatch!", 3000);
            return;
        }

        const double timeStamp = 0.001*values[Timestamp].toInt();
        for (int i=0; i<DataNum-1; ++i) {   // with no timestamp
            seriesVec[i].append(timeStamp, values[i+1].toDouble());
        }
    }
    csvFile.close();

    ui->statusBar->showMessage("Get Data for Plot.",3000);
}

void
Plot::plotData()
{
    QList<QChartView *> m_charts;

    for (size_t i=0; i<1; ++i) {
        QChart *chart = new QChart();
        chart->addSeries(&seriesVec[i]);
        chart->legend()->hide();

        QValueAxis *axisX = new QValueAxis;
        axisX->setLabelFormat("%f");
        axisX->setTitleText("Time");
        chart->addAxis(axisX, Qt::AlignBottom);
        seriesVec[i].attachAxis(axisX);

        QValueAxis *axisY = new QValueAxis;
        axisY->setLabelFormat("%f");
        switch (i) {
            case 0:
                axisY->setTitleText("Acc X"); break;
            case 1:
                axisY->setTitleText("Acc Y"); break;
            case 2:
                axisY->setTitleText("Acc Z"); break;
            default:
                break;
        }
        chart->addAxis(axisY, Qt::AlignLeft);
        seriesVec[i].attachAxis(axisY);

        QChartView *chartView = new QChartView(chart);
        chartView->setRenderHint(QPainter::Antialiasing);

        ui->gridLayout_plot->addWidget(chartView,i,0);
        m_charts << chartView;
    }
    ui->tabWidget->setCurrentIndex(1);
}
