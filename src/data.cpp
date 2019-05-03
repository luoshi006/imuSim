#include "data.h"

#include "QIODevice"
#include <QTextStream>

#include <iostream>

using std::cout;
using std::endl;

Data::Data(Ui::MainWindow *parent)
    : ui(parent)
    , period(0)
    , freq(0)
{
    qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));
}

void
Data::setData(QString lhs)
{// generate imu data and write to csv file.
    fileName = lhs;
    QFile csvFile(fileName);
    if (!csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        cout<<"[err] cannot open file for writing: "<<qPrintable(csvFile.errorString())<<endl;
        return;
    }

    QTextStream csv(&csvFile);
//        csv << "test"<< "\n";
    csv << "#timestamp, accX, accY, accZ, gyroX, gyroY, gyroZ"<<endl;
    size_t num = freq * period;
    size_t timeStamp = 0;
    double dt = 1./freq;

    double sigmaAcc[3], sigmaGyro[3];
    for (size_t i=0; i<3; ++i) {
        sigmaAcc[i] = sqrt(dt/_at[i])*_ai[i];
        sigmaGyro[i]= sqrt(dt/_gt[i])*_gi[i];
    }
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> noise_ax(0.0, sigmaAcc[0]);
    std::normal_distribution<double> noise_ay(0.0, sigmaAcc[1]);
    std::normal_distribution<double> noise_az(0.0, sigmaAcc[2]);
    std::normal_distribution<double> noise_gx(0.0, sigmaGyro[0]);
    std::normal_distribution<double> noise_gy(0.0, sigmaGyro[1]);
    std::normal_distribution<double> noise_gz(0.0, sigmaGyro[2]);

//    normrnd(0,gwx/sqrt(dt))
    std::normal_distribution<double> noise_awx(0.0, _aw[0]);
    std::normal_distribution<double> noise_awy(0.0, _aw[1]);
    std::normal_distribution<double> noise_awz(0.0, _aw[2]);
    std::normal_distribution<double> noise_gwx(0.0, _gw[0]);
    std::normal_distribution<double> noise_gwy(0.0, _gw[1]);
    std::normal_distribution<double> noise_gwz(0.0, _gw[2]);
//    distribution(generator);

    for(size_t i = 0; i < num; ++i) {
        timeStamp += 1000.0*dt;

        // for now just fit static state
        double _data_true[DataNum] = {0};
        double _data_noise[DataNum]= {0};

        _ag[0] += noise_ax(generator);
        _ag[1] += noise_ay(generator);
        _ag[2] += noise_az(generator);
        _gg[0] += noise_gx(generator);
        _gg[1] += noise_gy(generator);
        _gg[2] += noise_gz(generator);

//        gyrox_n(i) = gyrox(i) + gbx + normrnd(0,gwx/sqrt(dt)) + Rgx(i);
        // generate data here.
        _data_noise[0] = _data_true[0] + _ab[0] + noise_awx(generator) + _ag[0];
        _data_noise[1] = _data_true[1] + _ab[1] + noise_awy(generator) + _ag[1];
        _data_noise[2] = _data_true[2] + _ab[2] + noise_awz(generator) + _ag[2] - 9.80665;
        _data_noise[3] = _data_true[3] + _gb[0] + noise_gwx(generator) + _gg[0];
        _data_noise[4] = _data_true[4] + _gb[1] + noise_gwy(generator) + _gg[1];
        _data_noise[5] = _data_true[5] + _gb[2] + noise_gwz(generator) + _gg[2];

        csv<<timeStamp<<", ";
        for (size_t j=0; j<DataNum; ++j) {
            csv<< _data_noise[j]<<",";
        }
        csv<<endl;
    }

    csvFile.close();
    ui->statusBar->showMessage("Data saved.",3000);
}
