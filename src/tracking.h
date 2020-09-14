#ifndef TRACKING_H
#define TRACKING_H

#include "measurementPackage.h"
#include "kalmanFilter.h"
#include <fstream>
#include <iostream>


class tracking{
    private:

    public:
        std::ofstream outputFile;
        bool first_measurement;
        long previous_time_stamp;
        float dt;
        std::vector<measurementPackage> package_list;

        kalmanFilter kf;
        float noise_ax, noise_ay;

        tracking();
        ~tracking();
        void process_measurements(measurementPackage* mp);
};

#endif