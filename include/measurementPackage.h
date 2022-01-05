#ifndef MEASUREMENT_PACKAGE_H
#define MEASUREMENT_PACKAGE_H

#include <eigen3/Eigen/Dense>

class measurementPackage
{
    private:
        
    public:
        Eigen::VectorXd raw_measurements;
        // timestamp in microseconds
        long timestamp;
        enum sensor {
            LASER,
            RADAR
        };
        enum sensor sensor_type;
        measurementPackage();
        ~measurementPackage();
};

#endif