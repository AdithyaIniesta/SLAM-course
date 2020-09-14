#include <fstream>
#include <string>
#include <sstream>
#include <cmath>

#include "measurementPackage.h"
#include "tracking.h"

// set sensor marcos to "0" to diable them 
#define DISABLE_LASER 1
#define DISABLE_RADAR 1

int main(){

    // read input file 
    std::ifstream sensor_file ("/home/adithyainiesta/Udacity/EKF/obj_pose-laser-radar-synthetic-input.txt");
    if (!sensor_file.is_open()){
        std::cout << "Cannot open the input file\n";
    }

    else{
        double x, y, rho, phi, rho_dot;
        std::string line, sensor_type;
        tracking sensor_tracking;

        while (getline(sensor_file,line)){

                measurementPackage meas_package;
                std::istringstream stream_line(line);
                stream_line >>  sensor_type;
                
                // process laser measurements
                if (sensor_type.compare("L") == 0 && DISABLE_LASER){   
                    meas_package.sensor_type = measurementPackage::LASER; 
                    meas_package.raw_measurements = Eigen::VectorXd(2,1);
                    stream_line >> x;
                    stream_line >> y;
                    stream_line >> meas_package.timestamp;
                    meas_package.raw_measurements << x,y;
                    sensor_tracking.package_list.push_back(meas_package);            
                }
                
                // process radar measurements
                else if (sensor_type.compare("R") == 0 && DISABLE_RADAR){
                    meas_package.sensor_type = measurementPackage::RADAR;
                    meas_package.raw_measurements = Eigen::VectorXd(3,1);
                    stream_line >> rho;
                    stream_line >> phi;
                    stream_line >> rho_dot;
                    stream_line >> meas_package.timestamp;
                    meas_package.raw_measurements << rho, phi, rho_dot;
                    sensor_tracking.package_list.push_back(meas_package);
                }
        } // finished reading all raw measurements
        
        sensor_tracking.first_measurement = true;
        size_t N = sensor_tracking.package_list.size();
        for (int i = 0; i < N; i++){
            sensor_tracking.process_measurements(&sensor_tracking.package_list[i]);
        }
    }

    if (sensor_file.is_open()){
        sensor_file.close();
    }
    
    return 0;
}







