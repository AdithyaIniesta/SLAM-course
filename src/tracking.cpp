#include "tracking.h"

tracking::tracking(){
    
    outputFile.open("/home/adithyainiesta/Udacity/EKF/output.csv");
    outputFile << "x1, y1, x2, y2, x3, y3\n";
    kf.x = Eigen::MatrixXd(4,1);
    
    kf.H = Eigen::MatrixXd(2,4);
    kf.H << 1,0,0,0, 
            0,1,0,0;
    
    kf.R_laser = Eigen::MatrixXd(2,2);
    kf.R_laser << 0.0225, 0,
                0, 0.0225;

    kf.R_radar = Eigen::MatrixXd(3,3);
    kf.R_radar << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    kf.F = Eigen::MatrixXd(4,4);
    kf.Q = Eigen::MatrixXd(4,4);

    kf.P = Eigen::MatrixXd(4,4);
    kf.P << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;
            
    noise_ax = 9;
    noise_ay = 9;
}

tracking::~tracking(){
}

void tracking::process_measurements(measurementPackage* mp){

    if (first_measurement){ 
        if(mp->sensor_type == measurementPackage::RADAR){

            double rho = mp->raw_measurements[0];
            double phi = mp->raw_measurements[1];
            double rho_dot = mp->raw_measurements[2];

            double px = rho * cos(phi);

            if (px < 0.0001){
                px = 0.0001;
            }
            
            double py = rho * sin(phi);

            if(py < 0.0001){
                py = 0.0001;
            }
            
            kf.x << px, 
                    py, 
                    rho_dot * cos(phi), 
                    rho_dot * sin(phi);
        }   

        else if(mp->sensor_type == measurementPackage::LASER){

            kf.x << mp->raw_measurements[0], 
                    mp->raw_measurements[1],
                    0,
                    0;
        }

        previous_time_stamp = mp->timestamp;
        first_measurement = !first_measurement;
    }
    else{
        
        dt = (mp->timestamp - previous_time_stamp) / 1000000.0;
        previous_time_stamp = mp->timestamp;

        kf. F << 1, 0, dt, 0,
                0, 1, 0, dt,
                0, 0, 1, 0,
                0, 0, 0, 1;

        double dt_2 = pow(dt, 2);
        double dt_3 = dt_2 * dt;
        double dt_4 = dt_3 * dt;

        kf.Q << 0.25 * dt_4 * noise_ax, 0, 0.5 * dt_3 * noise_ax, 0, 
                0, 0.25 * dt_4 * noise_ay, 0, 0.5 * dt_3 * noise_ay, 
                0.5 * dt_3 * noise_ax, 0, dt_2 * noise_ax, 0,
                0, 0.5 * dt_3 * noise_ay, 0, dt_2 * noise_ay; 


        kf.predict();

        if (mp->sensor_type == measurementPackage::LASER){
            kf.Z = Eigen::MatrixXd(2,1);
            kf.Z << mp->raw_measurements[0],
                    mp->raw_measurements[1];
            kf.update();
        }
        else if(mp->sensor_type == measurementPackage::RADAR){
            kf.Z = Eigen::MatrixXd(3,1);
            kf.Z << mp->raw_measurements[0], 
                    mp->raw_measurements[1], 
                    mp->raw_measurements[2];
            kf.updateEKF();
        }
        
        Eigen::MatrixXd m = kf.printMean();
        outputFile << m(0) << "," << m(1) << "\n";
    }
}
