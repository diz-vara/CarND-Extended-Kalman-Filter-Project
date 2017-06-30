#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

 
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
    0, 1, 0, 0;
              
  //create a 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = VectorXd(4);

  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;


  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;

  ekf_.Q_ = MatrixXd(4, 4);
  //set the acceleration noise components
  //noise_ax = 5;
  //noise_ay = 5;
 


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {




  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    double px(0);
    double py(0);
    double vx(5);
    double vy(0);
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      //std::cout << "RADAR" << std::endl;
      double rho = measurement_pack.raw_measurements_[0];
      double phi = measurement_pack.raw_measurements_[1];
      double rho_dot = measurement_pack.raw_measurements_[2];

      px = rho * cos(phi);
      py = rho * sin(phi);
      vx = rho_dot * cos(phi);
      vy = rho_dot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //std::cout << "LiDAR" << std::endl;
      /**
      Initialize state.
      */
      px = measurement_pack.raw_measurements_(0);
      py = measurement_pack.raw_measurements_(1);
    }

    ekf_.x_ = VectorXd(4);
    ekf_.x_ << px, py, vx, vy;

    //Hj_ = Tools::CalculateJacobian(ekf_.x_);

    this->previous_timestamp_ = measurement_pack.timestamp_;


    noise_ax = 9;
    noise_ay = 9;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  dt_in_microseconds = measurement_pack.timestamp_ - previous_timestamp_;
  previous_timestamp_ = measurement_pack.timestamp_;

  //convert to seconds
  double dt = (double)(dt_in_microseconds) / 1000000.;
  static double dt_old(-1);

  //std:cout << "dt=" << dt << std::endl;

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  if (dt != dt_old) {
    //recalculate matices only if dt changed
    dt_old = dt;

    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
    //2. Set the process covariance matrix Q
    double dt2 = dt * dt;
    double dt3 = dt2*dt;
    double dt4 = dt3*dt;

    ekf_.Q_ << dt4 / 4 * noise_ax, 0, dt3 / 2 * noise_ax, 0,
      0, dt4 / 4 * noise_ay, 0, dt3 / 2 * noise_ay,
      dt3 / 2 * noise_ax, 0, dt2*noise_ax, 0,
      0, dt3 / 2 * noise_ay, 0, dt2 *noise_ay;
  }

  //3. Call the Kalman Filter predict() function
  ekf_.Predict();


  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = Tools::CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }


  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
