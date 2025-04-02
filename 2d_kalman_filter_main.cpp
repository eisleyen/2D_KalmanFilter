#include <iostream>
#include <vector>
#include <Eigen>
#include <ctime>

class KalmanFilter2D {
	public:
	  KalmanFilter2D(double dt) : dt_(dt) {
      // initializing a state vector [position, velocity]
      state_ << 0.0, 0.0;
  
      // state transition matrix (A)
      A_ << 1, dt_,
            0, 1;
  
      // control matrix (B) -- control input is acceleration
      B_ << 0.5 * dt_ * dt_,
            dt_;
  
      // measurement matrix (H) -- position and velocity measured
      H_ << 1, 0,
      0, 1;
  
      // covariance matrices
      P_ = Eigen::Matrix2d::Identity() * 1.0; // initial uncertainty
      Q_ = Eigen::Matrix2d::Identity() * 0.1; // process noise
      R_ << 2.0, 0,
        0, 0.5; // measurement noise - assuming wheel speed sensor more accurate than gps
	  }
	    
    void update (double pos_measurement, double vel_measurement, double acceleration) {
      state_ = A_ * state_ + B_ * acceleration; // predicting new state
      P_ = A_ * P_ * A_.transpose() + Q_; // predicting new uncertainty
    
      Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_; //innovation covariance
      Eigen::Matrix2d K = P_ * H_.transpose() * S.inverse(); // kalman gain
    
    
      Eigen::Vector2d z;
      z << pos_measurement,
           vel_measurement;

      state_ = state_ + K * (z - H_ * state_);
      P_ = (Eigen::Matrix2d::Identity() - K * H_) * P_;
    }
		
    Eigen::Vector2d getState() {
        return state_;
    }
		
private:
  double dt_; // time step
  Eigen::Vector2d state_;  // [position, velocity]
  Eigen::Matrix2d A_, P_, Q_, R_;  // State transition, covariance, process noise, measurement noise
  Eigen::Vector2d B_;  // control matrix - acceleration
  Eigen::Matrix<double, 2, 2> H_;  // Measurement matrix
};

int main() {
	srand(time(0)); //random num generator
	KalmanFilter2D k_f(0.1); //time step = 0.1s
	
	double true_position = 0.0;
  double true_velocity = 5.0;
  double acceleration = 0.2;

	for (int i = 0; i < 50; i++) {
		// dynamic acceleration: Combination of sinusoidal and random noise
		double acceleration = 0.5 * sin(i*0.2) + ((rand() % 100) / 100.0 - 0.5) * 0.2;
		double position_measurement = true_position + ((rand() % 100) / 100.0 - 0.5) * 2.0;  // GPS
    double velocity_measurement = true_velocity + ((rand() % 100) / 100.0 - 0.5) * 0.5;  // Wheel sensor
        
    k_f.update(position_measurement, velocity_measurement, acceleration);
    Eigen::Vector2d estimated_state = k_f.getState();
        
    std::cout << "Step " << i 
              << " | Acceleration: " << acceleration 
              << " | Position: " << estimated_state(0) 
              << " | Velocity: " << estimated_state(1) << std::endl;
                  
    true_position += true_velocity * 0.1 + 0.5*acceleration*0.1*0.1;
    true_velocity += acceleration * 0.1;
}
	
return 0;
}
