![GitHub Logo](/images/screenshot.png)
Format: ![Alt Text](url)

# Extended Kalman Filter

This project utilizes a kalman filter to estimate the state of a moving object with nosy LIDAR and RADAR measurement.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

## Installation
Clone the GitHub repository and run in the master directory

* Step 1: `$ git clone`
* Step 2: `$ cmake`
* Step 3: `$ make`
* Step 4: `$ ./ExtendedKF`
-- will return 
Listening to port 4567
Connected!!!
* Step 5: 

## Rubric 1 Compiling
### Criteria: Your code should compile.
Code must compile without errors with `cmake` and `make`.

## Rubric 2 Accuracy
### Criteria: Meets Specifications
Your px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52]

## Rubric 3 Follows the Correct Algorithm
### Criteria 3.1: Meets Specifications
Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

### Algorithem Flow

1. Initialize Covariance and State Matrix
2. Prediction Step
3. Measurement Update Step (y = z - Hx')
4. Calculate New Object State (x = x' + ky)

* Set up Laser Matrixes
* Set up Radar Matrixes (convert space: calculate Jacobian Matrix using rho, phi, rho_dot)

### Criteria 3.2: Initialize State and Covariance Matrices
#### first measurement
```
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);

      ekf_.x_(0) = ro * cos(phi);
      ekf_.x_(1) = ro * sin(phi);
      ekf_.x_(2) = ro_dot * cos(phi);
      ekf_.x_(3) = ro_dot * sin(phi);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
```

### Criteria 3.3: Predict Object Position to Current timestep and update prediction

#### compute time delta w/ dt in seconds
```
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
   previous_timestamp_ = measurement_pack.timestamp_;

   float dt_2 = dt * dt;
   float dt_3 = dt_2 * dt;
   float dt_4 = dt_3 * dt;
```
#### integrate time into F matrix
```  
   ekf_.F_(0,2) = dt;
   ekf_.F_(1,3) = dt;
```
#### set acceleration noise
```  
   float noise_ax = 9;
   float noise_ay = 9;
```
#### set Q, process covariance variance matrix
```  
   ekf_.Q_ = MatrixXd(4,4);
   ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
              0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
              dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
              0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;


  ekf_.Predict();
```
### Criteria 3.4: Set up the appropriate matrices given the type of measurement and calls correct measurement function for a given sensor type
```
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    
    // Radar updates
    Tools tools; 
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
```

## Rubric 3: Code Efficiency
### Criteria: Your algorithm should avoid unnecessary calculations
Done
