#include <Wire.h>
// Initialization of variables outside the loop
double velocity_x = 0, velocity_y = 0, velocity_z = 0;
double displacement_x = 0, displacement_y = 0, displacement_z = 0;
double lastAccX = 0, lastAccY = 0, lastAccZ = 0;
long loopTimer, loopTimer2;
int temperature;
double accelPitch;
double accelRoll;
long acc_x, acc_y, acc_z;
double accel_x, accel_y, accel_z;
double gyroRoll, gyroPitch, gyroYaw;
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
double rotation_x, rotation_y, rotation_z;
double freq, dt;
double tau = 0.98;
double roll = 0;
double pitch = 0;

// 250 deg/s --> 131.0, 500 deg/s --> 65.5, 1000 deg/s --> 32.8, 2000 deg/s --> 16.4
long scaleFactorGyro = 65.5;

// 2g --> 16384 , 4g --> 8192 , 8g --> 4096, 16g --> 2048
long scaleFactorAccel = 8192;

void loop() {
    // Calculate the sampling frequency and delta time
    freq = 1 / ((micros() - loopTimer2) * 1e-6);
    loopTimer2 = micros();
    dt = 1 / freq;

    // Read raw data from MPU-6050
    read_mpu_6050_data();

    // Convert raw data to meaningful units (g-force)
    accel_x = (double)acc_x / scaleFactorAccel;
    accel_y = (double)acc_y / scaleFactorAccel;
    accel_z = (double)acc_z / scaleFactorAccel;

    // Subtract gravity component and calibration offset (assuming device is oriented such that z-axis measures gravity when stationary)
    accel_z -= 1.0; // Adjust according to orientation, remove if necessary

    // Simple low-pass filter to smooth accelerometer data
    accel_x = 0.9 * lastAccX + 0.1 * accel_x;
    accel_y = 0.9 * lastAccY + 0.1 * accel_y;
    accel_z = 0.9 * lastAccZ + 0.1 * accel_z;

    // Update last acceleration values
    lastAccX = accel_x;
    lastAccY = accel_y;
    lastAccZ = accel_z;

    // Basic integration for velocity and displacement
    velocity_x += accel_x * dt;
    displacement_x += velocity_x * dt;

    velocity_y += accel_y * dt;
    displacement_y += velocity_y * dt;

    velocity_z += accel_z * dt;
    displacement_z += velocity_z * dt;

    // Output data
    Serial.print("Velocity X: "); Serial.println(velocity_x);
    Serial.print("Displacement X: "); Serial.println(displacement_x);

    // Ensure fixed loop interval
    while (micros() - loopTimer <= 4000);
    loopTimer = micros();
}
