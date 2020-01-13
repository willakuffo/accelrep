#include <I2Cdev.h>
# include <Wire.h>
# include "MPU6050.h"

float conval = 1638.4; // constant to convert raw acc data into m/s^2
int16_t ax, ay, az, gx, gy, gz; //collect raw values acc as global
float ax_p, ay_p, az_p;
float real_acx, real_acy, real_acz;//stationary noise.offsets filtered with bandpass
float real_gx, real_gy, real_gz;
# define gravity 9.81
int meanX, meanY, meanZ;
float accelmin[3], accelmax[3], gyromin[3], gyromax[3];;
float angle_x, angle_y, angle_z;
float pitch_acc,roll_acc,yaw_acc,pitch_gyro,roll_gyro,yaw_gyro;

MPU6050 mpu;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(38400);
  mpu.initialize();
  mpu.setSleepEnabled(false);
  //mpu.setFullScaleAccelRange(0); // 0=> +-250deg/s ; 1=> +-500deg/s ; 2=> +-1000deg/s ; 3=> 2000deg/s
  //mpu.setFullScaleGyroRange(3);  // 0=> +-2g ; 1=> +-4g ; 2=> +-8g ; 3=> +-16g
  callibrate_noise(50,20); //required for autoFilter of acc

}

void loop() {
  // put your main code here, to run repeatedly:
  // mpu.setDLPFMode(6);//set lowpass filter
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  acc_process(6); //process acc values into ms/^2
 //=======================AUTO CONFIG================================= 
 //accel filter 
  float a[3] = {accelmin[0],accelmin[1], accelmin[2]}; //min  values for bandpass
  float b[3] = {accelmax[0],accelmax[1],accelmax[2]}; //max amplitudes for bandmass
  //gyro filter
  float c[3] = {gyromin[0],gyromin[1], gyromin[2]}; //min  values for bandpass
  float d[3] = {gyromax[0],gyromax[1],gyromax[2]}; //max amplitudes for bandmass
  //======================MANUAL CONFIG===========================
  //float a[3] = { -1, -1, 9.79}; //min  values for bandpass
  //float b[3] = {1, 1, 9.81}; //max amplitudes for bandmass
  accFilter(a, b, 20); //manual bandpass filter
  //float c[3] = { -160, -90, 140}; //min  values for bandpass
  //float d[3] = { -150, -80, 150}; //max amplitudes for bandmass
  gyroFilter(c, d, 20);
 // ================================================================

 //gyro
 //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Serial.print(real_acx); Serial.print("  ");
  Serial.print(real_acy); Serial.print("  ");
  Serial.print(real_acz);Serial.print(" ");
  //accel
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  Serial.print(real_gx); Serial.print("  ");
     Serial.print(real_gy); Serial.print("  ");
     Serial.println(real_gz);
  //gyro_orientation(20);

}

void acc_process(int filter_mode) {
  //process and convert to ms/^2
  mpu.setDLPFMode(filter_mode);//low pass filter mode
  // map and process raw values to gravity
  //normalization values are affected by fullscale reading
  ax_p = ax / 1573.90;
  ay_p = ay / 1662.18;
  az_p = -(az / 2052.11); // reason for -ve is that my mpu is upside down
}
void callibrate_noise(int dataSize ,int spd) {
  //function to remove stationary noise or offsets
  // accelerometer noise collection
  float noise_Acx[dataSize];
  float noise_Acy[dataSize];
  float noise_Acz[dataSize];
  float noise_Gx[dataSize];
  float noise_Gy[dataSize];
  float noise_Gz[dataSize];

  for (int i = 0; i < dataSize; i = i + 1) {
    //collect noise
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    acc_process(6);
    noise_Acx[i] = ax_p;
    noise_Acy[i] = ay_p;
    noise_Acz[i] = az_p;
    noise_Gx[i] = gx;
    noise_Gy[i] = gy;
    noise_Gz[i] = gz;
    delay(spd);// callibration speed /sampling rate
    Serial.print("Accel: "); Serial.print(ax_p); Serial.print("  ");
    Serial.print(ay_p); Serial.print("  ");
    Serial.print(az_p); Serial.print("  GYRO: ");
    Serial.print(gx); Serial.print("  ");
    Serial.print(gy); Serial.print("  ");
    Serial.print(gz); Serial.print(" #");
    if (i == dataSize - 1) {
      Serial.println("\nACCELEROMETERS NOISE CALIBRATION SUCCESSFUL");
      continue;
    }
    else if (i < dataSize) {
      Serial.print(i);
      Serial.println("  => Callibrating IMU Noise");
    }
    // for finding minimum values --algorithm: - if im greater than you, make me you else make me me (dont change me)
    //'' '''''''''maximum'''''''''''''''''' -      if im less '''''''''''''''''''''''''''''''''''''''''''''''''''''''
    // itereation to find accelerometer max noise values
    /*if (acxmax < noise_Acx[i]) {
      acxmax = noise_Acx[i];
      } else {
      acxmax = acxmax;
      }
      if (acymax < noise_Acy[i]) {
      acymax = noise_Acy[i];
      } else {
      acymax = acymax;
      }
      if (aczmax < noise_Acz[i]) {
      aczmax = noise_Acz[i];
      } else {
      aczmax = aczmax;
      }
      // min noise values for accelerometer
      if (acxmin > noise_Acx[i]) {
      acxmin = noise_Acx[i];
      } else {
      acxmin = acxmin;
      }
      if (acymin > noise_Acy[i]) {
      acymin = noise_Acy[i];
      } else {
      acymin = acymin;
      }
      if (aczmin < noise_Acz[i]) {
      aczmin = noise_Acz[i];
      } else {
      aczmin = aczmin;
      }
      //gyro
      // itereation to find gyro max noise values
      if (gxmax < noise_Gx[i]) {
      gxmax = noise_Gx[i];
      } else {
      gxmax = gxmax;
      }
      if (gymax < noise_Gy[i]) {
      gymax = noise_Gy[i];
      } else {
      gymax = gymax;
      }
      if (gzmax < noise_Gz[i]) {
      gzmax = noise_Gz[i];
      } else {
      gzmax = gzmax;
      }
      // min noise values for gyro
      if (gxmin > noise_Gx[i]) {
      gxmin = noise_Gx[i];
      } else {
      gxmin = gxmin;
      }
      if (gymin > noise_Gy[i]) {
      gymin = noise_Gy[i];
      } else {
      gymin = gymin;
      }
      if (gzmin < noise_Gz[i]) {
      gzmin = noise_Gz[i];
      } else {
      gzmin = gzmin;
      }*/
  }
  float acxmax = noise_Acx[0], acymax = noise_Acy[0], aczmax = noise_Acz[0], acxmin = noise_Acx[0], acymin = noise_Acy[0], aczmin = noise_Acz[0];
  float gxmax = noise_Gx[0], gymax = noise_Gy[0], gzmax = noise_Gz[0], gxmin = noise_Gx[0], gymin = noise_Gy[0], gzmin = noise_Gz[0];
  for (int i = 0; i < dataSize - 1; i = i + 1) {
    Serial.println("SETTING IMU NOISE THRESHOLDS");
    acxmax = max(noise_Acx[i], acxmax); acxmin = min(noise_Acx[i], acxmin);
    acymax = max(noise_Acy[i], acymax); acymin = min(noise_Acy[i], acymin);
    aczmax = max(noise_Acz[i], aczmax); aczmin = min(noise_Acz[i], aczmin);
    gxmax = max(noise_Gx[i], gxmax); gxmin = min(noise_Gx[i], gxmin);
    gymax = max(noise_Gy[i], gymax); gymin = min(noise_Gy[i], gymin);
    gzmax = max(noise_Gz[i], gzmax); gzmin = min(noise_Gz[i], gzmin);
  }
  // put maximum and minimum noise values into callibrated arrays
  accelmax[0] = acxmax; accelmax[1] = acymax; accelmax[2] = aczmax;
  accelmin[0] = acxmin; accelmin[1] = acymin; accelmin[2] = aczmin;
  Serial.println("Accelerometer Thresholds");
  Serial.print(acxmax); Serial.print("  ");
  Serial.print(acymax); Serial.print("  ");
  Serial.println(aczmax);
  Serial.print(acxmin); Serial.print("  ");
  Serial.print(acymin); Serial.print("  ");
  Serial.println(aczmin);
  gyromax[0] = gxmax; gyromax[1] = gymax; gyromax[2] = gzmax;
  gyromin[0] = gxmin; gyromin[1] = gymin; gyromin[2] = gzmin;
  Serial.println("GYroscope Thresholds");
  Serial.print(gxmax); Serial.print("  ");
  Serial.print(gymax); Serial.print("  ");
  Serial.println(gzmax);
  Serial.print(gxmin); Serial.print("  ");
  Serial.print(gymin); Serial.print("  ");
  Serial.println(gzmin);
  delay(5000); //debugging delay
}



void accFilter(float accel_min[3], float accel_max[3], int spd) {
  //manual mode of accFilter_auto, rather specify your own min and maximum noise bands
  //filters stationary noise acts as a bandpass filter
  //no need to call callibrate_noise function for auto callibration. That is for auto acc
  if (ax_p >= accel_min[0] && ax_p <= accel_max[0]) { // if x  axis accel is within noise range
    real_acx = 0;
  } else {
    real_acx = ax_p;
  }

  if (ay_p <= accel_max[1] && ay_p >= accel_min[1]) { // if axis accel is within noise range
    real_acy = 0;
  } else {
    real_acy = ay_p;
  }

  if (az_p <= accel_max[2] && az_p >= accel_min[2]) { // if z axis accel is within noise range
    real_acz = 9.81;
  }
  else {
    real_acz = abs(az_p);  // particularly for z to account for gravity
  }
  delay(spd); //return rate
}



void gyroFilter(float gyro_min[3], float gyro_max[3], int spd) {
  //manual mode of gyroFilter_auto, rather specify your own min and maximum noise bands
  //filters stationary noise acts as a bandpass filter
  //no need to call callibrate_noise function for auto callibration. That is for auto acc
  if (gx >= gyro_min[0] && gx <= gyro_max[0]) { // if x  axis accel is within noise range
    real_gx = 0;
  } else {
    real_gx = gx;
  }

  if (gy <= gyro_max[1] && gy >= gyro_min[1]) { // if axis accel is within noise range
    real_gy = 0;
  } else {
    real_gy = gy;
  }

  if (gz <= gyro_max[2] && gz >= gyro_min[2]) { // if z axis accel is within noise range
    real_gz = 0;
  }
  else {
    real_gz = gz;  // particularly for z to account for gravity
  }
  delay(spd); //return rate
}

void acc_orientation() {
pitch_acc = asin(ax_p/9.81)*RAD_TO_DEG; roll_acc = asin(ay_p/9.81)*RAD_TO_DEG; yaw_acc = asin(az_p/9.81)*RAD_TO_DEG;
Serial.print(pitch_acc);Serial.print(" ");Serial.print(roll_acc);Serial.print(" ");
Serial.println(yaw_acc);
}
void gyro_orientation(int sample_rate) {
pitch_gyro = pitch_gyro+(gx*sample_rate);
roll_gyro = roll_gyro+(gx*sample_rate);
yaw_gyro = yaw_gyro+(gx*sample_rate);
Serial.print("                           ");
Serial.print(pitch_gyro);Serial.print(" ");Serial.print(roll_gyro);Serial.print(" ");
Serial.println(yaw_gyro);
delay(sample_rate);
}
