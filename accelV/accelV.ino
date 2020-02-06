#include <I2Cdev.h>
# include <Wire.h>
# include "MPU6050.h"

float conval = 1638.4; // constant to convert raw acc data into m/s^2
int16_t ax, ay, az, gx, gy, gz; //collect raw values acc as global
float ax_p, ay_p, az_p, gx_p, gy_p, gz_p;
float real_acx, real_acy, real_acz;//stationary noise.offsets filtered with bandpass
float real_gx, real_gy, real_gz;
# define gravity 9.81
float a[3], b[3], c[3], d[3]; //thresholds
float accelmin[3], accelmax[3], gyromin[3], gyromax[3];;
float angle_x, angle_y, angle_z;
float pitch_acc, roll_acc, yaw_acc, pitch_gyro, roll_gyro, yaw_gyro;
float axis_angle,axis1_angle,axis2_angle,axis_acc;
int rotor[2];
float F_pitch,F_roll;//complimetary filter angles
float acxmax, acymax, aczmax, acxmin , acymin , aczmin;
float gxmax , gymax , gzmax, gxmin, gymin , gzmin ;

boolean CALLIBRATION_MODE_AUTO = false;//callibration mode

MPU6050 mpu;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(38400);
  mpu.initialize();
  mpu.setSleepEnabled(false);
  mpu.setFullScaleAccelRange(0); // 0=> +-250deg/s ; 1=> +-500deg/s ; 2=> +-1000deg/s ; 3=> 2000deg/s
  mpu.setFullScaleGyroRange(0);  // 0=> +-2g ; 1=> +-4g ; 2=> +-8g ; 3=> +-16g
  if (CALLIBRATION_MODE_AUTO) {
    callibrate_noise(50, 100); //required for autoFilter of acc.Callibrate for (n_samples,sample_period)
    //=======================AUTO CONFIG=================================
    //accel filter
    a[0] = acxmin; a[1] = acymin; a[2] = aczmin; //min  values for bandpass
    b[0] = acxmax; b[1] = acymax, b[2] = aczmax; //max amplitudes for bandmass
    //gyro filter
    c[0] = gxmin; c[1] = gymin; c[2] = gzmin; //min  values for bandpass
    d[0] = gxmax; d[1] = gymax; d[2] = gzmax; //max amplitudes for bandmass

  }
  else {
    Serial.println("   //======================MANUAL CONFIG===========================\n!!! Callibration mode is set to Manual!!!\n PLEASE ensure that the vaules for the filter array is provided");
    //gyro
    c[0] = 0; c[1] = 0; c[2] = -1; //min  values for bandpass
    d[0] = 0; d[1] = 0; d[2] = 1; //max amplitudes for bandmass
    //accel
    a[0] = 0.4; a[1] = -0.13; a[2] = 9.70; //min  values for bandpass
    b[0] = 0.5; b[1] = -0.09; b[2] = 9.81; //max amplitudes for bandmass
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  // mpu.setDLPFMode(6);//set lowpass filter
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  mpu.setDLPFMode(6);//low pass filter mode
  acc_process(16384.0); //process acc values into ms/^2
  gyro_process(131);
  //accFilter(a, b, 0); //manual bandpass filter
  gyroFilter(c, d, 0);
  // ================================================================

  //gyro
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 /* Serial.print(ax_p); Serial.print("  ");
    Serial.print(ay_p); Serial.print("  ");
    Serial.print(az_p); Serial.println(" ");*/
  //accel
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  Serial.print(real_gx+1); Serial.print("  ");
//    Serial.print(real_gy+1.5); Serial.print("  ");
//   Serial.println(real_gz);
  gyro_orientation(0.0196);
  acc_orientation();
  //CFilter(0.94,0.06);
}

void acc_process(float sensitivity) {
  //process and convert to ms/^2

  // map and process raw values to gravity
  //normalization values are affected by fullscale reading
  ax_p = ax / sensitivity*9.81;
  ay_p = ay / sensitivity*9.81;
  az_p = -(az / sensitivity)*9.81;
  //currently returns G-forces
  /*ax_p = ax / 1573.90;
    ay_p = ay / 1662.18;
    az_p = -(az / 2052.11); // reason for -ve is that my mpu is upside down*/
}

void gyro_process(int sensitivity) {
  gx_p = gx / sensitivity;
  gy_p = gy / sensitivity;
  gz_p = gz / sensitivity;

}
void callibrate_noise(int dataSize , int spd) {
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

  }//end of for loop that collects noise

  acxmax = noise_Acx[0], acymax = noise_Acy[0], aczmax = noise_Acz[0], acxmin = noise_Acx[0], acymin = noise_Acy[0], aczmin = noise_Acz[0];
  gxmax = noise_Gx[0], gymax = noise_Gy[0], gzmax = noise_Gz[0], gxmin = noise_Gx[0], gymin = noise_Gy[0], gzmin = noise_Gz[0];

  for (int i = 0; i < dataSize - 1; i = i + 1) {
    Serial.println("SETTING IMU NOISE THRESHOLDS");
    acxmax = max(noise_Acx[i], acxmax); acxmin = min(noise_Acx[i], acxmin);
    acymax = max(noise_Acy[i], acymax); acymin = min(noise_Acy[i], acymin);
    aczmax = max(noise_Acz[i], aczmax); aczmin = min(noise_Acz[i], aczmin);
    gxmax = max(noise_Gx[i], gxmax); gxmin = min(noise_Gx[i], gxmin);
    gymax = max(noise_Gy[i], gymax); gymin = min(noise_Gy[i], gymin);
    gzmax = max(noise_Gz[i], gzmax); gzmin = min(noise_Gz[i], gzmin);
  }

  Serial.println("Accelerometer Thresholds");
  Serial.print(acxmax); Serial.print("  ");
  Serial.print(acymax); Serial.print("  ");
  Serial.println(aczmax);
  Serial.print(acxmin); Serial.print("  ");
  Serial.print(acymin); Serial.print("  ");
  Serial.println(aczmin);

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
  if (gx_p >= gyro_min[0] && gx_p <= gyro_max[0]) { // if x  axis accel is within noise range
    real_gx = 0;
  } else {
    real_gx = gx_p;
  }

  if (gy_p <= gyro_max[1] && gy_p >= gyro_min[1]) { // if axis accel is within noise range
    real_gy = 0;
  } else {
    real_gy = gy_p;
  }

  if (gz_p <= gyro_max[2] && gz_p >= gyro_min[2]) { // if z axis accel is within noise range
    real_gz = 0;
  }
  else {
    real_gz = gz_p;  // particularly for z to account for gravity
  }
  delay(spd); //return rate
}


void acc_orientation() {

  pitch_acc = atan2(ay_p, sqrt(pow(ax_p,2)+pow(az_p,2))) * RAD_TO_DEG; roll_acc = atan2(ax_p, sqrt(pow(ay_p,2)+pow(az_p,2)))* RAD_TO_DEG; yaw_acc = asin(az_p / 9.81) * RAD_TO_DEG;
  if (isnan(yaw_acc)){
    yaw_acc = 90;
    
  /*Serial.print(pitch_acc); Serial.print(" "); Serial.print(roll_acc); Serial.print(" ");
  Serial.println(yaw_acc);*/
}
}

void gyro_orientation(double sample_rate) {
  int gyro_offset_x = -1; // x offset
  int gyro_offset_y = 0;//y
  pitch_gyro = pitch_gyro + ((real_gx-gyro_offset_x) * sample_rate);
  roll_gyro = roll_gyro + ((real_gy-gyro_offset_y) * sample_rate);
  yaw_gyro = yaw_gyro + (real_gz * sample_rate);
  // Serial.print("                           ");
  /*Serial.print(-900);  // To freeze the lower limit
  Serial.print(" ");
  Serial.print(900);  // To freeze the upper limit
  Serial.print(" ");
  Serial.print(pitch_gyro); Serial.print(" "); Serial.print(roll_gyro); Serial.print(" ");
  Serial.println(yaw_gyro);*/
}

void CFilter(float gyro_coeff,float accel_coeff){
  F_pitch = gyro_coeff*pitch_gyro+accel_coeff*pitch_acc;
  F_roll = gyro_coeff*roll_gyro+accel_coeff*roll_acc;
  Serial.print(F_pitch);Serial.print(" ");Serial.println(F_roll);
  }
