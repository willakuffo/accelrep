#include <I2Cdev.h>
# include <Wire.h>
# include "MPU6050.h"

float conval = 1638.4; // constant to convert raw acc data into m/s^2
int16_t ax, ay, az, gx, gy, gz; //collect raw values acc as global
float ax_p, ay_p, az_p;
float real_acx, real_acy, real_acz;//stationary noise.offsets filtered with bandpass
float real_gx, real_gy, real_gz;
# define gravity 9.81
int accelX_buff[100], accelY_buff[100], accelZ_buff[100];
int meanX, meanY, meanZ;
int accelmin[3], accelmax[3], gyromin[3], gyromax[3];;
float angle_x, angle_y, angle_z;

MPU6050 mpu;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(38400);
  mpu.initialize();
  mpu.setSleepEnabled(false);
  callibrate_noise(50,20); //required for autoFilter of acc

}

void loop() {
  // put your main code here, to run repeatedly:
  // mpu.setDLPFMode(6);//set lowpass filter
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  acc_process(6); //process acc values into ms/^2
  accFilter_auto(20);//for auto bandpass filter...requires callibration_noise to run first in setup to acquire thresholds automatically
  gyroFilter_auto(20);//
  //float a[3] = { -1, -1, 9.79}; //min  values for bandpass
  //float b[3] = {1, 1, 9.81}; //max amplitudes for bandmass
  //accFilter_man(a, b, 20); //manual bandpass filter
  //float c[3] = { -160, -90, 140}; //min  values for bandpass
  //float d[3] = { -150, -80, 150}; //max amplitudes for bandmass
  //gyroFilter_man(c, d, 20);
  Serial.print(real_gx); Serial.print("  ");
  Serial.print(real_gy); Serial.print("  ");
  Serial.println(real_gz);
  /* Serial.print(real_acx); Serial.print("  ");
     Serial.print(real_acy); Serial.print("  ");
     Serial.println(real_acz);*/
  //orientation();
  //delay(200);
}

void acc_process(int filter_mode) {
  //process and convert to ms/^2
  mpu.setDLPFMode(filter_mode);//low pass filter mode 0
  // map and process raw values to gravity
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
  delay(30000); //debugging delay
}

void accFilter_auto(int spd) {
  //filters stationary noise acts as a bandpass filter

  if (ax_p >= accelmin[0] && ax_p <= accelmax[0]) { // if x  axis accel is within noise range
    real_acx = 0;
  } else {
    real_acx = ax_p;
  }

  if (ay_p <= accelmax[1] && ay_p >= accelmin[1]) { // if axis accel is within noise range
    real_acy = 0;
  } else {
    real_acy = ay_p;
  }

  if (az_p <= accelmax[2] && az_p >= accelmin[2]) { // if z axis accel is within noise range
    real_acz = 9.81;
  }
  else {
    real_acz = abs(az_p);  // particularly for z to account for gravity
  }
  delay(spd); //return rate
}
void accFilter_man(float accel_min[3], float accel_max[3], int spd) {
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
void gyroFilter_auto(int spd) {
  //filters stationary noise acts as a bandpass filter

  if (gx >= gyromin[0] && gx <= gyromax[0]) { // if x  axis accel is within noise range
    real_gx = 0;
  } else {
    real_gx = gx;
  }

  if (gy <= gyromax[1] && gy >= gyromin[1]) { // if axis accel is within noise range
    real_gy = 0;
  } else {
    real_gy = gy;
  }

  if (az_p <= gyromax[2] && az_p >= gyromin[2]) { // if z axis accel is within noise range
    real_acz = 0;
  }
  else {
    real_gz = gz;  // particularly for z to account for gravity
  }
  delay(spd); //return rate
}
void gyroFilter_man(float gyro_min[3], float gyro_max[3], int spd) {
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

void orientation(int Ts) {
  float angle_x, angle_y, angle_z;
  angle_x = angle_x + gx * Ts;
  angle_y = angle_y + gy * Ts;
  angle_z = angle_z + gz * Ts;
  Serial.print(gx); Serial.print("  ");
  Serial.print(gy); Serial.print("  ");
  Serial.println(gz);

  //delay(Ts);//sampling rate

}
