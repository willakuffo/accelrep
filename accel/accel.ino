# include "MPU6050.h"
# include <Wire.h>
# define conval 1638.4 // constant to convert raw acc data into m/s^2


int16_t ax, ay, az,gx,gy,gz;//collect raw values acc as global
int td = 0;
unsigned long ptd = millis(); //previous /initial td
float sx,sy,sz; // declare 3d distances as global variables
float fine_acx, fine_acy, fine_acz;// make processed acc global

int accelmin[3], accelmax[3],gyromin[3],gyromax[3]; // hold minimum and max noise vaues
float real_acx, real_acy, real_acz, real_gyroX, real_gyroY, real_gyroZ; // final processed and stable acc values
float travel; // distance covered in dicrete time

MPU6050 mpu; // mpu object
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600); 
mpu.initialize(); Serial.print("initializing MPU...");

if (mpu.testConnection() == true) {
	Serial.print("Device is ready");
	delay(10); 
	callibrate_acc(50); //noise callibrate accelerometers
    callibrate_gyro(50); //noise callibrate gyros
}
else { Serial.print("PLease check MPU, connection FAiLED");
delay(3000);
}
}


	//delay(1000);
void loop() {
	
	
  // put your main code here, to run repeatedly:
	//mpu.getAcceleration(&ax, &ay, &az); //  retrieve acceleration as global variables
    mpu.getRotation(&gx,&gy,&gz); // rtrieve angular acceleration as global variables
    gyroFilter();
	//mpuFilter(); //filer acc values before calculating distance
	//float a = dist(1,20); // get x acc at 20 milliseconds interval
	Serial.print(gx);Serial.print("  ");Serial.println(real_gyroX);
}

float acc_process(float acc) {
	// convert raw acc into appropriate units m/s^2
	
	float acceleration = acc / conval;
	return acceleration;

}
float dist(int accd,float dtime) {
	
	acc_collect(dtime);
	accFilter(); // filter noise out of acc and make readings more stable

	switch (accd) { //use filterd values to compute
	case 1://x
		travel = travel + (real_acx * pow(dtime/3, 2) + 0.5*real_acx*pow(dtime/3, 2));
		break;
	case 2://y
		travel = travel + (real_acy * pow(dtime/3, 2) + 0.5*real_acy*pow(dtime/3, 2));
		break;
	case 3://z
		travel = travel + (real_acz * pow(dtime/3, 2) + 0.5*real_acz*pow(dtime/3, 2));
		break;
	}
		return travel;
	
}

bool acc_collect(int discrete_interval) {
	//function to collect discrete acceleration values at specific disctrete intervals in milliseconds
	unsigned long ctd = millis(); //previous /initial td
	
	if (ctd - ptd >= discrete_interval) {
		//then collect acceleration data
		fine_acx = acc_process(ax); // appropriately procesed acc
		fine_acy = acc_process(ay);
		fine_acz = acc_process(az);
		ptd = ctd;
		return true; // when value is collected
	}
	else { return false; }
	
}
int acc_collect_on_delay(int discrete_interval) {
	//function to collect discrete acceleration values at specific disctrete intervals in milliseconds
	
	delayMicroseconds(discrete_interval);
		//then collect acceleration data
		fine_acx = acc_process(ax); // appropriately procesed acc
		fine_acy = acc_process(ay);
		fine_acz = acc_process(az);
		
		return true; // when value is collected
}
void callibrate_acc(int speed) {
# define dataSize 100
	//float noise_gyro [3][dataSize];
	//float noise_accel [3][dataSize];

	// accelerometer noise collection
	float noise_Acx[dataSize];
	float noise_Acy[dataSize];
	float noise_Acz[dataSize];

	//gyrocope noise collection
	float noise_Gyx[dataSize];
	float noise_Gyy[dataSize];
	float noise_Gyz[dataSize];

//	float gyxmax = noise_Gyx[0], gyymax = noise_Gyy[0], gyzmax = noise_Gyz[0], gyxmin = noise_Gyx[0], gyymin = noise_Gyy[0], gyzmin = noise_Gyz[0];
	float acxmax = noise_Acx[0], acymax = noise_Acy[0], aczmax = noise_Acz[0], acxmin = noise_Acx[0], acymin = noise_Acy[0], aczmin = noise_Acz[0];


	for (int i = 0; i < dataSize; i = i + 1) {

		noise_Acx[i] = fine_acx;
		noise_Acy[i] = fine_acy;
		noise_Acz[i] = fine_acz;

		delay(speed);// callibration speed


		if (i == dataSize - 1) {
			Serial.println("Data is collected");
			continue;
		}
		else if (i < dataSize) {
			Serial.println("Callibrating Accelerometers");
		}
		// for finding minimum values --algorithm: - if im greater than you, make me you else make me me (dont change me)
		//'' '''''''''maximum'''''''''''''''''' -      if im less '''''''''''''''''''''''''''''''''''''''''''''''''''''''

		if (acxmax < noise_Acx[i]) {
			acxmax = noise_Acx[i];
		}
		else { acxmax = acxmax; } 

		if (acymax < noise_Acy[i]) {
			acymax = noise_Acy[i];
		}
		else { acymax = acymax; }

		if (aczmax < noise_Acz[i]) {
			aczmax = noise_Acz[i];
		}
		else { aczmax = aczmax; }

		// min noise values for accelerometer
		if (acxmin > noise_Acx[i]) {
			acxmin = noise_Acx[i];
		}
		else { acxmin = acxmin; }

		if (acymin > noise_Acy[i]) {
			acymin = noise_Acy[i];
		}
		else { acymin = acymin; }

		if (aczmin < noise_Acz[i]) {
			aczmin = noise_Acz[i];
		}
		else { aczmin = aczmin; }
	}
	// put maximum and minimum noise values into callibrated arrays

	/*gyromax[0] = gyxmax; gyromax[1] = gyymax; gyromax[2] = gyzmax;
	gyromin[0] = gyxmin; gyromin[1] = gyymin; gyromin[2] = gyzmin;*/


	accelmax[0] = acxmax; accelmax[1] = acymax; accelmax[2] = aczmax;
	accelmin[0] = acxmin; accelmin[1] = acymin; accelmin[2] = aczmin;

}

void accFilter() { // filters mpu data by prcessing net values
    // passing processed but unfiltered acc to function for noise filtration
	float accel_X = fine_acx; 
	float accel_Y = fine_acy;
	float accel_Z = fine_acz;


	// ACCELERATION FILTER

	if (accel_X > accelmin[0] && accel_X < accelmax[0]) { // if x  axis accel is within noise range
		real_acx = 0;
	}
	else { real_acx = accel_X; }


	if (accel_Y < accelmax[1] && accel_Y > accelmin[1]) { // if axis accel is within noise range
		real_acy = 0;
	}
	else { real_acy = accel_Y; }


	if (accel_Z < accelmax[2] && accel_Z > accelmin[2]) { // if z axis accel is within noise range
		real_acz = 0;
	}
	else { real_acz = abs(9.81 - accel_Z); } // particularly for z to account for gravity
}


void gyroFilter(){
// GYROSCOPE FILTER
    //realtime filtering of gyros for more stable gyros
    float gyro_X = gx;
    float gyro_Y = gy;
    float gyro_Z = gz;

	if (gyro_X < gyromax[0] && gyro_X > gyromin[0]) { // if x  axis gyro rotation is within noise range
		real_gyroX = 0;
	}
	else { real_gyroX = gyro_X; }


	if (gyro_Y < gyromax[1] && gyro_Y > gyromin[1]) { // if axis gyro rotation is within noise range
		real_gyroY = 0;
	}
	else { real_gyroY = gyro_Y; }


	if (gyro_Z < gyromax[2] && gyro_Z > gyromin[2]) { // if z axis gyro rotation is within noise range
		real_gyroZ = 0;
	}
	else { real_gyroZ = gyro_Z; }

}


   

void callibrate_gyro(int speed){
# define dataSize 100
 
  //gyrocope noise collection
  float noise_Gyx[dataSize];
  float noise_Gyy[dataSize];
  float noise_Gyz[dataSize];

  float gyxmax = noise_Gyx[0], gyymax = noise_Gyy[0], gyzmax = noise_Gyz[0], gyxmin = noise_Gyx[0], gyymin = noise_Gyy[0], gyzmin = noise_Gyz[0];
  
  for (int i = 0; i < dataSize; i = i + 1) {
      //passing unstable gyros for stable callibration by removing noise
	  noise_Gyx[i] = gx;
	  noise_Gyy[i] = gy;
	  noise_Gyz[i] = gz;

	  delay(speed);// callibration speed


	  if (i == dataSize - 1) {
		  Serial.println("Data is collected");
		  continue;
	  }
	  else if (i < dataSize) {
		  Serial.println("Callibrating gyroscopes");
	  }
	  // for finding minimum values --algorithm: - if im greater than you, make me you else make me me (dont change me)
	  //'' '''''''''maximum'''''''''''''''''' -      if im less '''''''''''''''''''''''''''''''''''''''''''''''''''''''
	  
	  // iteration to find gyroscope max noise value
	  if (gyxmax < noise_Gyx[i]) {
		  gyxmax = noise_Gyx[i];
	   }
	  else { gyxmax = gyxmax; }

	  if (gyymax < noise_Gyy[i]) {
		  gyymax = noise_Gyy[i];
	  }
	  else { gyymax = gyymax; }

	  if (gyzmax < noise_Gyz[i]) {
		  gyzmax = noise_Gyz[i];
	  }
	  else { gyzmax = gyzmax; }
	  
	  //min gyro noise value
	  if (gyxmin > noise_Gyx[i]) {
		  gyxmin = noise_Gyx[i];
	  }
	  else { gyxmin = gyxmin; }

	  if (gyymin > noise_Gyx[i]) {
		  gyymin = noise_Gyx[i];
	  }
	  else { gyymin = gyymin; }
	 
	  if (gyzmin < noise_Gyx[i]) {
		  gyzmin = noise_Gyx[i];
	  }
	  else { gyzmin = gyzmin; }

}
  // put maximum and minimum noise values into callibrated arrays

  gyromax[0] = gyxmax; gyromax[1] = gyymax; gyromax[2] = gyzmax;
  gyromin[0] = gyxmin; gyromin[1] = gyymin; gyromin[2] = gyzmin;
  
}

  




void distint_lin_acc() {
	//distintly separates rotational acceleration from linear acceleration
	


}