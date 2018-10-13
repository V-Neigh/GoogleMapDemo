#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

//gyroscopes
MPU6050 accelgyro1(0x69);
MPU6050 accelgyro2(0x68);

//for calibration
const int butPin1 = 7;
const int butPin2 = 8;

int buffersize=300;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=5;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

//for calculations
int16_t ax, ay, az, gx, gy, gz;
int16_t ax1, ay1, az1, gx1, gy1, gz1;

double timeStep, time, timePrev;
double arx, ary, arz, grx, gry, grz, gsx, gsy, gsz, rx, ry, rz;
double arx1, ary1, arz1, grx1, gry1, grz1, gsx1, gsy1, gsz1, rx1, ry1, rz1;

int i;
double gyroScale = 131;

//for posture check
int calibCnt = 0;
int16_t ag1x, ag1y, ag1z, ag2x, ag2y, ag2z;
bool saveCoords1 = false, saveCoords2 = false;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initializing");
  accelgyro1.initialize();
  accelgyro2.initialize();

  Serial.println("Testing device connection 1...");
  Serial.println(accelgyro1.testConnection() ? "MPU6050 #1 connection successful" : "MPU6050 1 connection failed");
  Serial.println("Testing device connection 2...");
  Serial.println(accelgyro2.testConnection() ? "MPU6050 #2 connection successful" : "MPU6050 2 connection failed");

  time = millis();

  i = 1;

  //reset offsets
  accelgyro1.setXAccelOffset(0);
  accelgyro1.setYAccelOffset(0);
  accelgyro1.setZAccelOffset(0);
  accelgyro1.setXGyroOffset(0);
  accelgyro1.setYGyroOffset(0);
  accelgyro1.setZGyroOffset(0);

  accelgyro2.setXAccelOffset(0);
  accelgyro2.setYAccelOffset(0);
  accelgyro2.setZAccelOffset(0);
  accelgyro2.setXGyroOffset(0);
  accelgyro2.setYGyroOffset(0);
  accelgyro2.setZGyroOffset(0);
}

void loop() {
  //MANUAL CALIBRATION
  if (digitalRead(butPin1)==HIGH){
    Serial.println("Recalibrating sensor 1");
    calibrateGyroMain(accelgyro1);
    saveCoords1 = true;
  }

  if (digitalRead(butPin2)==HIGH){
    Serial.println("Recalibrating sensor 2");
    calibrateGyroMain(accelgyro2);
    saveCoords2 = true;
  }

  //CALCULATIONS

  // set up time for integration
  timePrev = time;
  time = millis();
  timeStep = (time - timePrev) / 1000; // time-step in s

  // collect readings
  accelgyro1.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  accelgyro2.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);

  // apply gyro scale from datasheet
  gsx = gx/gyroScale;   gsy = gy/gyroScale;   gsz = gz/gyroScale;
  gsx1 = gx1/gyroScale;   gsy1 = gy1/gyroScale;   gsz1 = gz1/gyroScale;

  // calculate accelerometer angles
  arx = (180/3.141592) * atan(ax / sqrt(square(ay) + square(az))); 
  ary = (180/3.141592) * atan(ay / sqrt(square(ax) + square(az)));
  arz = (180/3.141592) * atan(sqrt(square(ay) + square(ax)) / az);
  arx1 = (180/3.141592) * atan(ax1 / sqrt(square(ay1) + square(az1))); 
  ary1 = (180/3.141592) * atan(ay1 / sqrt(square(ax1) + square(az1)));
  arz1 = (180/3.141592) * atan(sqrt(square(ay1) + square(ax1)) / az1);

  // set initial values equal to accel values
  if (i == 1) {
    grx = arx;
    gry = ary;
    grz = arz;
    grx1 = arx1;
    gry1 = ary1;
    grz1 = arz1;
  }
  // integrate to find the gyro angle
  else{
    grx = grx + (timeStep * gsx);
    gry = gry + (timeStep * gsy);
    grz = grz + (timeStep * gsz);
    grx1 = grx1 + (timeStep * gsx1);
    gry1 = gry1 + (timeStep * gsy1);
    grz1 = grz1 + (timeStep * gsz1);
  }

  // apply filter
  rx = (0.96 * arx) + (0.04 * grx);
  ry = (0.96 * ary) + (0.04 * gry);
  rz = (0.96 * arz) + (0.04 * grz);
  rx1 = (0.96 * arx1) + (0.04 * grx1);
  ry1 = (0.96 * ary1) + (0.04 * gry1);
  rz1 = (0.96 * arz1) + (0.04 * grz1);

  if (saveCoords1){
    ag1x = rx;
    ag1y = ry;
    ag1z = rz;
    saveCoords1 = false;
    Serial.println("Saved Coords");
  }
  if (saveCoords2){
    ag2x = rx1;
    ag2y = ry1;
    ag2z = rz1;
    saveCoords2 = false;
    Serial.println("Saved Coords");
  }

  // print result
  Serial.print(i);   Serial.print("\t");
//  Serial.print(timePrev);   Serial.print("\t");
//  Serial.print(time);   Serial.print("\t");
//  Serial.print(timeStep, 5);   Serial.print("\t\t");
//  Serial.print(ax);   Serial.print("\t");
//  Serial.print(ay);   Serial.print("\t");
//  Serial.print(az);   Serial.print("\t\t");
//  Serial.print(gx);   Serial.print("\t");
//  Serial.print(gy);   Serial.print("\t");
//  Serial.print(gz);   Serial.print("\t\t");
//  Serial.print(arx);   Serial.print("\t");
//  Serial.print(ary);   Serial.print("\t");
//  Serial.print(arz);   Serial.print("\t\t");
//  Serial.print(grx);   Serial.print("\t");
//  Serial.print(gry);   Serial.print("\t");
//  Serial.print(grz);   Serial.print("\t\t");
//  Serial.print(rx);   Serial.print("\t");
//  Serial.print(ry);   Serial.print("\t");
//  Serial.println(rz);

  Serial.print(rx);   Serial.print("\t");
  Serial.print(ry);   Serial.print("\t");
  Serial.println(rz); Serial.print("\t");
  Serial.print(rx1);   Serial.print("\t");
  Serial.print(ry1);   Serial.print("\t");
  Serial.println(rz1);

  i = i + 1;
  delay(1000);
}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void meansensors(MPU6050 accelgyro){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
 
  while (i<(buffersize+51)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>50 && i<=(buffersize+50)){
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+50)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}
 
void calibration(MPU6050 accelgyro){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;
 
  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);
 
    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);
 
    meansensors(accelgyro);
    Serial.println("...");
 
    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;
 
    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;
 
    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;
 
    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);
 
    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);
 
    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);
 
    if (ready==6) break;
  }
}

void calibrateGyroMain(MPU6050 accelgyro){
  if (state==0){
      Serial.println("\nReading sensors for first time...");
      meansensors(accelgyro);
      state++;
      delay(1000);
    }
   
    if (state==1) {
      Serial.println("\nCalculating offsets...");
      calibration(accelgyro);
      state++;
      delay(1000);
    }
   
    if (state==2) {
      meansensors(accelgyro);
      Serial.println("\nFINISHED!");
      Serial.print("\nSensor readings with offsets:\t");
      Serial.print(mean_ax); 
      Serial.print("\t");
      Serial.print(mean_ay); 
      Serial.print("\t");
      Serial.print(mean_az); 
      Serial.print("\t");
      Serial.print(mean_gx); 
      Serial.print("\t");
      Serial.print(mean_gy); 
      Serial.print("\t");
      Serial.println(mean_gz);
      Serial.print("Your offsets:\t");
      Serial.print(ax_offset); 
      Serial.print("\t");
      Serial.print(ay_offset); 
      Serial.print("\t");
      Serial.print(az_offset); 
      Serial.print("\t");
      Serial.print(gx_offset); 
      Serial.print("\t");
      Serial.print(gy_offset); 
      Serial.print("\t");
      Serial.println(gz_offset); 
      Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
      Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
      Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
      state = 0;
   }
   calibCnt++;
}

void checkPosture(){

}
