#include <Wire.h>


#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
MPU6050 mpu;
// ======================================================================================

//===                           from i2cdev dmp lib                                   ===

// ======================================================================================

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];
float pitch, roll;

unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;


void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

inline unsigned long get_last_time() {
  return last_read_time;
}
inline float get_last_x_angle() {
  return last_x_angle;
}
inline float get_last_y_angle() {
  return last_y_angle;
}
inline float get_last_z_angle() {
  return last_z_angle;
}
inline float get_last_gyro_x_angle() {
  return last_gyro_x_angle;
}
inline float get_last_gyro_y_angle() {
  return last_gyro_y_angle;
}
inline float get_last_gyro_z_angle() {
  return last_gyro_z_angle;
}

//  Use the following global variables
//  to calibrate the gyroscope sensor and accelerometer readings
float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;


// This global variable tells us how to scale gyroscope data
float    GYRO_FACTOR;

// This global varible tells how to scale acclerometer data
float    ACCEL_FACTOR;

// Variables to store the values from the sensor readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Buffer for data output
char dataOut[256];
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                CALIBRATION_ROUTINE                       ===
// ================================================================
// Simple calibration - just average first few readings to subtract
// from the later data
void calibrate_sensors() {
  int       num_readings = 1000;

  // Discard the first reading (don't know if this is needed or
  // not, however, it won't hurt.)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Read and average the raw values
  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    base_x_gyro += gx;
    base_y_gyro += gy;
    base_z_gyro += gz;
    base_x_accel += ax;
    base_y_accel += ay;
    base_y_accel += az;
  }

  base_x_gyro /= num_readings;
  base_y_gyro /= num_readings;
  base_z_gyro /= num_readings;
  base_x_accel /= num_readings;
  base_y_accel /= num_readings;
  base_z_accel /= num_readings;
}


// ======================================================================================

// ======================================================================================

int8_t elevator = 4, rudder = 6, aileron_R = 3, aileron_L = 7, throt = 5; // initialising pin no's for servos
Servo Throttle, Elevator, Rudder, Aileron_R, Aileron_L;
uint16_t rx_ch[7], chmin[7] = {1500, 1500, 1500, 1500, 1500, 1500, 1500}, chmax[7] = {1800, 1800, 1800, 1800, 1800, 1800, 1800}, rfmin[7], rfmax[7]; // GIVING INITIAL VALUES
int16_t chm[7];
uint16_t count = 0;
byte last_ch[7];
bool flag_arm = 0, failsafe = 1, flag_accelval = 0, flag_calib = 0;// failsafe=1 ==> transmitter not calibrated,after calibration failsafe=1.
int16_t x = 500;
unsigned long timer[7];
float a, k;
float time_prev, time_curr, time_elapsed;

const float r2d = 180 * 7 / 22;
float throttle;
float pid_p_kp, pid_p_ki, pid_p_kd;
float pid_r_kp, pid_r_ki, pid_r_kd;
int8_t i;
void int_initialise();
void readSensors();

void arming();
void disarming();
float pitch_offset = 0, roll_offset = 0;
float ro, pi;
int8_t pitch_lim = 9;



void setup()
{
  Wire.begin();
  int_initialise();
  while (!Serial);

  devStatus = mpu.dmpInitialize();


  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // Set the full scale range of the gyro
    uint8_t FS_SEL = 0;
    //mpu.setFullScaleGyroRange(FS_SEL);

    // get default full scale value of gyro - may have changed from default
    // function call returns values between 0 and 3
    uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
    GYRO_FACTOR = 131.0 / (FS_SEL + 1);


    // get default full scale value of accelerometer - may not be default value.
    // Accelerometer scale factor doesn't reall matter as it divides out
    uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
    //ACCEL_FACTOR = 16384.0/(AFS_SEL + 1);

    // Set the full scale range of the accelerometer
    //uint8_t AFS_SEL = 0;
    //mpu.setFullScaleAccelRange(AFS_SEL);

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }


  calibrate_sensors();
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);



  pid_p_kp = 1    ; pid_p_kd = 0; pid_p_ki = 0;
  pid_r_kp = 1; pid_r_kd = 0; pid_r_ki = 0;                                                            //coefficients of pid
  a = -0.52;                                                                                            //complementary filter coeffiient


  Elevator.attach(elevator);                                      //Attaching servos to respective control surfaces
  Rudder.attach(rudder);
  Aileron_R.attach(aileron_R);
  Aileron_L.attach(aileron_L);
  Throttle.attach(throt);

  for (i = 0; i < 100; i++)
  {
    //delay(1);
    readSensors();
    roll_offset += roll;
    pitch_offset += pitch;

  }
  roll_offset /= i;
  pitch_offset  /=  i;

}

//////////////////////////////////////////////////           loop           ///////////////////////////////////////////


void loop()
{
  float pid_p_setpoint, pid_p_curr_value;
  float pid_p_prev_error, pid_p_error;
  float pid_r_setpoint, pid_r_curr_value;
  float pid_r_prev_error, pid_r_error;
  float pid_p_p_error, pid_p_i_error, pid_p_d_error;
  float pid_r_p_error, pid_r_i_error, pid_r_d_error;

  if (rx_ch[4] <= 1200 && rx_ch[3] > 1800 && rx_ch[1] > 1800 && rx_ch[2] <= 1200) //
  {
    failsafe == 0;
    //calibration();
  }

  readSensors();
  if (flag_arm == 0)
    arming();

  pid_p_setpoint = chm[2];

  pid_r_setpoint = chm[1];

  pid_p_curr_value = pitch - pitch_offset;
  pid_r_curr_value = roll - roll_offset;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  pid_p_p_error = pid_p_curr_value - pid_p_setpoint;
  pid_p_d_error = (pid_p_p_error - pid_p_prev_error);
  pid_p_i_error += pid_p_p_error;

  pid_p_error = pid_p_kp * pid_p_p_error + pid_p_kd * pid_p_d_error + pid_p_ki * pid_p_i_error;

  pid_p_prev_error = pid_p_p_error;

  if (pid_p_error < -80)
    pid_p_error = -80;
  else if (pid_p_error > 80)
    pid_p_error = 80;


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pid_r_p_error = pid_r_curr_value - pid_r_setpoint;
  pid_r_d_error = (pid_r_p_error - pid_r_prev_error);
  pid_r_i_error += pid_r_p_error;

  pid_r_error = pid_r_kp * pid_r_p_error + pid_r_kd * pid_r_d_error + pid_r_ki * pid_r_i_error;

  pid_r_prev_error = pid_r_p_error;

  if (pid_r_error < -80)
    pid_r_error = -80;
  else if (pid_r_error > 80)
    pid_r_error = 80;



  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////



  Aileron_R.write((int)(80 - pid_r_error)); //servo pointing towards fusilage
  Aileron_L.write((int)(80 - pid_r_error));  //servo pointing towards fusilage
  Elevator.write((int)(80 + pid_p_error));   // servo on right side away from fusilage
  Rudder.write(chm[4]);
  if (flag_arm == 1 && failsafe == 0)
  {

    Throttle.writeMicroseconds(chm[3]);
    //  Serial.print(throttle);
    //Serial.print("\t");

    disarming();
  }

}


////////////////////////////                ini_init                    ///////////////////////////////////////////////////
void int_initialise()
{
  PCICR |= (1 << PCIE0); // pin change interrupt for DIGITAL PINS ON UNO 8,9,10,11
  PCMSK0 |= (1 << PCINT0); //enables interrupt for pin 8
  PCMSK0 |= (1 << PCINT1); //enables interrupt for pin 9
  PCMSK0 |= (1 << PCINT2); //enables interrupt for pin 10
  PCMSK0 |= (1 << PCINT5); //enables interrupt for pin 13
}


/////////////////////////////                 INTERRUPUTS               ////////////////////////////////////
ISR(PCINT0_vect)    {
  ///////////////////          channel 1           /////////////////////
  if (last_ch[1] == 0 && (PINB & 0b00000001))        //checking state of digital pin 13
  {
    last_ch[1] = 1;
    timer[1] = micros();

  }
  else if (last_ch[1] == 1 && !(PINB & 0b00000001))
  {

    last_ch[1] = 0;
    rx_ch[1] = micros() - timer[1];
    // Serial.print(rx_ch[1]);
    if ((rx_ch[1] <= chmin[1]) && (rx_ch[1] > 990))
      chmin[1] = rx_ch[1];
    if ((rx_ch[1] >= chmax[1]) && (rx_ch[1] < 2050))
      chmax[1] = rx_ch[1];

    if (rx_ch[1] < 990)
      rx_ch[1] = chmin[1];
    chm[1] = (((float)(rx_ch[1] - chmin[1]) / (float)(chmax[1] - chmin[1])) * 160) - 80 ; // chm[]= mapped values of channels


  }
  ///////////////////                 channel 2            ////////////////////
  if (last_ch[2] == 0 && (PINB & 0b00000010))        //checking state of digital pin 14
  {
    last_ch[2] = 1;
    timer[2] = micros();
  }
  else if (last_ch[2] == 1 && !(PINB & 0b00000010))
  {
    last_ch[2] = 0;
    rx_ch[2] = micros() - timer[2];
    if ((rx_ch[2] <= chmin[2]) && (rx_ch[2] > 940))
      chmin[2] = rx_ch[2];
    if ((rx_ch[2] >= chmax[2]) && (rx_ch[2] < 2100))
      chmax[2] = rx_ch[2];
    if (rx_ch[2] < 940)
      rx_ch[2] = chmin[2];
    chm[2] = (((float)(rx_ch[2] - chmin[2]) / (float)(chmax[2] - chmin[2])) * 2 * pitch_lim ) - pitch_lim ; // chm[]= mapped values of channels
  }
  ///////////////////             channel 3                     /////////////////////
  if (last_ch[3] == 0 && (PINB & 0b00000100))        //checking state of digital pin 11
  {
    last_ch[3] = 1;
    timer[3] = micros();
  }
  else if (last_ch[3] == 1 && !(PINB & 0b00000100))
  {
    last_ch[3] = 0;
    rx_ch[3] = micros() - timer[3];
    chm[3] = rx_ch[3];


  }
  ///////////////////              channel 4               /////////////////////
  if (last_ch[4] == 0 && (PINB & 0b00100000))        //checking state of digital pin 10
  {
    last_ch[4] = 1;
    timer[4] = micros();
  }
  else if (last_ch[4] == 1 && !(PINB & 0b00100000))
  {
    last_ch[4] = 0;
    rx_ch[4] = micros() - timer[4];
    if ((rx_ch[4] <= chmin[4]) && (rx_ch[4] > 990))
      chmin[4] = rx_ch[4];
    if ((rx_ch[4] >= chmax[4]) && (rx_ch[4] < 2030))
      chmax[4] = rx_ch[4];
    if (rx_ch[4] < 990)
      rx_ch[4] = chmin[4];
    chm[4] = (((float)(rx_ch[4] - chmin[4]) / (float)(chmax[4] - chmin[4])) * 160) - 80 ; // chm[]= mapped values of channels
  }

}
//////////////////////////////////////            arming               //////////////////////////////////
void arming()
{

  if ((chm[3] < 1100 && chm[4] > 70) && failsafe == 0)
  {
    count++;
    //Serial.println(count);
    if (count > x)
    { Serial.println("armed");
      flag_arm = 1;
      ////Serial.println("armed");

    }
  }
  else if (chm[4] < 70 && chm[4] > -70)
  { count = 0;
  }
}

////////////////////////////             disarming             //////////////////////////////////////////////
void disarming()
{
  if (chm[3] < 1100 && chm[4] < -70 && flag_accelval == 0)
  {
    count++;
    if (count > x)
    { Serial.println("disarmed");
      flag_arm = 0;
      //  Serial.println("armed");
    }
  }
  else if (chm[4] > -70 && chm[4] < 70)
  {
    count = 0;
  }
}



////////////////////////////           readsensor          ///////////////////////////////
void readSensors()
{

  const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159

  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  unsigned long t_now = millis();


  float gyro_z;
  while (!mpuInterrupt && fifoCount < packetSize) {

    // Keep calculating the values of the complementary filter angles for comparison with DMP here
    // Read the raw accel/gyro values from the MPU-6050
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Get time of last raw data read
    unsigned long t_now = millis();

    // Remove offsets and scale gyro data
    float gyro_x = (gx - base_x_gyro) / GYRO_FACTOR;
    float gyro_y = (gy - base_y_gyro) / GYRO_FACTOR;
    gyro_z = (gz - base_z_gyro) / GYRO_FACTOR;
    float accel_x = ax; // - base_x_accel;
    float accel_y = ay; // - base_y_accel;
    float accel_z = az; // - base_z_accel;


    float accel_angle_y = atan(-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;
    float accel_angle_x = atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * RADIANS_TO_DEGREES;
    float accel_angle_z = 0;

    // Compute the (filtered) gyro angles
    float dt = (t_now - get_last_time()) / 1000.0;
    float gyro_angle_x = gyro_x * dt + get_last_x_angle();
    float gyro_angle_y = gyro_y * dt + get_last_y_angle();
    float gyro_angle_z = gyro_z * dt + get_last_z_angle();

    // Compute the drifting gyro angles
    float unfiltered_gyro_angle_x = gyro_x * dt + get_last_gyro_x_angle();
    float unfiltered_gyro_angle_y = gyro_y * dt + get_last_gyro_y_angle();
    float unfiltered_gyro_angle_z = gyro_z * dt + get_last_gyro_z_angle();


  }


  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Obtain Euler angles from buffer
    //mpu.dmpGetQuaternion(&q, fifoBuffer);
    //mpu.dmpGetEuler(euler, &q);

    // Obtain YPR angles from buffer
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }

  roll = (ypr[1] * r2d) - (ypr[2] * r2d) * sin(gyro_z * 0.000001066) ; // corrected values

  pitch = (ypr[2] * r2d) + (ypr[1] * r2d) * sin(gyro_z * 0.000001066);



}
