//#include <nRF24L01.h>
//#include <RF24.h>
//#include <RF24_config.h>

#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#include <Servo.h>
#include <Wire.h>

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

// Latest PIN
#define MOTOR_1 B10000000 // digital pin 7
#define MOTOR_2 B01000000 // digital pin 6
#define MOTOR_3 B00100000 // digital pin 5
#define MOTOR_4 B00010000 // digital pin 4

#define MOTOR_MIN 950
//#define MOTOR_MAX 2000
#define MOTOR_MAX 1300
#define MOTOR_KEEP_RUNNING 1060

#define SLAVE_ADDRESS 0x04
#define INIT_GYRO_NUM 200
//#define INIT_GYRO_NUM 100

#define SET_ROLL 1
#define SET_PITCH 2
#define SET_YAW 3
#define SET_POWER 4
#define SET_ROLL_OUTER_P 5
#define SET_ROLL_P 6
#define SET_ROLL_I 7
#define SET_ROLL_D 8
#define SET_PITCH_OUTER_P 9
#define SET_PITCH_P 10
#define SET_PITCH_I 11
#define SET_PITCH_D 12
#define SET_YAW_OUTER_P 13
#define SET_YAW_P 14
#define SET_YAW_I 15
#define SET_YAW_D 16
#define TRIM_ROLL 17
#define TRIM_PITCH 18
#define TRIM_YAW 19

#define PI_BUFF_SIZE 6

#define YAW_COMP_BUFF_SIZE 30

#define CMD_NONE              0x00
#define CMD_HEADER_START      0x6E
#define CMD_CONTROL           0x6E
#define CMD_BUFF_LEN          5
#define GREEN_LED_PIN         8
#define CONTROL_VALUE_THRESHOLD   40
#define POWER_VALUE_THRESHOLD     100

//bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

float ypr[3];
float yprLast[3];
int16_t gyro[3];

uint8_t serialBuffCode = 0;
uint8_t serialBuffIndex = 0;
uint8_t serialBuffer[4] = {0x00, 0x00, 0x00, 0x00};

int target_power = 0;
/* IMU Data */
double accX = 0, accY = 0, accZ = 0;
double gyroX = 0, gyroY = 0, gyroZ = 0;
int16_t tempRaw;

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

/*
double static outer_Krp = 1, Krp = 1, Kri = 0.001, Krd = 0.001;
double static outer_Kpp = 1, Kpp = 1, Kpi = 0.001, Kpd = 0.001;
double static outer_Kyp = 1, Kyp = 1, Kyi = 0.001, Kyd = 0.001;
*/
double static outer_Krp = 6, Krp = 2.24, Kri = 0.17, Krd = 0.0035; //Kri = 0.15
double static outer_Kpp = outer_Krp, Kpp = Krp, Kpi = Kri, Kpd = Krd;
double static outer_Kyp = 4, Kyp = 1, Kyi = 0, Kyd = 0;

double static roll_trim = 0, pitch_trim = 0, yaw_trim = 0;
double static rTarget_deg = 0, pTarget_deg = 0, yTarget_deg = 0;
double static rTrim_deg = 0, pTrim_deg = 0, yTrim_deg = 0;
double static rCurr_deg = 0, pCurr_deg = 0, yCurr_deg = 0;
double static rErr = 0, pErr = 0, yErr = 0;
double static rErr1 = 0, pErr1 = 0, yErr1 = 0;
double static r_p_control = 0, r_i_control = 0, r_d_control = 0;
double static p_p_control = 0, p_i_control = 0, p_d_control = 0;
double static y_p_control = 0, y_i_control = 0, y_d_control = 0;
double static r_p_control1 = 0;
double static r_pid_control = 0, p_pid_control = 0, y_pid_control = 0;
double static r_rate_err = 0, r_rate_err1 = 0;
double static p_rate_err = 0, p_rate_err1 = 0;
double static y_rate_err = 0, y_rate_err1 = 0;

double loop_cycle = 0.004;
int loop_micro = 4000;
unsigned long m1_timer, m2_timer, m3_timer, m4_timer, esc_timer;
unsigned long timer;
uint8_t i2cData[14]; // Buffer for I2C data

uint8_t pi_buf[PI_BUFF_SIZE];
//**SPI volatile byte pos;
//**SPI volatile boolean process_it;
//char buf[100];

int output[4];
Servo motor[4];
/*
bool isInit_r_hist = false, isInit_p_hist = false;
int r_dh_num = 0, p_dh_num;
int dHist_len = 32;
float dHist[32];
*/
bool isInit_hist = false;
int r_dh_num = 0;
int p_dh_num = 0;
int dHist_len = 32;
float r_dHist[32];
float p_dHist[32];



double temp_ax[6], temp_ay[6], temp_az[6];
double temp_gx[6], temp_gy[6], temp_gz[6];


//Declaring some global variables
int32_t gyro_x, gyro_y, gyro_z;
int32_t acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float angle_pitch, angle_roll, angle_yaw;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;


float yawCompBuffer[YAW_COMP_BUFF_SIZE];

int gyro_address = 0x6B;

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
RF24 radio(9,10);
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

int keep_cmd_led = 0;
unsigned long elapsed = 0;
unsigned long snapshot_time = millis();

bool print_command = false;
bool print_sensors = true;


void setup() {
  Serial.begin(9600);

  printf_begin();

  pinMode(GREEN_LED_PIN, OUTPUT);

   // Setup and configure rf radio
  radio.begin();
  radio.setAutoAck(false);
  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  radio.startListening();

  radio.printDetails();
  
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1,pipes[0]);
  
//**SPI  SPCR |= _BV(SPE);
//**SPI  SPCR &= ~_BV(MSTR);
//**SPI  SPCR |= _BV(SPIE);
//**SPI  pos = 0;
//**SPI  process_it = false;

//  pinMode(MISO, OUTPUT);
  // now turn on interrupts
//**SPI  SPI.attachInterrupt();

// For SPI??
//  Wire.begin();
//  TWBR = 12;

  // Latest PIN
  DDRD |= MOTOR_1 | MOTOR_2 | MOTOR_3 | MOTOR_4;
  for(int i=0; i<100; i++){
    PORTD |= MOTOR_1 | MOTOR_2 | MOTOR_3 | MOTOR_4;
    delay(1);
    PORTD &= ~(MOTOR_1 | MOTOR_2 | MOTOR_3 | MOTOR_4);
    delay(2);
  }

//  set_gyro_registers();
  setup_mpu_6050_registers();

  for (int cal_int = 0; cal_int < INIT_GYRO_NUM; cal_int ++){                  //Run this code 200 times
    if(cal_int % 125 == 0)Serial.print(".");                              //Print a dot on the LCD every 125 readings
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    // Latest PIN
    PORTD |= MOTOR_1 | MOTOR_2 | MOTOR_3 | MOTOR_4;
    delay(1);
    PORTD &= ~(MOTOR_1 | MOTOR_2 | MOTOR_3 | MOTOR_4);
    delay(2);
  }
  gyro_x_cal /= INIT_GYRO_NUM;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= INIT_GYRO_NUM;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= INIT_GYRO_NUM;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
  Serial.println();

//  initTemp(temp_ax); initTemp(temp_ay); initTemp(temp_az);
//  initTemp(temp_gx); initTemp(temp_gy); initTemp(temp_gz);

  timer = micros();

  
 // delay(2000);
 // digitalWrite(GREEN_LED_PIN, LOW);
}

void setPidValue(int set_num, float value){
  /*
  switch (set_num){
    case SET_ROLL_OUTER_P: outer_Krp = value; break;
    case SET_ROLL_P: Krp = value; break;
    case SET_ROLL_I: Kri = value; break;
    case SET_ROLL_D: Krd = value; break;
    case SET_PITCH_OUTER_P: outer_Kpp = value; break;
    case SET_PITCH_P: Kpp = value; break;
    case SET_PITCH_I: Kpi = value; break;
    case SET_PITCH_D: Kpd = value; break;
    case SET_YAW_OUTER_P: outer_Kyp = value; break;
    case SET_YAW_P: Kyp = value; break;
    case SET_YAW_I: Kyi = value; break;
    case SET_YAW_D: Kyd = value; break;
  }
  */
}


/*
int index = 0;
uint8_t pi_code = 0;
ISR (SPI_STC_vect){
  byte c = SPDR;  // grab byte from SPI Data Register
 // Serial.println(c, HEX);
//  memset(pi_buf, NULL, 5);
  if(pi_code == 0){
    pi_code = c;
 //   Serial.print("code: "); Serial.println(c);
  } else{
  //Serial.println(c, HEX);
 //   Serial.print("data: "); Serial.println(c, HEX);
    switch (pi_code){
      case SET_ROLL: {
        uint8_t raw_roll = c;
        if(raw_roll & 128) rTarget_deg = -(raw_roll ^ 128);
        else rTarget_deg = raw_roll;
        break;
      }
      case SET_PITCH: {
        uint8_t raw_pitch = c;
        if(raw_pitch & 128) pTarget_deg = -(raw_pitch ^ 128);
        else pTarget_deg = raw_pitch;
        break;
      }
      case SET_YAW: {
        uint8_t raw_yaw = c;
        if(raw_yaw & 128) yTarget_deg = -(raw_yaw ^ 128);
        else yTarget_deg = raw_yaw;
        break;
      }
      case SET_POWER: {
        uint8_t raw_power = c;
        target_power = raw_power;
        break;
      }
    //    pi_buf[index] = c;
    //    Serial.println(pi_buf[index]);
        
      case SET_ROLL_OUTER_P:
      case SET_ROLL_P:
      case SET_ROLL_I:
      case SET_ROLL_D:
      case SET_PITCH_OUTER_P:
      case SET_PITCH_P:
      case SET_PITCH_I:
      case SET_PITCH_D:
      case SET_YAW_OUTER_P:
      case SET_YAW_P:
      case SET_YAW_I:
      case SET_YAW_D:
        if(serialBuffCode != pi_code){
          serialBuffCode = pi_code;
          serialBuffIndex = 0;
          serialBuffer[serialBuffIndex++] = c;
        } else{
          serialBuffer[serialBuffIndex++] = c;
          if(serialBuffIndex == 4){
            float f = makeFloat(serialBuffer);
            setPidValue(serialBuffCode, f);
            serialBuffIndex = 0;
            serialBuffCode = 0;
          }
        }
        break;
      case TRIM_ROLL:
      case TRIM_PITCH:
      case TRIM_YAW:
      
  //      if(index < PI_BUFF_SIZE - 1){
    //      Serial.print("index : "); Serial.println(index);
  //        pi_buf[index++] = c;

  //        if(index == PI_BUFF_SIZE - 1){
  //          float f = makeFloat(pi_buf);
     //       Serial.println(f, 5);
  //          index = 0;
  //          pi_code = 0;
  //        }
  //      }
        break;
    //  default:
    //    pi_code = 0;
    }
    pi_code = 0;
  }

  // add to buffer if room
//  if (pos < sizeof pi_buf){
//    pi_buf[pos++] = c;
    // example: newline means time to process buffer
//    if (c == '\n') process_it = true;
  }  // end of room available
  
}  // end of interrupt routine SPI_STC_vect
*/


int count = 0;

int comp_counter = 0;
bool isCompInit = false;

int cmdBuff[CMD_BUFF_LEN] = {0};

void loop() {

  unsigned long start_time = millis();
  snapshot_time = millis();

  if(keep_cmd_led > 0){
    digitalWrite(GREEN_LED_PIN, HIGH);
    keep_cmd_led -= elapsed;
    keep_cmd_led = keep_cmd_led > 0 ? keep_cmd_led : 0;
  } else{
    digitalWrite(GREEN_LED_PIN, LOW);
  }


  if ( radio.available() ){
    // Dump the payloads until we've gotten everything
    unsigned long got_time;
    bool done = false;
    
    done = radio.read(&cmdBuff, CMD_BUFF_LEN * 2);

    if(done){

      if(cmdBuff[0] == CMD_CONTROL){
        rTarget_deg = abs(cmdBuff[1]) <= CONTROL_VALUE_THRESHOLD ? cmdBuff[1] : rTarget_deg;
        pTarget_deg = abs(cmdBuff[2]) <= CONTROL_VALUE_THRESHOLD ? cmdBuff[2] : pTarget_deg;
        yTarget_deg = abs(cmdBuff[3]) <= CONTROL_VALUE_THRESHOLD ? cmdBuff[3] : yTarget_deg;
        target_power = cmdBuff[4] <= POWER_VALUE_THRESHOLD ? cmdBuff[4] : target_power;
      }

      if(print_command){
        printf("buff : ");
        for(int i=0; i<CMD_BUFF_LEN; i++){
          printf("%d, ", cmdBuff[i]);
        }
        printf("\n");
      }
      
      
      keep_cmd_led = 100;
    }

    while (!done){
      // Fetch the payload, and see if this was the last one.
     // done = radio.read( &got_time, sizeof(unsigned long) );
     int temp;
     done = radio.read(temp, 1);
      // Spew it
      /*
      Serial.print("temp : ");
      Serial.println(temp);
      */
   }

  }

  
  
  read_mpu_6050_data();

  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  angle_yaw += gyro_z * 0.0000611;                                    //Calculate the traveled yaw angle and add this to the angle_yaw variable

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angle
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angle
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  } else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  //To dampen the pitch and roll angles a complementary filter is used
  kalAngleX = kalAngleX * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  kalAngleY = kalAngleY * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  if(angle_yaw > 180){
    angle_yaw = -360 + angle_yaw;
  } else if(angle_yaw < -180){
    angle_yaw = 360 - angle_yaw;
  }

  if(target_power < 1 || yTarget_deg != 0){
    angle_yaw = 0;
  }

  kalAngleZ = angle_yaw;

  double gyroXrate = gyro_x / 131.0;
  double gyroYrate = gyro_y / 131.0;
  double gyroZrate = gyro_z / 131.0;


  rCurr_deg = kalAngleX;
  pCurr_deg = kalAngleY;
  yCurr_deg = kalAngleZ;
  rTrim_deg = rCurr_deg + roll_trim;
  pTrim_deg = pCurr_deg + pitch_trim;
  yTrim_deg = yCurr_deg + yaw_trim;
  rErr = rTarget_deg - rTrim_deg;
  pErr = pTarget_deg - pTrim_deg;
  yErr = yTarget_deg - yTrim_deg;

  if(rErr > 40) rErr = 40;
  else if(rErr < -40) rErr = -40;

  if(pErr > 40) pErr = 40;
  else if(pErr < -40) pErr = -40;

  if(yErr > 40) yErr = 40;
  else if(yErr < -40) yErr = -40;

  // roll, pitch P control
  /*
  double r_gyro_err = gyroXrate * outer_Krp;
  double r_stable_err = Krp * rErr;
  double r_rate_err = (Krp * rErr) - (gyroXrate * outer_Krp);
  r_p_control = r_rate_err;
  */

  r_rate_err = (outer_Krp * rErr) - gyroXrate;
  r_p_control = r_rate_err * Krp;

    

/*
  dHist[dh_num++] = Krd * (r_rate_err - r_rate_err1) / loop_cycle;
  if(dh_num >= dHist_len){
    dh_num = 0;
    isInit_hist = true;
  }
  double der = 0;
  for(int i=0; i<dHist_len; i++){
    der += dHist[i];
  }
  if(isInit_hist == true) der /= dHist_len;
  else der = (r_rate_err - r_rate_err1) / loop_cycle;
  der = Krd * der;
*/
//  r_d_control = Krd * (r_rate_err - r_rate_err1) / loop_cycle;
  // pitch P, D control

  p_rate_err = (outer_Kpp * pErr) - gyroYrate;
  p_p_control = p_rate_err * Kpp;

  y_rate_err = (outer_Kyp * yErr) - gyroZrate;
//  y_p_control = y_rate_err * Kyp;
  y_p_control = y_rate_err * Kyp;

  float d_tau = 0.05;
  float r_d_comp = (r_rate_err - r_rate_err1) / loop_cycle;
  float p_d_comp = (p_rate_err - p_rate_err1) / loop_cycle;
  float y_d_comp = (y_rate_err - y_rate_err1) / loop_cycle;
  r_d_control = (d_tau * r_d_control + loop_cycle * r_d_comp) / (d_tau + loop_cycle);
  p_d_control = (d_tau * p_d_control + loop_cycle * p_d_comp) / (d_tau + loop_cycle);
  y_d_control = (d_tau * y_d_control + loop_cycle * y_d_comp) / (d_tau + loop_cycle);
  
  // roll, pitch, yaw I control
  if(target_power != 0){
    r_i_control += Kri * r_rate_err * loop_cycle;
    if(r_i_control > 100) r_i_control = 100;
    else if(r_i_control < -100) r_i_control = -100;

    p_i_control += Kpi * p_rate_err * loop_cycle;
    if(p_i_control > 100) p_i_control = 100;
    else if(p_i_control < -100) p_i_control = -100;

    y_i_control += Kyi * y_rate_err * loop_cycle;
    if(y_i_control > 100) y_i_control = 100;
    else if(y_i_control < -100) y_i_control = -100;
  }
  
  r_pid_control = r_p_control + r_i_control + r_d_control * Krd;
  p_pid_control = p_p_control + p_i_control + p_d_control * Kpd;
  y_pid_control = y_p_control + y_i_control + y_d_control * Kyd;

  rErr1 = rErr;  pErr1 = pErr;  yErr1 = yErr;
  r_rate_err1 = r_rate_err;
  p_rate_err1 = p_rate_err;
  y_rate_err1 = y_rate_err;

  if(target_power != 0){
    int power = target_power * 8 + MOTOR_MIN;

    if(p_pid_control < 0) p_pid_control = -p_pid_control;
    if(r_pid_control < 0) r_pid_control = -r_pid_control;
    if(y_pid_control < 0) y_pid_control = -y_pid_control;

    if(p_pid_control > 200) p_pid_control = 200;
    if(r_pid_control > 200) r_pid_control = 200;
    if(y_pid_control > 200) y_pid_control = 200;

    if(r_rate_err > 0){
      output[0] = power - r_pid_control;
      output[1] = power - r_pid_control;
      output[2] = power + r_pid_control;
      output[3] = power + r_pid_control;
    } else{
      output[0] = power + r_pid_control;
      output[1] = power + r_pid_control;
      output[2] = power - r_pid_control;
      output[3] = power - r_pid_control;
    }
 
    if(p_rate_err < 0){
      output[0] -= p_pid_control;
      output[1] += p_pid_control;
      output[2] -= p_pid_control;
      output[3] += p_pid_control;
    } else{
      output[0] += p_pid_control;
      output[1] -= p_pid_control;
      output[2] += p_pid_control;
      output[3] -= p_pid_control;
    }

    if(y_rate_err < 0){
      output[0] -= y_pid_control;
      output[1] += y_pid_control;
      output[2] += y_pid_control;
      output[3] -= y_pid_control;
    } else{
      output[0] += y_pid_control;
      output[1] -= y_pid_control;
      output[2] -= y_pid_control;
      output[3] += y_pid_control;
    }

    output[0] = output[0] > MOTOR_MAX - 100 ? MOTOR_MAX - 100 : output[0];
    output[1] = output[1] > MOTOR_MAX - 100 ? MOTOR_MAX - 100 : output[1];
    output[2] = output[2] > MOTOR_MAX - 100 ? MOTOR_MAX - 100 : output[2];
    output[3] = output[3] > MOTOR_MAX - 100 ? MOTOR_MAX - 100 : output[3];
    /*
    output[0] = output[0] < MOTOR_MIN ? MOTOR_MIN : output[0];
    output[1] = output[1] < MOTOR_MIN ? MOTOR_MIN : output[1];
    output[2] = output[2] < MOTOR_MIN ? MOTOR_MIN : output[2];
    output[3] = output[3] < MOTOR_MIN ? MOTOR_MIN : output[3];
    */
    output[0] = output[0] < MOTOR_KEEP_RUNNING ? MOTOR_KEEP_RUNNING : output[0];
    output[1] = output[1] < MOTOR_KEEP_RUNNING ? MOTOR_KEEP_RUNNING : output[1];
    output[2] = output[2] < MOTOR_KEEP_RUNNING ? MOTOR_KEEP_RUNNING : output[2];
    output[3] = output[3] < MOTOR_KEEP_RUNNING ? MOTOR_KEEP_RUNNING : output[3];
  } else{
    output[0] = MOTOR_MIN;
    output[1] = MOTOR_MIN;
    output[2] = MOTOR_MIN;
    output[3] = MOTOR_MIN;
  }
  

  int els = micros() - timer;
  /*
  while(micros() - timer < loop_micro);
  timer = micros();
  */
  static int i = 0;
  if(i++ > 15 && print_sensors){
  //if(i++ > 15 && false){
    /*
    Serial.print(rTarget_deg); Serial.print("\t");
    Serial.print(pTarget_deg); Serial.print("\t");
    Serial.print(target_power); Serial.print("\t");
    Serial.print(output[0]); Serial.print("\t");
    Serial.print(output[1]); Serial.print("\t");
    */

    Serial.print(angle_pitch); Serial.print("\t");
    Serial.print(angle_roll); Serial.print("\t");
    Serial.print(angle_yaw); Serial.print("\t");
    Serial.print(acc_total_vector); Serial.print("\t");
    Serial.print(angle_pitch_acc); Serial.print("\t");
    Serial.print(angle_roll_acc); Serial.print("\t");
    Serial.print(kalAngleX); Serial.print("\t");
    Serial.print(kalAngleY); Serial.print("\t");
    
//    
//    Serial.print(kalAngleX); Serial.print("\t");
//    Serial.print(kalAngleY); Serial.print("\t");
//    Serial.print(kalAngleZ); Serial.print("\t");
//
//    Serial.print(cmdBuff[0]); Serial.print("\t");
//    Serial.print(cmdBuff[1]); Serial.print("\t");
//    Serial.print(cmdBuff[2]); Serial.print("\t");
//    Serial.print(cmdBuff[3]); Serial.print("\t");
//    Serial.print(cmdBuff[4]); Serial.print("\t");
//    

/*
    Serial.print("Roll : ");
    Serial.print(outer_Krp); Serial.print(", "); Serial.print(Krp); Serial.print(", ");
    Serial.print(Kri); Serial.print(", "); Serial.print(Krd); Serial.print("\t");
    */
    
  //  Serial.print(r_p_control); Serial.print("\t");
  //  Serial.print(p_p_control); Serial.print("\t");
  //  Serial.print(r_i_control); Serial.print("\t");
  //  Serial.print(p_i_control); Serial.print("\t");
    
    //Serial.print(y_i_control); Serial.print("\t");
    /*
    
    Serial.print(y_pid_control); Serial.print("\t");
    

    Serial.print(output[0]); Serial.print("\t");
    Serial.print(output[1]); Serial.print("\t");
    Serial.print(output[2]); Serial.print("\t");
    Serial.print(output[3]); Serial.print("\t");
*/

/*
    Serial.print(outer_Krp); Serial.print("\t");
    Serial.print(Krp); Serial.print("\t");
    Serial.print(Kri); Serial.print("\t");
    Serial.print(Krd); Serial.print("\t");
    Serial.print(outer_Kpp); Serial.print("\t");
    Serial.print(Kpp); Serial.print("\t");
    Serial.print(Kpi); Serial.print("\t");
    Serial.print(Kpd); Serial.print("\t");
    Serial.print(outer_Kyp); Serial.print("\t");
    Serial.print(Kyp); Serial.print("\t");
    Serial.print(Kyi); Serial.print("\t");
    Serial.print(Kyd); Serial.print("\t");
  */
    Serial.println(els);
    i = 0;
  }

  
  while(micros() - timer < loop_micro);
  timer = micros();

  // Latest PIN
  PORTD |= MOTOR_1 | MOTOR_2 | MOTOR_3 | MOTOR_4;
  m1_timer = output[0] + timer;
  m2_timer = output[1] + timer;
  m3_timer = output[2] + timer;
  m4_timer = output[3] + timer;


  // Latest PIN
//  while(PORTD & B10010000){
  while(PORTD & B11110000){
    esc_timer = micros();
    if(m1_timer <= esc_timer) PORTD &= ~MOTOR_1;
    if(m2_timer <= esc_timer) PORTD &= ~MOTOR_2;
    if(m3_timer <= esc_timer) PORTD &= ~MOTOR_3;
    if(m4_timer <= esc_timer) PORTD &= ~MOTOR_4;
  }


  


/*
#if 0 // Set to 1 to print the temperature
  Serial.print("\t");
  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
#endif
*/


  elapsed = millis() - start_time;
  
    
}

float makeFloat(uint8_t buf[]){
  uint32_t f_temp = (((uint32_t)buf[0]&0xFF)<<24);
  f_temp += (((uint32_t)buf[1]&0xFF)<<16);
  f_temp += (((uint32_t)buf[2]&0xFF)<<8);
  f_temp += (((uint32_t)buf[3]&0xFF));
  float f = *((float*)&f_temp);
//  Serial.print("float : "); Serial.println(f);
  return f;
}

void initTemp(double coef[]){
  coef[0] = 0;
  coef[1] = 0;
  coef[2] = 0;
  coef[3] = 0;
  coef[4] = 0;
  coef[5] = 0;
}

double tempAmender(double coef[], short data, double temp){
  coef[0] += data * data;
  coef[1] += data;
  coef[2] += data;
  coef[3] += 1;
  coef[4] += data * temp;
  coef[5] += temp;

  double s = 1 / (coef[0] * coef[3] - coef[1] * coef[2]);
  double temp_coef = s * (coef[3] * coef[4] - coef[1] * coef[5]);

  double result = data - temp_coef * temp;
  if(isnan(result) != 0) result = data;

  return result;
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
//  Wire.beginTransmission(gyro_address);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
//  Wire.requestFrom(gyro_address,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
 // temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
}

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  // Low Pass Filter
/*
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x1A);                                                    
  Wire.write(0x03);                                                    
  Wire.endTransmission();
*/
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  
  Wire.beginTransmission(0x68);                                      //Start communication with the address found during search
  Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                    //End the transmission with the gyro    
 
}


void set_gyro_registers(){
  //Setup the MPU-6050

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1B);                                                          //Start reading @ register 0x1B
    Wire.endTransmission();                                                    //End the transmission
    Wire.requestFrom(gyro_address, 1);                                         //Request 1 bytes from the gyro
/*
    while(Wire.available() < 1);                                               //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){                                                   //Check if the value is 0x08
      Serial.println("Warning!!!");                                                   //Turn on the warning led
      while(1)delay(10);                                                       //Stay in this loop for ever
    }
  */  
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro    
 
}


/*
char* readCodeData(){
//  char buf[100];
//  memset(buf, '\0', 100);
  int i = 0;
  if(Serial.available() > 0){
    char read_ch;
    while(true){
      read_ch = Serial.read();
      if(read_ch == NULL || read_ch == 10) break;
      if(read_ch > 0 && read_ch < 128){
        buf[i++] = read_ch;
      }
    }
  }
  buf[i] = '\0';
  return buf;
}
*/


/*
String readCodeData(){
  String code_data = "";
  if(Serial.available() > 0){
    char read_ch;
    while(true){
      read_ch = Serial.read();
      if(read_ch == NULL || read_ch == 10) break;
      if(read_ch > 0 && read_ch < 128){
        code_data.concat(read_ch);
      }
    }
  }
  return code_data;
}
*/
