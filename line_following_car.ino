#include <ECE3.h>

uint16_t sensorValues[8];

uint16_t calibration0[8];
uint16_t calibration1[8];
uint16_t calibration2[8];
uint16_t calibration3[8];
uint16_t calibration4[8];
uint16_t calibration5[8];
uint16_t calibration6[8];
uint16_t calibration7[8];
uint16_t calibration8[8];
uint16_t calibration9[8];

uint16_t fusedData[8];

int calibrationValuesMin[8];
int calibrationValuesMax[8];
int calibrationArray[10][8];

const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwn_pin = 39;

int left_push_pin_value;
int right_push_pin_value;

const int left_push_pin = PUSH2;
const int right_push_pin = PUSH1;

const int LED_RF = P2_0;

const float left_default_spd = 80;
const float right_default_spd = 80;

float left_past_spd = left_default_spd;
float right_past_spd = right_default_spd;

float left_cur_spd = left_default_spd;
float right_cur_spd = right_default_spd;

const double kP = 0.075;
const double kD = 1.5;
int fusionConstant;
int pastFusionConstant = 5;

int overallMinSensorValue = 2500;
int overallMaxSensorValue = 0;
int minSensorValue = 2500;
int maxSensorValue = 0;

int startMoving = 0;
int startCalibration = 0;
int maxSensorValuesIndex = 0;

int showingBlack = 0;
int showingBlackTurnaround = 0;
int timesTurned = 0;
int firstTimeMoving = 1;
int turn = 0;

void setup()
{
  ECE3_Init();
  Serial.begin(9600);
  delay(2000);

  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwn_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH);

  pinMode(left_push_pin, INPUT_PULLUP);
  pinMode(right_push_pin, INPUT_PULLUP);

  pinMode(LED_RF, OUTPUT);

}

void loop()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);
  
  // checking push buttons
  left_push_pin_value = digitalRead(left_push_pin);
  right_push_pin_value = digitalRead(right_push_pin);

  if (right_push_pin_value == 0){
    startCalibration = 1;
  }
  if (left_push_pin_value == 0){
    startMoving = 1;
  }

  // calibrate if right push button is pressed 
  if (startCalibration == 1){
    
    // read sensor values
    ECE3_read_IR(calibration0);  
    delay(500);
    ECE3_read_IR(calibration1);
    delay(500);
    ECE3_read_IR(calibration2);
    delay(500);
    ECE3_read_IR(calibration3);
    delay(500);
    ECE3_read_IR(calibration4);
    delay(500);
    ECE3_read_IR(calibration5);
    delay(500);
    ECE3_read_IR(calibration6);
    delay(500);
    ECE3_read_IR(calibration7);
    delay(500);
    ECE3_read_IR(calibration8);
    delay(500);
    ECE3_read_IR(calibration9);

    // get all calibration values into a single array (manually)
    for (unsigned char a = 0; a < 8; a++){ 
      calibrationArray[0][a] = calibration0[a];
      }
    for (unsigned char a = 0; a < 8; a++){ 
      calibrationArray[1][a] = calibration1[a];
      }
    for (unsigned char a = 0; a < 8; a++){ 
      calibrationArray[2][a] = calibration2[a]; 
      }
    for (unsigned char a = 0; a < 8; a++){ 
      calibrationArray[3][a] = calibration3[a];
      }
    for (unsigned char a = 0; a < 8; a++){ 
      calibrationArray[4][a] = calibration4[a]; 
      }
    for (unsigned char a = 0; a < 8; a++){ 
      calibrationArray[5][a] = calibration5[a];
      }
    for (unsigned char a = 0; a < 8; a++){ 
      calibrationArray[6][a] = calibration6[a]; 
      }
    for (unsigned char a = 0; a < 8; a++){ 
      calibrationArray[7][a] = calibration7[a];
      }
    for (unsigned char a = 0; a < 8; a++){ 
      calibrationArray[8][a] = calibration8[a]; 
      }
    for (unsigned char a = 0; a < 8; a++){ 
      calibrationArray[9][a] = calibration9[a]; 
      }

    for (unsigned char i = 0; i < 8; i++){
      // find min value for each sensor
      minSensorValue = 2500;
      for (unsigned char j = 0; j < 10; j++){
        if (calibrationArray[j][i] < minSensorValue){
          minSensorValue = calibrationArray[j][i];
        }
      }
      calibrationValuesMin[i] = minSensorValue;
    }

    for (unsigned char i = 0; i < 8; i++){
      maxSensorValue = 0;
      for (unsigned char k = 0; k < 10; k++){
        if (calibrationArray[k][i] > maxSensorValue){
          maxSensorValue = calibrationArray[k][i];
        }
      }
      calibrationValuesMax[i] = maxSensorValue;    
    }

    // find max value for all sensors
    overallMaxSensorValue = 0;
    for (unsigned char i = 0; i < 8; i++){
        if (calibrationValuesMax[i] > overallMaxSensorValue){
          overallMaxSensorValue = calibrationValuesMax[i];
        }
      }

    //find min value of all sensors
    overallMinSensorValue = 0;
    for (unsigned char i = 0; i < 8; i++){
        if (calibrationValuesMin[i] < overallMinSensorValue){
          overallMinSensorValue = calibrationValuesMin[i];
        }
      }

    digitalWrite(LED_RF, HIGH);
    delay(500);
    digitalWrite(LED_RF, LOW);
    delay(100);
    digitalWrite(LED_RF, HIGH);
    delay(500);
    digitalWrite(LED_RF, LOW);
    delay(100);
    digitalWrite(LED_RF, HIGH);
    delay(500);
    digitalWrite(LED_RF, LOW);

    // reset calibration flag to off
    startCalibration = 0;
  }

  // move if left push button is pressed
  if (startMoving == 1){
    
    if (firstTimeMoving == 1){
      delay(2000);
      firstTimeMoving = 0;
    }
    
    
    // normalization + fusion
    for (unsigned char i = 0; i < 7; i++)
    {
      fusedData[i] = ((abs(sensorValues[i] - calibrationValuesMin[i])*1000)/2500);
    }

    fusionConstant = -2*fusedData[0] + -1.75*fusedData[1] + -1.5*fusedData[2] + -1*fusedData[3] +
                     1*fusedData[4] + 1.5*fusedData[5] + 1.75*fusedData[6] + 2*fusedData[7];

   
    // find sensor reading highest value
    maxSensorValuesIndex = 0;
    maxSensorValue = 0;

    for (unsigned char i = 0; i < 7; i++){
      if (fusedData[i] > maxSensorValue){
        maxSensorValue = fusedData[i];
        maxSensorValuesIndex = i;
        }
    }

    // adjusting speed
    if (fusionConstant < 0){
      left_cur_spd = left_default_spd + kP*abs(fusionConstant) + kD *(abs(fusionConstant) - abs(pastFusionConstant));
      right_cur_spd = right_default_spd;
    }
    if (fusionConstant > 0) {
      left_cur_spd = left_default_spd;
      right_cur_spd = right_default_spd + kP*abs(fusionConstant) + kD *(abs(fusionConstant) - abs(pastFusionConstant));
    }
    
    //turn around condition
    showingBlack = 0;
    for (unsigned char i = 0; i < 7; i++){
      if (sensorValues[i] > (calibrationValuesMax[i] - 50)){
        showingBlack += 1;
        }
    }

    if (showingBlack > 5){
      showingBlackTurnaround += 1;
    }

    if (showingBlackTurnaround > 0){
      turn = 1;
      showingBlackTurnaround = 0;
    }

    
    if (turn == 1 && timesTurned > 0){
        left_cur_spd = 0;
        right_cur_spd = 0;
        analogWrite(left_pwm_pin, left_cur_spd);
        analogWrite(right_pwn_pin, right_cur_spd);
        digitalWrite(right_nslp_pin, LOW);
        digitalWrite(left_nslp_pin, LOW);
    }
    
    if (turn == 1 && timesTurned == 0){
        //stop car
        left_cur_spd = 0;
        right_cur_spd = 0;
        analogWrite(left_pwm_pin, left_cur_spd);
        analogWrite(right_pwn_pin, right_cur_spd);
        //turn car
        left_cur_spd = 250;
        right_cur_spd = 250;
        digitalWrite(left_dir_pin, HIGH);
        analogWrite(left_pwm_pin, left_cur_spd);
        analogWrite(right_pwn_pin, right_cur_spd);
        delay(195);
        //move car forward off end block a little
        digitalWrite(left_dir_pin, LOW);
        left_cur_spd = left_default_spd;
        right_cur_spd = right_default_spd;
        analogWrite(left_pwm_pin, left_cur_spd);
        analogWrite(right_pwn_pin, right_cur_spd);
        turn = 0;
        timesTurned += 1;
    }


    analogWrite(left_pwm_pin, left_cur_spd);
    analogWrite(right_pwn_pin, right_cur_spd);
    
    pastFusionConstant = fusionConstant;
  }
}
