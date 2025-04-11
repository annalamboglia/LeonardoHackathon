#include <ESP32Servo.h>
#include "HT16K33_Lib_For_ESP32.h"

//MOTOR
#define left_ctrl  33  //define direction control pins of the left motor as gpio33
#define left_pwm  26   //define PWM control pins of the left motor as gpio26.
#define right_ctrl  32 //define direction control pins of the right motor as gpio32.
#define right_pwm  25  //define PWM control pins of the right motor as gpio25

//Ultrasonic sensor
#define TRIG_PIN 5 // Define the signal input of the ultrasonic sensor as gpio5.
#define ECHO_PIN 18 // Define the signal output of the ultrasonic sensor as gpio18.
#define MAX_DISTANCE 700 // Maximum sensor distance is rated at 400-500cm.
#define OBSTACLE_DISTANCE 20
#define SERVO_PIN 4
//#define PHOTOSENSITIVE_PIN 34 //Define the pins that ESP32 reads photosensitive

//timeOut= 2*MAX_DISTANCE /100 /340 *1000000 = MAX_DISTANCE*58.8
float timeOut = MAX_DISTANCE * 60; 
int soundVelocity = 340; // define sound speed=340m/s

//Variabili globali
int lightCount = 0;  
const int LIGHT_THRESHOLD = 3000;
//const int OBSTACLE_DISTANCE = 20;
const int speed = 110;

// Variabili per il timer
unsigned long startTime;
const unsigned long THREE_MINUTES = 180000; // 3 minuti in millisecondi (3 * 60 * 1000)

//servo
//const int servoPin = 4; //set the pin of the servo to gpio4.
Servo myservo;  // create servo object to control a servo

//Photosensor
//int photosensitiveADC;  
//photoresistors
#define light_L_Pin  34   //define the pins of the left photoresistor as gpio34
#define light_R_Pin  35   //define the pins of the right photoresistor as gpio35
int left_light; 
int right_light;

//Dot 8*8 matrix
#define SDA 21
#define SCL 22

ESP32_HT16K33 matrix = ESP32_HT16K33();
//The brightness values can be set from 1 to 15, with 1 darkest and 15 brightest
#define  A  10

//Matrix Number
byte numero0[8] = {0x00, 0x00, 0xff, 0x81, 0x81, 0xff, 0x00, 0x00};
byte numero1[8] = {0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00};
byte numero2[8] = {0x00, 0x00, 0xf9, 0x89, 0x89, 0x8f, 0x00, 0x00};
byte numero3[8] = {0x00, 0x00, 0xff, 0x89, 0x89, 0x89, 0x00, 0x00};
byte triste[8] = {0xe0, 0x20, 0x27, 0x20, 0x20, 0x27, 0x20, 0xe0};


//IR line tracking 
#define tracking_left  17  //set the pin of the left line tracking sensor to gpio17
#define tracking_right  16  //set the pin of the right line tracking sensor to gpio16
int L_val,R_val;//Define two variables

//-------------Wifi  ------------------------
//const char *ssid_Router     = "TalentGardenOpening"; //Enter the router name
//const char *password_Router = "Welcome2023!"; //Enter the router password 

void setup() {
  Serial.begin(115200);
  //Motor
  pinMode(left_ctrl,OUTPUT); //set control pins of the left motor to OUTPUT
  ledcAttach(left_pwm, 1200, 8); //Set the frequency of left_pwm pin to 1200, PWM resolution to 8 that duty cycle is 256.
  pinMode(right_ctrl,OUTPUT); //set direction control pins of the right motor to OUTPUT..
  ledcAttach(right_pwm, 1200, 8); //Set the frequency of right_pwm pin to 1200, PWM resolution to 8 that duty cycle is 256.
  //Servo
  pinMode(TRIG_PIN,OUTPUT); //set TRIG_PIN to OUTPUT.
  pinMode(ECHO_PIN,INPUT); //set ECHO_PIN to INPUT.
  myservo.setPeriodHertz(50);           // standard 50 hz servo
  myservo.attach(SERVO_PIN, 500, 2500);  // attaches the servo on servoPin to the servo object.
  myservo.write(90); //the initial angle of the servo is set to 90° .
  delay(500);

  //Photosensor
  //pinMode(PHOTOSENSITIVE_PIN, INPUT);//Configure the pins for input mode
  pinMode(light_L_Pin, INPUT); //set pins of the left sensor to INPUT
  pinMode(light_R_Pin, INPUT); //set pins of the right sensor to INPUT

  //Dot matrix
  matrix.init(0x70, SDA, SCL);//Initialize matrix
  matrix.showLedMatrix(numero0,0,0);
  matrix.show();
  delay(500);

  //Tracking
  pinMode(tracking_left, INPUT); //Set right pins of the left sensor to input
  pinMode(tracking_right, INPUT); //Set right pins of the right sensor to input
 
  //Timer
  startTime = millis();
  Serial.println("Timer avviato!");

}


void loop(){

  avoid();//obstacle avoidance

  if (millis() - startTime >= startTime+THREE_MINUTES) {
    Serial.println("Timer completato! Eseguito un'azione.");
    matrix.showLedMatrix(triste,0,0);
  }
  //TRACKING
  //tracking();
}

//AGGIORNA IL DISPLAY
void updateDisplay() {
    if (lightCount==1)
      matrix.showLedMatrix(numero1,0,0);
    else if (lightCount==2)
      matrix.showLedMatrix(numero2,0,0);
    else 
      matrix.showLedMatrix(numero3,0,0);
}

float checkdistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  float distance = pulseIn(ECHO_PIN, HIGH) / 58.00;
  delay(10);
  return distance;
}

void detectLights() {
    static unsigned long lastDetectionTime = 0;

    left_light = analogRead(light_L_Pin);//Read the value of the left photoresistor
    right_light = analogRead(light_R_Pin);//Read the value of the right photoresistor
    Serial.print("left_light_value = ");
    Serial.println(left_light);
    Serial.print("right_light_value = ");
    Serial.println(right_light);

    if (right_light>left_light+100 && right_light > LIGHT_THRESHOLD && (millis() - lastDetectionTime) > 500){
          lightCount++;
          lastDetectionTime = millis();
          Serial.printf("Light detected! Count: %d\n", lightCount);
          updateDisplay();
    }
}

void avoid(){
  float distance;
  distance = checkdistance(); //Get the value of ultrasonic distance
  Serial.printf("Distanza: %f\n", distance);
  delay(100);
  if((distance < OBSTACLE_DISTANCE) && (distance != 0)){//if 0<distance<10 c'è un ostacolo 
    //activateBuzzer(300);
    car_Stop();//stop
    float s=check_for_obstacle_left();
    float d=check_for_obstacle_right();
    delay(500);

    if (s>OBSTACLE_DISTANCE){ //Vai a sinistra se è libero
      Serial.print("Car left \n");
      car_left();
      delay(100);
      //car_back();
    }
    else if (d>OBSTACLE_DISTANCE) //Vai a destra se è libero
    {
      Serial.print("Car right \n");
      car_right();
      delay(100);
    }
    if (s<OBSTACLE_DISTANCE && d<OBSTACLE_DISTANCE)  { // Torna indietro
      car_left();
      delay(100);
      car_left();
    }
  }
  else{ // E libero
    Serial.print("NON HO OSTACOLI \n");
    car_front();//go forward  
    //Controllo la luce
    if(lightCount<=3){  
      detectLights();
      delay(500);
    }
  }
}

float check_for_obstacle_right() //define check for obstacle to the right of the vehicle
{
  float dist;
  myservo.write(20); //the initial angle of the servo is set to 90° .
  delay(500);
  dist=checkdistance();
  Serial.printf("Distanza DESTRA: %f\n", dist);
  myservo.write(90); // return to initial state
  delay(500);
  return dist;
  /*
  if((dist < OBSTACLE_DISTANCE)&&(dist != 0)){
    return true;
    }
  else {
    return false;
  }*/
}

float check_for_obstacle_left() //define check for obstacle to the right of the vehicle
{
  float dist;
  myservo.write(160); //the initial angle of the servo is set to 90° .
  delay(500);
  dist=checkdistance();
  Serial.printf("Distanza SINISTRA: %f\n", dist);
  myservo.write(90); // return to initial state
  delay(500);
  return dist;
  /*
  if((dist < OBSTACLE_DISTANCE)&&(dist != 0)){
    return true;
    }
  else {
    return false;
  }
  */
}

void car_front()//define the state of going forward
{
  Serial.print("AVANTI \n");
  digitalWrite(left_ctrl,LOW); // set direction control pins of the left motor to LOW.
  ledcWrite(left_pwm, speed); // the left motor outputs PWM 200
  digitalWrite(right_ctrl,LOW); // set control pins of the right motor to LOW.
  ledcWrite(right_pwm, speed+9); // the right motor outputs PWM 200
  delay(50);//delay in 2s
}
void car_back()//define the state of going back
{
  Serial.printf("INDIETRO \n");
  digitalWrite(left_ctrl,HIGH); //set direction control pins of the left motor to HIGH..
  ledcWrite(left_pwm, 50); //the left motor outputs PWM 100
  digitalWrite(right_ctrl,HIGH); //set control pins of the right motor to HIGH..
  ledcWrite(right_pwm, 50); //the right motor outputs PWM 100
}

void car_left()//define the state of turning left
{
  Serial.printf("SINISTRA \n");
  digitalWrite(left_ctrl,HIGH); //set direction control pins of the left motor to HIGH..
  ledcWrite(left_pwm, speed); //the left motor outputs PWM 100
  digitalWrite(right_ctrl,LOW); //set control pins of the right motor to LOW.
  ledcWrite(right_pwm, speed); //the right motor outputs PWM 100
  delay(300); // Wait for 500 milliseconds, adjust this for your desired turn time  
  // Stop the motors after the turn
  digitalWrite(left_ctrl, LOW);     // Stop left motor
  digitalWrite(right_ctrl, LOW);    // Stop right motor
}

void car_right()//define the state of turning right
{
  Serial.print("DESTRA \n");
  digitalWrite(left_ctrl,LOW); //set direction control pins of the left motor to LOW.
  ledcWrite(left_pwm, speed); //the left motor outputs PWM 100
  digitalWrite(right_ctrl,HIGH); //set control pins of the right motor to HIGH..
  ledcWrite(right_pwm, speed); //the right motor outputs PWM 100
  delay(300); // Wait for 500 milliseconds, adjust this for your desired turn time  
  // Stop the motors after the turn
  digitalWrite(left_ctrl, LOW);     // Stop left motor
  digitalWrite(right_ctrl, LOW);    // Stop right motor
}
void car_Stop()//define the state of stopping
{
  Serial.printf("STOP \n");
  digitalWrite(left_ctrl,LOW);// set direction control pins of the left motor to LOW.
  ledcWrite(left_pwm, 0); // the left motor outputs PWM 0.
  digitalWrite(right_ctrl,LOW);// set control pins of the right motor to LOW.
  ledcWrite(right_pwm, 0); // the right motor outputs PWM 0
  delay(200);//delay in 2s
}


//TRACKING
void tracking()
{
  L_val = digitalRead(tracking_left);//read the value of the left line tracking sensor
  R_val = digitalRead(tracking_right);//read the value of the right line tracking sensor
  Serial.printf("R_val");
  Serial.printf("L_val");

  if((L_val == 1)&&(R_val == 1))//if both of sensors detect black lines
  {
    car_front();//go forward
  }
  else if((L_val == 1)&&(R_val == 0))//if only the left sensor detects black lines
  {
    car_left();//turn left
  }
  else if((L_val == 0)&&(R_val == 1))//if only the right sensor detects black lines
  {
    car_right();//turn right
  }
  else//if none of sensors detects black lines
  {
    car_Stop();//stop
   }
}
