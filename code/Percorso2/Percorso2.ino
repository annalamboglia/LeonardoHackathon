/*
Project 07: follow me
Car follows the object
*/
#include <ESP32Servo.h>
//motor
#define left_ctrl  33  //define direction control pins of the left motor as gpio33
#define left_pwm  26   //define PWM control pins of the left motor as gpio26.
#define right_ctrl  32 //define direction control pins of the right motor as gpio32.
#define right_pwm  25  //define PWM control pins of the right motor as gpio25

//ultrasonic sensor
#define TRIG_PIN 5 // set signals input of the ultrasonic sensor to gpio5.
#define ECHO_PIN 18 //set signals output of the ultrasonic sensor to gpio18.

//photoresistors
#define light_L_Pin  34   //define the pins of the left photoresistor as gpio34
#define light_R_Pin  35   //define the pins of the right photoresistor as gpio35

#define SDA 21
#define SCL 22

int offset_dx=25;
int left_light; 
int right_light;
long dist_ostacolo=10;
long distance,a1,a2;//define three distance variables

// Posizioni del servo
const int CENTER_POS = 90;
const int RIGHT_POS = 30;
const int LEFT_POS = 150;

//servo
const int servoPin = 4;//set the pin of the servo to gpio4.
const int servoPinPinza = 23;//set the pin of the servo to gpio4.

Servo myservo;  // create servo object to control a servo
Servo servoPinza;  // create servo object to control a servo


int luci_accese=0;
// Costanti da calibrare in base alla tua macchina
const int velocita_cm_al_sec = 10;  // Quanti cm percorre al secondo
const int pwm = 150;

void setup() {
  pinMode(left_ctrl, OUTPUT); //set control pins of the left motor to OUTPUT
  ledcAttach(left_pwm, 1200, 8); //Set the frequency of left_pwm pin to 1200, PWM resolution to 8 that duty cycle is 256.
  pinMode(right_ctrl,OUTPUT);//set direction control pins of the right motor to OUTPUT..
  ledcAttach(right_pwm, 1200, 8); //Set the frequency of right_pwm pin to 1200, PWM resolution to 8 that duty cycle is 256.
  
  pinMode(TRIG_PIN,OUTPUT);//set TRIG_PIN to OUTPUT.
  pinMode(ECHO_PIN,INPUT);//set ECHO_PIN to INPUT.
//  pinMode(PHOTOSENSITIVE_PIN, INPUT);//Configure the pins for input mode
  pinMode(light_L_Pin, INPUT); //set pins of the left sensor to INPUT
  pinMode(light_R_Pin, INPUT); //set pins of the right sensor to INPUT

  myservo.setPeriodHertz(50);           // standard 50 hz servo
  myservo.attach(servoPin, 500, 2500);  // attaches the servo on servoPin to the servo object.
  myservo.write(90);  // the initial angle of the servo is set to 90° .

  delay(500);
  Serial.begin(115200);

  servoPinza.setPeriodHertz(50);           // standard 50 hz servo
  servoPinza.attach(servoPinPinza, 500, 2500);  // attaches the servo on servoPin to the servo object.
  servoPinza.write(120);  // the initial angle of the servo is set to 180° .

  Serial.print("pinza");
  delay(1000);


    //task1
  
  muoviAvantiCm(30);
  gira90GradiSinistra();
  
  delay(1000);
  muoviAvantiCm(25);
  
  apriPinza(40);
  delay(1000);

  muoviAvantiCm(5);
  delay(500);
  gira180Gradi();
  delay(500);
  apriPinza(120);
  //percorri_tunnel();
  
  //TODO: accendi display con il numero di luci corrette: luci_accese
  //PC: parametro da calibrare
}

// La direzione iniziale in uscita dal tunnel è "front" =0, cioè il robot è rivolto verso l'uscita.
// Per ruotare il robot di 90°:
// - Girando a sinistra (in senso antiorario), aggiorniamo la direzione con dir = (dir + 1) % 4.
// - Girando a destra (in senso orario), aggiorniamo la direzione con dir = (dir + 3) % 4,
//   che è equivalente a (dir - 1), ma evita i numeri negativi grazie al modulo 4.

// La mappatura delle direzioni numeriche è la seguente:
// front  = 0    // Il robot è rivolto verso l'uscita (direzione iniziale quando esce dal tunnel).
// left   = 1    // Il robot è rivolto a sinistra (90° in senso antiorario).
// back   = 2    // Il robot è rivolto dietro (180° dalla direzione iniziale).
// right  = 3    // Il robot è rivolto a destra (90° in senso orario).

int direzione=0;


void loop()
{
  //dovrei già essere uscito dal tunnel quindi sono in direzione di uscita
  //cioè direzione=0

  print_info();

  //avoid();//obstacle avoidance
  //camminaCosteggiandoDestra();
  delay(500);
}

void muoviAvantiCm(int cm) {

  int durata = (cm * 1000) / velocita_cm_al_sec;  // durata in ms
  Serial.print("Muovo avanti per ");
  Serial.println(" cm");

  // Vai avanti
  digitalWrite(left_ctrl, LOW);  
  digitalWrite(right_ctrl, LOW); 
  ledcWrite(left_pwm, pwm);
  ledcWrite(right_pwm, pwm);

  delay(durata);  // Aspetta finché ha percorso i cm stimati

  // Ferma
  ledcWrite(left_pwm, 0);
  ledcWrite(right_pwm, 0);
}


// Funzione per aprire la pinza
void apriPinza(int angolo) {
  servoPinza.write(angolo); // Modifica l’angolo se necessario
  Serial.printf("Pinza aperta: %d",angolo);
}


void camminaCosteggiandoDestra() {
  int dir = 0;  // Iniziamo in "front" = 0
  
  while(true) {
    // Misura la distanza dall'ostacolo davanti
    distance = checkdistance();
    
    // Se non c'è un ostacolo davanti, continua a muoversi in avanti
    if (distance > 10 ) {
      car_front();
    } 
    // Se c'è un ostacolo davanti, prova a girare
    else {

    }
    // Aggiungi una pausa per evitare loop troppo veloci
    delay(200);
  }
}

void percorri_tunnel()
{
    //tunnel iniziale
  car_front();
  luci_accese=countLight();
  car_Stop();
  delay(500);
  car_left();
  delay(500);
  car_Stop();

}

int countLight()
{
  int luci=0;
  distance = checkdistance();
  Serial.printf("Distanza: %d\n", distance);
  while(distance>15)//
  {
    right_light = analogRead(light_R_Pin);//Read the value of the right photoresistor
    if(right_light>500){//PC
      luci++;
      delay(1000);//aspetta un secondo leggere la prossima. PC
    }
    distance = checkdistance();
  }
  if(luci>3)
    luci=3;
  //turn left
  //Serial.printf("luci accese rilevate: %d\n", luci);
  return luci; 
}
// Misura distanza verso destra
long checkdistanceRight() {
  myservo.write(RIGHT_POS);  // Gira verso destra
  delay(300);
  long dist = checkdistance();
  myservo.write(CENTER_POS);  // Torna al centro
  delay(300);
  return dist;
}

// Misura distanza verso sinistra
long checkdistanceLeft() {
  myservo.write(LEFT_POS);  // Gira verso sinistra
  delay(300);
  long dist = checkdistance();
  myservo.write(CENTER_POS);  // Torna al centro
  delay(300);
  return dist;
}
void print_info()
{
  distance = checkdistance();//Get the distance measured by the ultrasonic sensor
  Serial.printf("%dcm\n", distance);  // Stampa distanza con "cm" e va a capo
  right_light = analogRead(light_R_Pin);  // Legge il sensore
  Serial.printf("photosensitiveADC right: %d\n", right_light);  // Stampa il valore e aggiunge una riga vuota
  Serial.printf("luci accese: %d\n", luci_accese);  // Stampa il valore e aggiunge una riga vuota
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

void car_front()//define the state of going forward
{
  digitalWrite(left_ctrl,LOW); //set direction control pins of the left motor to LOW.
  ledcWrite(left_pwm, 100); //the left motor outputs PWM 100
  digitalWrite(right_ctrl,LOW); //set control pins of the right motor to LOW.
  ledcWrite(right_pwm, 100+offset_dx); //the right motor outputs PWM 100
}
void car_back()//define the state of going back
{
  digitalWrite(left_ctrl,HIGH); //set direction control pins of the left motor to HIGH..
  ledcWrite(left_pwm, 100); //the left motor outputs PWM 100
  digitalWrite(right_ctrl,HIGH); //set control pins of the right motor to HIGH..
  ledcWrite(right_pwm, 100); //the right motor outputs PWM 100
}
void car_left()//define the state of turning left
{
  digitalWrite(left_ctrl,HIGH); //set direction control pins of the left motor to HIGH..
  ledcWrite(left_pwm, 100); //the left motor outputs PWM 100
  digitalWrite(right_ctrl,LOW); //set control pins of the right motor to LOW.
  ledcWrite(right_pwm, 100); //the right motor outputs PWM 100
}

void gira90GradiSinistra() {
  // Ruotiamo di 90 gradi
  // Supponiamo che la rotazione di 90 gradi duri Y millisecondi
  int durata_rotazione = 500;  // Modifica questo valore in base alla tua velocità di rotazione
  
  // Imposta i motori per girare in direzioni opposte (per rotazione)
  digitalWrite(left_ctrl, HIGH);  // Imposta il motore sinistro per girare in avanti
  ledcWrite(left_pwm, 100);       // Imposta PWM per il motore sinistro
  digitalWrite(right_ctrl, HIGH); // Imposta il motore destro per girare all'indietro
  ledcWrite(right_pwm, 100);      // Imposta PWM per il motore destro
  
  delay(durata_rotazione);  // Mantieni la rotazione per il tempo necessario per girare 90 gradi

  // Ferma i motori dopo la rotazione
  ledcWrite(left_pwm, 0);
  ledcWrite(right_pwm, 0);
}
void gira90GradiDestra() {
  // Ruotiamo di 90 gradi a destra
  // Supponiamo che la rotazione di 90 gradi duri Y millisecondi
  int durata_rotazione = 500;  // Modifica questo valore in base alla tua velocità di rotazione
  
  // Imposta i motori per girare in direzioni opposte per ruotare a destra
  digitalWrite(left_ctrl, LOW);  // Imposta il motore sinistro per girare all'indietro
  ledcWrite(left_pwm, 100);      // Imposta PWM per il motore sinistro
  digitalWrite(right_ctrl, HIGH); // Imposta il motore destro per girare in avanti
  ledcWrite(right_pwm, 100);     // Imposta PWM per il motore destro
  
  delay(durata_rotazione);  // Mantieni la rotazione per il tempo necessario per girare 90 gradi

  // Ferma i motori dopo la rotazione
  ledcWrite(left_pwm, 0);
  ledcWrite(right_pwm, 0);
}


void gira180Gradi() {
  // Ruotiamo di 180 gradi
  // Supponiamo che la rotazione di 180 gradi duri X millisecondi
  int durata_rotazione = 1000;  // Modifica questo valore in base alla tua velocità di rotazione
  
  // Imposta i motori per girare in direzioni opposte (per rotazione)
  digitalWrite(left_ctrl, HIGH);  // Imposta il motore sinistro per girare in avanti
  ledcWrite(left_pwm, 100);       // Imposta PWM per il motore sinistro
  digitalWrite(right_ctrl, HIGH); // Imposta il motore destro per girare all'indietro
  ledcWrite(right_pwm, 100);      // Imposta PWM per il motore destro
  
  delay(durata_rotazione);  // Mantieni la rotazione per il tempo necessario per girare 180 gradi

  // Ferma i motori dopo la rotazione
  ledcWrite(left_pwm, 0);
  ledcWrite(right_pwm, 0);
}

void car_right()//define the state of turning right
{
  digitalWrite(left_ctrl,LOW); //set direction control pins of the left motor to LOW.
  ledcWrite(left_pwm, 100); //the left motor outputs PWM 100
  digitalWrite(right_ctrl,HIGH); //set control pins of the right motor to HIGH..
  ledcWrite(right_pwm, 100); //the right motor outputs PWM 100
}
void car_Stop()//define the state of stopping
{
  digitalWrite(left_ctrl,LOW);//set direction control pins of the left motor to LOW.
  ledcWrite(left_pwm, 0); //the left motor outputs PWM 0 
  digitalWrite(right_ctrl,LOW);//set control pins of the right motor to LOW.
  ledcWrite(right_pwm, 0); //the right motor outputs PWM 0
} 