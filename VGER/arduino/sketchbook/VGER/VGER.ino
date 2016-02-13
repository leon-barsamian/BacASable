// debut de la configuration de l'ultrason
// # Connection:
// #       Pin 1 VCC (URM V3.2) -> VCC (Arduino)
// #       Pin 2 GND (URM V3.2) -> GND (Arduino)
// #       Pin 4 PWM (URM V3.2) -> Pin 3 (Arduino)
// #       Pin 6 COMP/TRIG (URM V3.2) -> Pin 5 (Arduino)
// # Pin mode: PWM
// # Working Mode: PWM passive control mode.
// # If it is your first time to use it,please make sure the two jumpers to the right hand
// # side of the device are set to TTL mode. You'll also find a secondary jumper on
// # the left hand side, you must break this connection or you may damage your device.

#include <Servo.h>                                  // Include Servo library
Servo myservo;                                      // create servo object to control a servo

int pos=0;                                          // variable to store the servo position
int URPWM=11;                                        // PWM Output 0-25000us,every 50us represent 1cm
int URTRIG=12;                                       // PWM trigger pin

int angleMin = 20;
int angleMax = 150;

boolean angleSuivant=true;                                    // create a boolean variable
unsigned long time;                                 // create a time variable
unsigned long urmTimer = 0;                          // timer for managing the sensor reading flash rate

unsigned int distanceResultat=0;
uint8_t EnPwmCmd[4]={0x44,0x22,0xbb,0x01};          // distance measure command

// Fin de la configuration de l'ultrason

//Standard PWM DC control
const int E1 = 5;     //M1 Speed Control
const int E2 = 6;     //M2 Speed Control
const int M1 = 4;    //M1 Direction Control
const int M2 = 7;    //M1 Direction Control

///For previous Romeo, please use these pins.
//int E1 = 6;     //M1 Speed Control
//int E2 = 9;     //M2 Speed Control
//int M1 = 7;    //M1 Direction Control
//int M2 = 8;    //M1 Direction Control

const int led = 13;
const String CMD_DIR_AVANCE ="a";
const String CMD_DIR_RECULE ="r";
const String CMD_DIR_TOURNE_DROITE ="td";
const String CMD_DIR_TOURNE_GAUCHE ="tg";
const String CMD_DIR_STOP ="s";
const String CMD_TEMP = "TMP";
const String CMD_SCAN = "SCAN";

char val;
String txtMsg = "";


/*****declaration capteurs IRs*****/
const int Infrared_left = 2;
     //Sensor IR gauche
const int Infrared_right = 3;
     //Sensor IR droite
int Infrared_left_state = 0; //declaration de variable
int Infrared_right_state = 0; //declaration de variable
/**********/

/*** capteur d'humidité et température ***/

#define DHT11_PIN 2      // ADC0

/*** ***/



void stop(void)                    //Stop
{
  digitalWrite(E1,LOW);
  digitalWrite(E2,LOW);
}
void advance(char a,char b)          //Move forward
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);
  digitalWrite(M2,HIGH);
}
void back_off (char a,char b)          //Move backward
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);
  analogWrite (E2,b);
  digitalWrite(M2,LOW);
}
void turn_L (char a,char b)             //Turn Left
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);
  analogWrite (E2,b);
  digitalWrite(M2,HIGH);
}
void turn_R (char a,char b)             //Turn Right
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);
  digitalWrite(M2,LOW);
}
void setup(void)
{
  Serial.begin(115200);      //Set Baud Rate
  Serial.println("Initialisation...");
  //int i;
  //for(i=4;i<=7;i++)
  //  pinMode(i, OUTPUT);

    
  // Initialisation des pins en input
  pinMode(Infrared_left, INPUT);
  pinMode(Infrared_right, INPUT);
  pinMode(led, OUTPUT);

  //configuration du capteur de temperature
  DDRC |= _BV(DHT11_PIN);
  PORTC |= _BV(DHT11_PIN);


  PWM_Mode_Setup();

  Serial.println("VGER operationel.");
}

void loop(void)
{

  verifierObstacle();
  while (Serial.available() > 0) {
    val = Serial.read();

    if (val == '\n') {


      if ( CMD_DIR_AVANCE.equals(txtMsg) ) {

        advance (100,100);

      } else if ( CMD_DIR_STOP.equals(txtMsg)) {

        stop();

      } else if ( CMD_DIR_RECULE.equals(txtMsg)) {
          back_off (100,100);
      } else if ( CMD_DIR_TOURNE_DROITE.equals(txtMsg)) {
          turn_R (100,100);
      } else if ( CMD_DIR_TOURNE_GAUCHE.equals(txtMsg)) {
          turn_L (100,100);
      } else if (CMD_TEMP.equals(txtMsg)) {
          releverTemperature();
      } else if (CMD_SCAN.equals(txtMsg)) {
          scanUltraSon();
      } else {
          stop();
      }

      txtMsg = "";
    }
    else {
      txtMsg +=val;
    }
  }
  //verifierObstacle();

}

void verifierObstacle(void) {
  if (digitalRead(Infrared_left) == LOW && digitalRead(Infrared_right) == LOW) {
        //back_off(255,255);
        //delay(1000);
     digitalWrite(led, HIGH);
  } else {
     digitalWrite(led, LOW);
  }
}



//specifique dht11
byte read_dht11_dat() {

  byte i = 0;
  byte result=0;

  for(i=0; i< 8; i++) {

    while(!(PINC & _BV(DHT11_PIN)));  // wait for 50us
    delayMicroseconds(30);

    if(PINC & _BV(DHT11_PIN))
      result |=(1<<(7-i));

    while((PINC & _BV(DHT11_PIN)));  // wait '1' finish

  }

  return result;

}

void releverTemperature() {



  byte dht11_dat[5];

  byte dht11_in;

  byte i;// start condition

	 // 1. pull-down i/o pin from 18ms

  PORTC &= ~_BV(DHT11_PIN);

  delay(18);

  PORTC |= _BV(DHT11_PIN);

  delayMicroseconds(40);

  DDRC &= ~_BV(DHT11_PIN);

  delayMicroseconds(40);



  dht11_in = PINC & _BV(DHT11_PIN);

  if(dht11_in)

  {

    Serial.println("dht11 start condition 1 not met");

    return;

  }

  delayMicroseconds(80);

  dht11_in = PINC & _BV(DHT11_PIN);

  if(!dht11_in)

  {

    Serial.println("dht11 start condition 2 not met");

    return;

  }



  delayMicroseconds(80);// now ready for data reception

  for (i=0; i<5; i++)

    dht11_dat[i] = read_dht11_dat();

  DDRC |= _BV(DHT11_PIN);

  PORTC |= _BV(DHT11_PIN);

  byte dht11_check_sum = dht11_dat[0]+dht11_dat[1]+dht11_dat[2]+dht11_dat[3];// check check_sum

  if(dht11_dat[4]!= dht11_check_sum)

  {

    Serial.println("DHT11 checksum error");

  }

  Serial.print("Current humdity = ");
  Serial.print(dht11_dat[0], DEC);
  Serial.print(".");
  Serial.print(dht11_dat[1], DEC);
  Serial.print("%  ");
  Serial.print("temperature = ");
  Serial.print(dht11_dat[2], DEC);
  Serial.print(".");
  Serial.print(dht11_dat[3], DEC);
  Serial.println("C  ");

}

//configuration UMR37
void PWM_Mode_Setup(){
  Serial.println("Initialisation ultra-son");
  pinMode(URTRIG,OUTPUT);                            // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG,HIGH);                         // Set to HIGH

    pinMode(URPWM, INPUT);                             // Sending Enable PWM mode command

  for(int i=0;i<4;i++){
    Serial.write(EnPwmCmd[i]);
  }
  Serial.println("");
  Serial.println("Initialisation ultra-son OK");
}


void scanUltraSon(){
  Serial.println("DEBUT_SCAN");
    myservo.attach(7);                                // Pin 9 to control servo
  for (int angle = angleMin ; angle < angleMax ; angle++) {
    myservo.write(angle);
    PWM_Mode();
    delay(100);
  }

  for (int angle = angleMax ; angle > angleMin ; angle--) {
    myservo.write(angle);
    PWM_Mode();
    delay(100);
  }
  myservo.detach();
  Serial.println("FIN_SCAN");
}


void PWM_Mode(){                                     // a low pull on pin COMP/TRIG  triggering a sensor reading
  digitalWrite(URTRIG, LOW);
  digitalWrite(URTRIG, HIGH);                      // reading Pin PWM will output pulses

  Serial.print(myservo.read());
  Serial.print("|");

  unsigned long distanceMesuree=pulseIn(URPWM,LOW);

  if(distanceMesuree==50000){                     // the reading is invalid.
    Serial.println("ERR");
  } else {
    distanceResultat = distanceMesuree/50;                  // every 50us low level stands for 1cm
    Serial.println(distanceResultat);
  }
  digitalWrite(URTRIG, LOW);
}

//fin configuration UMR37
