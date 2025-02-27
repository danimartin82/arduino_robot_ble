/******************************************************************************
* L298N motor control
* IN1 -> PIN  8
* IN2 -> PIN  9
* IN3 -> PIN 10
* IN4 -> PIN 11
*
*  HC-05 bluetooth interface
*  RX -> PIN 4
*  TX -> PIN 5
*  EN -> PIN 2
* 
* Ultrasonidos
* Echo -> PIN 12
* Trigger -> PIN 13
*
* Mp3 player
* MP3_RX -> Pin 6
* MP3_TX -> Pin 7

*******************************************************************************/

#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

// BLE Commands
# define FORDWARD    'F'
# define BACKWARD    'B'
# define LEFT        'L'
# define RIGHT       'R'
#define PAUSE        'P'

// Motor A
#define MOTOR_IN1     8
#define MOTOR_IN2     9

// Motor B
#define MOTOR_IN3    10
#define MOTOR_IN4    11

//BLE
#define BLE_rxPin     4
#define BLE_txPin     5
#define BLE_enablePin 2

// Ultrasonidos
#define EchoPin      12
#define TriggerPin   13

// MP3
#define MP3_RX       6
#define MP3_TX       7



void izquierda(void);
void derecha(void);
void adelante(void);
void atras(void);
void printDetail(uint8_t type, int value);


SoftwareSerial DFPlayerSerial(MP3_RX, MP3_TX);
SoftwareSerial BT (BLE_rxPin,BLE_txPin); 
DFRobotDFPlayerMini myDFPlayer;
char cmd;
char direcion;
int distancia;
bool audio_flag;



void setup() {

  pinMode (MOTOR_IN1, OUTPUT);
  pinMode (MOTOR_IN2, OUTPUT);
  pinMode (MOTOR_IN3, OUTPUT);
  pinMode (MOTOR_IN4, OUTPUT);
  pinMode(BLE_enablePin, OUTPUT);

  Serial.begin(9600);
  BT.begin(9600);
  DFPlayerSerial.begin(9600);

  // Enable BLE
  Serial.println("Levantando el modulo HC-05");
  digitalWrite(BLE_enablePin, HIGH);
  delay(500);



 if (!myDFPlayer.begin(DFPlayerSerial,true,true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
  }
  Serial.println(F("DFPlayer Mini online."));
  
  //myDFPlayer.volume(10);  //Set volume value. From 0 to 30

  myDFPlayer.play(1);  //Play the first mp3
  // Only one software serial can listen at a time
  BT.listen();


}

void loop() {


  distancia = ping();

  if (((direcion == FORDWARD) || (direcion == LEFT) || (direcion == RIGHT)) && (distancia < 20))
  {
    parar();
    //DFPlayerSerial.listen();
    if(audio_flag == false)
    {
      audio_flag = true;
      myDFPlayer.play(1);
    }
    
  }
  else
  {
    audio_flag = false;
  }

  BT.listen();
  if(BT.available())
  {
    cmd = BT.read();
    

    switch(cmd)
    {
      case FORDWARD:
      {
        if (distancia > 20)
        {
          direcion = cmd;
          adelante();
        }
        else
        {
          direcion = cmd;
          parar();
        }

      }
      break;
      case BACKWARD:
      {
        direcion = cmd;
        atras();
      }
      break;
      case LEFT:
      {
        direcion = cmd;
        izquierda();
      }
      break;
      case RIGHT:
      {
        direcion = cmd;
        derecha();
      }
      break;
      case PAUSE:
      {
        parar();
      }
      break;
      
    }
  }

    if(Serial.available())
  {

    BT.write(Serial.read());
  }


  delay(200);

}



void adelante(void)
{
  Serial.println("adelante");
  
  // Adelante motor A
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);

  // Adelante motor B
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);


}

void atras(void)
{
  Serial.println("atras");

  // Atras motor A
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);

  // Atras motor B
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
 
}

void derecha(void)
{
  Serial.println("derecha");
 
  //Adelante motor A
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  
  //Paramos motor B
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
 
}

void izquierda(void)
{
   Serial.println("izquierda");

  //Paramos motor A
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
 
  //Adelante motor B
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);

}

void parar(void)
{
  Serial.println("parar");

 //Paramos motor A
 digitalWrite(MOTOR_IN1, LOW);
 digitalWrite(MOTOR_IN2, LOW);

 //Paramos motor B
 digitalWrite(MOTOR_IN3, LOW);
 digitalWrite(MOTOR_IN4, LOW);

}



int ping(void) {
  long duration, distanceCm;
  
  digitalWrite(TriggerPin, LOW);  //para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds(4);
  digitalWrite(TriggerPin, HIGH);  //generamos Trigger (disparo) de 10us
  delayMicroseconds(10);
  digitalWrite(TriggerPin, LOW);
  
  duration = pulseIn(EchoPin, HIGH);  //medimos el tiempo entre pulsos, en microsegundos
  
  distanceCm = duration * 10 / 292/ 2;   //convertimos a distancia, en cm
  return distanceCm;
}


void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }

}
