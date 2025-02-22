#include <SoftwareSerial.h>

# define FORDWARD    'F'
# define BACKWARD    'B'
# define LEFT        'L'
# define RIGHT       'R'
#define PAUSE        'P'
// Motor A
//#define MOTOR_ENA    10
#define MOTOR_IN1     8
#define MOTOR_IN2     9

// Motor B
#define MOTOR_IN3    10
#define MOTOR_IN4    11
//#define MOTOR_ENB     9

//BLE
#define BLE_rxPin     4
#define BLE_txPin     5
#define BLE_enablePin 2
#define BLE_VccPin    3


SoftwareSerial BT (BLE_rxPin,BLE_txPin); 
void izquierda(void);
void derecha(void);
void adelante(void);
void atras(void);

void setup() {

  pinMode (MOTOR_IN1, OUTPUT);
  pinMode (MOTOR_IN2, OUTPUT);
  pinMode (MOTOR_IN3, OUTPUT);
  pinMode (MOTOR_IN4, OUTPUT);
  pinMode(BLE_enablePin, OUTPUT);
  pinMode(BLE_VccPin, OUTPUT);

  // Enable BLE
  digitalWrite(BLE_enablePin, HIGH);
  delay(500);

  Serial.begin(9600);
  BT.begin(9600);

  // Power on BLE
  Serial.println("Levantando el modulo HC-05");
  digitalWrite(BLE_VccPin, HIGH);
}

void loop() {
  if(BT.available())
  {
    char cmd = BT.read();
    //Serial.write(cmd);
    switch(cmd)
    {
      case FORDWARD:
      {
        adelante();
      }
      break;
      case BACKWARD:
      {
        atras();
      }
      break;
      case LEFT:
      {
        izquierda();
      }
      break;
      case RIGHT:
      {
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

