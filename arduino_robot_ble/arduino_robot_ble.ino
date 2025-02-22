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

 // pinMode (MOTOR_ENA, OUTPUT);
  //pinMode (MOTOR_ENB, OUTPUT);
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
   
  //Direccion motor A
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  //Direccion motor B
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);


}

void atras(void)
{
  Serial.println("atras");
    //Direccion motor A
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  //analogWrite(ENA, 50); //Velocidad motor A
  //Direccion motor B
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
 //analogWrite(ENB, 150); //Velocidad motor A
 
}

void derecha(void)
{
  Serial.println("derecha");
 
  //Direccion motor A
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  //analogWrite(ENA, 128); //Velocidad motor A
  
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
 
  //Direccion motor B
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);

}

void parar(void)
{
  Serial.println("parar");
 //Direccion motor A
 digitalWrite(MOTOR_IN1, LOW);
 digitalWrite(MOTOR_IN2, LOW);
 //analogWrite(ENA, 0); //Velocidad motor A
 //Direccion motor B
 digitalWrite(MOTOR_IN3, LOW);
 digitalWrite(MOTOR_IN4, LOW);
 //analogWrite(ENB, 0); //Velocidad motor A
}

