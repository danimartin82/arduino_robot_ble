#include <SoftwareSerial.h>

// Motor A
#define MOTOR_ENA    10
#define MOTOR_IN1     2
#define MOTOR_IN2     3

// Motor B
#define MOTOR_IN3     5
#define MOTOR_IN4     6
#define MOTOR_ENB     9

//BLE
#define BLE_rxPin     4
#define BLE_txPin     5
#define BLE_enablePin 2
#define BLE_VccPin    3


SoftwareSerial BT (BLE_rxPin,BLE_txPin); 


void setup() {

  pinMode (MOTOR_ENA, OUTPUT);
  pinMode (MOTOR_ENB, OUTPUT);
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
  digitalWrite(VccPin, HIGH);
}

void loop() {
  if(BT.available())    // Si llega un dato por el puerto BT se envía al monitor serial
  {
    Serial.write(BT.read());
  }

    if(Serial.available())  // Si llega un dato por el monitor serial se envía al puerto BT
  {

    BT.write(Serial.read());
  }

}



void Adelante ()
{
 //Direccion motor A
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, LOW);
 analogWrite(ENA, 255); //Velocidad motor A
 //Direccion motor B
 digitalWrite(IN3, HIGH);
 digitalWrite(IN4, LOW);
 analogWrite(ENB, 255); //Velocidad motor B
}

void Atras ()
{
 //Direccion motor A
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, HIGH);
 analogWrite(ENA, 128); //Velocidad motor A
 //Direccion motor B
 digitalWrite(IN3, LOW);
 digitalWrite(IN4, HIGH);
 analogWrite(ENB, 128); //Velocidad motor B
}

void Derecha ()
{
 //Direccion motor A
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, LOW);
 analogWrite(ENA, 200); //Velocidad motor A
 //Direccion motor B
 digitalWrite(IN3, LOW);
 digitalWrite(IN4, HIGH);
 analogWrite(ENB, 100); //Velocidad motor A
}

void Izquierda ()
{
 //Direccion motor A
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, HIGH);
 analogWrite(ENA, 50); //Velocidad motor A
 //Direccion motor B
 digitalWrite(IN3, HIGH);
 digitalWrite(IN4, LOW);
 analogWrite(ENB, 150); //Velocidad motor A
}

void Parar ()
{
 //Direccion motor A
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, LOW);
 analogWrite(ENA, 0); //Velocidad motor A
 //Direccion motor B
 digitalWrite(IN3, LOW);
 digitalWrite(IN4, LOW);
 analogWrite(ENB, 0); //Velocidad motor A
}

