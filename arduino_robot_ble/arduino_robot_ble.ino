#include <SoftwareSerial.h>


#define rxPin 4
#define txPin 5
#define enablePin 2
#define VccPin 3
SoftwareSerial BT (rxPin,txPin); 


void setup() {


    pinMode(enablePin, OUTPUT);
    pinMode(VccPin, OUTPUT);
    digitalWrite(enablePin, HIGH);
    delay(500);

    Serial.begin(9600);
    BT.begin(9600);
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
