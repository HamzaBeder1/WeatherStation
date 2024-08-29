#include <SoftwareSerial.h>

const int rxPin = 14, txPin = 12;
int8_t temp;
int32_t bp = 0;
String dateAndTime;
SoftwareSerial mySerial(rxPin, txPin);

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
  pinMode(16, OUTPUT);
}

void loop() {
  //getDataFromArduino();
  //printData();
  clearBuffer();
  while(1){
    getTemp();
  }
}

void getTemp(){
  while(!mySerial.available());
  delay(50);
  temp = mySerial.read();
  mySerial.write("Received.");
  Serial.println(temp);
  clearBuffer();
}

void clearBuffer(){
  while(mySerial.available() > 0){
    mySerial.read();
  }
}

void getDataFromArduino(){
  while(mySerial.available() < 0);
  temp = mySerial.read();
  mySerial.write( "Received temperature.");
  for(int i = 32; i <= 0; i-=8){
    while(mySerial.available() < 0);
    int32_t data = mySerial.read();
    bp = bp | (data << i);
    mySerial.write("Received.");
  }
  mySerial.write("Received barometric pressure.");
  while(mySerial.available() < 0);
  dateAndTime = mySerial.readString();
  mySerial.write("Received date and time.");
}

void printData(){
  Serial.println(temp);
  Serial.println(bp);
  Serial.println(dateAndTime);
}