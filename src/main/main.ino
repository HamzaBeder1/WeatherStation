#include <Wire.h>

#define BMP180_ADDR 119
void setup() {
  DDRD |= 0b00000010; //TX = output
  DDRD &= 0b11111110; //RX = input
  DHT11init();
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  /*getData();
  delay(5000);
  DDRD |= 0b00010000;*/
  Serial.println(readRegister_BMP180(244));
  delay(1000);
}

void DHT11init(){
  DDRD |= 0b00010000; //DHT11OUT = output
  PORTD |= 0b00010000; //DHT11OUT = HIGH.
}

void getData(){
  PORTD &= 0b11101111;
  delay(18); //18ms MCU start signal.
  PORTD |= 0b00010000; 
  DDRD &= 0b11101111; //turn signal pin to input in order to read data.
  while(bitRead(PIND, 4)); //wait for DHT response.
  while(!bitRead(PIND, 4)); //DHT response.
  while(bitRead(PIND, 4)); //DHT makes line HIGH.
  unsigned char * b = readByte();
  for(int i = 0; i < 5; i++){
    for(int j = 0; j < 8; j++)
      Serial.print(bitRead(b[i], j));
    Serial.println();
  }
  
  //delayMicroseconds(20);
  DDRD &= 0b11101111;
}

unsigned char * readByte(){
  unsigned char result[5] = {0,0,0,0,0};
  for(int i = 0; i < 40; i++){
    result[i/5] <<= 1;
    while(!bitRead(PIND,4)); //wait for 50us low signal.
    delayMicroseconds(30);
    if(bitRead(PIND, 4))//if still HIGH, then a 1 was transmitted.
      result[i/5] |= 1; //Take no action if a 0 was transmitted.
    while(bitRead(PIND,4)); //wait for the bit to complete transmission.
  }
  return result;
}

unsigned char readRegister_BMP180(uint8_t addr){
  Wire.beginTransmission(BMP180_ADDR); //begin communication with BMP180.
  Wire.write(addr);
  Wire.endTransmission(BMP180_ADDR);
  Wire.requestFrom(BMP180_ADDR, 1);
  unsigned char a;
  while(Wire.available()){
     a = Wire.read();
  }
  return a;
}

void writeRegiste_BMP180(uint8_t addr, uint8_t data){
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission(BMP180_ADDR);
}
