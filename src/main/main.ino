#include <Wire.h>

#define BMP180_ADDR 119
#define DS3231_ADDR 104

#define BMP180_CTRL_MEAS 0xF4
#define BMP180_OUT_MSB 0xF6

enum BMP180_REGISTERS : uint8_t{
  OUT_MSB = 0xF6,
  OUT_LSB = 0xF7,
  CTRL_MEAS = 0xF4
};

void setup() {
  DDRD |= 0b00000010; //TX = output
  DDRD &= 0b11111110; //RX = input
  DHT11init();
  Serial.begin(9600);
  Wire.begin();
}

class DS3231{
  private:
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint16_t year;

  public:
    void initDS3231(uint16_t, uint8_t *);
    void getDataDS3231();
    void hourModeSelect(int h);
    void printDS3231();
};

class BMP180{
  private:
    int16_t bmpAC1;
    int16_t bmpAC2;
    int16_t bmpAC3;
    uint16_t bmpAC4;
    uint16_t bmpAC5;
    uint16_t bmpAC6;
    int16_t bmpB1;
    int16_t bmpB2;
    int16_t bmpMB;
    int16_t bmpMC;
    int16_t bmpMD;
    int32_t bmpUT;
    int32_t bmpUP;
    int32_t bmpT;
    int32_t bmpP;
  public:
    void getCalibrationData();
    void getUT();
    uint8_t readRegisterBMP180(uint8_t reg);
    void writeRegisterBMP180(uint8_t , uint8_t);
};

void loop() {
  BMP180 a;
  delay(3000);
  a.getCalibrationData();
  while(1){

  }
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

unsigned char * readRegister_BMP180(unsigned char * data, int size){
  Wire.beginTransmission(BMP180_ADDR); 
  Wire.write(data[0]);
  Wire.endTransmission(BMP180_ADDR);

  Wire.requestFrom(BMP180_ADDR, size);
  int i;
  while(Wire.available() != size);
  for(i = 0; i < size; i++)
  { 
     data[i] = Wire.read();
  }
  return data;
}

void writeRegister_BMP180(unsigned char * data, int size){
  Wire.beginTransmission(BMP180_ADDR); 
  Wire.write(data, size);
  Wire.endTransmission(BMP180_ADDR);
}

void DS3231::getDataDS3231(){
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(DS3231_ADDR);

  Wire.requestFrom(DS3231_ADDR, 7);
  //while(Wire.available() != 7);

  this->second = bcd2dec(Wire.read());
  this->minute = bcd2dec(Wire.read());
  uint8_t temp = Wire.read();
  temp &= 0b10111111;
  temp = bcd2dec(temp);
  this->hour = temp; 
  this->day = bcd2dec(Wire.read());
  this->date=  bcd2dec(Wire.read());
  this->month = bcd2dec(Wire.read());
  this->year = 2000 + bcd2dec(Wire.read());

}

void DS3231:: initDS3231(uint16_t y, uint8_t * arr){
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x00); //go to first address for multibyte access.
  for(int i = 0; i < 6; i++){
    Wire.write(dec2bcd(arr[i]));
  }
  Wire.write(dec2bcd(y));
  Wire.endTransmission(BMP180_ADDR);
  hourModeSelect(12);
}

void DS3231::hourModeSelect(int h){
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x02);
  Wire.endTransmission(DS3231_ADDR);
  Wire.requestFrom(DS3231_ADDR, 1);
  uint8_t reg = Wire.read();
  if(h == 12)
    reg |= 0b01000000;
  else if(h == 24)
    reg &= 0b10111111;
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x02);
  Wire.write(reg);
  Wire.endTransmission(DS3231_ADDR);
}

void DS3231::printDS3231(){
  char buffer[50];
  sprintf(buffer, "%d-%d-%d %d:%d:%d", year,month, date, hour, minute, second);
  Serial.println(buffer);
}

void BMP180::getCalibrationData(){
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xAA);
  Wire.endTransmission(BMP180_ADDR);

  Wire.requestFrom(BMP180_ADDR, 22);
  bmpAC1 = Wire.read() << 8;
  bmpAC1 |= Wire.read();
  bmpAC2 = Wire.read() << 8;
  bmpAC2 |= Wire.read();
  bmpAC3 = Wire.read() << 8;
  bmpAC3 |= Wire.read();
  bmpAC4 = Wire.read() << 8;
  bmpAC4 |= Wire.read();
  bmpAC5 = Wire.read() << 8;
  bmpAC5 |= Wire.read();
  bmpAC6 = Wire.read() << 8;
  bmpAC6 |= Wire.read();
  bmpB1 = Wire.read() << 8;
  bmpB1 |= Wire.read();
  bmpB2 = Wire.read() << 8;
  bmpB2 |= Wire.read();
  bmpMB = Wire.read() << 8;
  bmpMB |= Wire.read();
  bmpMC = Wire.read() << 8; 
  bmpMC |= Wire.read();
  bmpMD = Wire.read() << 8;
  bmpMD |= Wire.read();
}

void BMP180:: getUT(){
  writeRegisterBMP180(0xF4, 0x2E);
  delay(5);
  bmpUT = readRegisterBMP180(0xF6) << 8;
  bmpUT |= readRegisterBMP180(0xF7);
}

uint8_t readRegisterBMP180(uint8_t reg){
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(reg);
  Wire.endTransmission(BMP180_ADDR);

  Wire.requestFrom(BMP180_ADDR, 1);
  uint8_t result = Wire.read();
  return result;
}

void BMP180:: writeRegisterBMP180(uint8_t reg, uint8_t val){
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(BMP180_ADDR);
}


uint8_t bcd2dec(uint8_t bcd){
  return bcd/16*10 + bcd%16;
}

uint8_t dec2bcd(uint8_t dec){
  return dec/10*16 + dec%10;
}


