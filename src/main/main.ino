#include <Wire.h>
#include <SoftwareSerial.h>

#define BMP180_ADDR 119
#define DS3231_ADDR 104

#define BMP180_CTRL_MEAS 0xF4
#define BMP180_OUT_MSB 0xF6
#define OSS_BMP180 0b00
#define CLK_RATE_SPI 14000000

const int rxPin = 13, txPin = 12;

enum BMP180_REGISTERS : uint8_t{
  OUT_MSB = 0xF6,
  OUT_LSB = 0xF7,
  CTRL_MEAS = 0xF4
};

SoftwareSerial mySerial(rxPin, txPin);
void setup() {
  DDRD |= 0b00000010; //TX = output
  DDRD &= 0b11111110; //RX = input
  DHT11init();
  Serial.begin(9600);
  mySerial.begin(9600);
  mySerial.listen();
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
    String getDateAndTime();
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
    int32_t bmpB5;
    int32_t bmpT;
    int32_t bmpP;
  public:
    void getCalibrationData();
    void getbmpUT();
    void getbmpUP();
    uint8_t readRegisterBMP180(uint8_t reg);
    void writeRegisterBMP180(uint8_t , uint8_t);
    void getbmpT();
    void getbmpB5();
    void getbmpP();
    int8_t getT();
    int32_t getP();

    void printCalibrationData();
    void printUT();
    void printUP();
    void printB5();
    void printT();
    void printP();
};

void loop() {
  /*BMP180 b;
  DS3231 d;
  b.getbmpT();
  b.getbmpP();
  d.getDataDS3231();
  b.printT();
  b.printP();
  d.printDS3231();
  Serial.println(b.getT());
  Serial.println(b.getP());
  Serial.println(d.getDateAndTime());
  int result = sendDataESP8266(b.getT(), b.getP(),d.getDateAndTime());
  delay(1000);*/
  clearBuffer();
  while(1){
    int x = sendTemp(50);
    Serial.println(x);
    delay(1000);
  }
}

int sendTemp(int8_t temp){
  mySerial.write(temp);
  while(!mySerial.available());
  delay(50);
  if(mySerial.readString() != "Received.")
    return -1;
  clearBuffer();
  return 0;
}

void clearBuffer(){
  while(mySerial.available() > 0){
    mySerial.read();
  }
}


int sendDataESP8266(int8_t temp, int32_t bp, String dateAndTime){
  mySerial.write(temp);
  while(mySerial.available() < 0);
  if(mySerial.readString() != "Received temperature.")
    return -1;
  int8_t dummy = bp;
  for(int i = 24; i >= 0; i-=8){
    mySerial.write((int8_t)(dummy >> i));
    while(mySerial.available() < 0);
    if(mySerial.readString() != "Received.")
      return -1;
  }
  while(mySerial.available() < 0);
  if(mySerial.readString() != "Received barometric pressure.")
    return -1;
  mySerial.print(dateAndTime);
  while(mySerial.available() < 0);
  if(mySerial.readString() != "Received date and time.")
    return -1;
  return 0;
}

void getDataESP8266(uint8_t data){
  String result;
  if(mySerial.available() > 0){
    while(mySerial.available()){
      result += mySerial.read();
    }
  }
  if(result == "Received")
    Serial.println("Success!");
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

String DS3231::getDateAndTime(){
  char buffer[50];
  sprintf(buffer, "%d-%d-%d %d:%d:%d", year,month, date, hour, minute, second);
  return buffer;
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

void BMP180:: getbmpUT(){
  writeRegisterBMP180(BMP180_CTRL_MEAS, 0x2E);
  delay(5);
  bmpUT = readRegisterBMP180(0xF6) << 8;
  bmpUT |= readRegisterBMP180(0xF7);
  //Serial.println(bmpUT);
}

void BMP180:: getbmpUP(){
  writeRegisterBMP180(BMP180_CTRL_MEAS, 0x34);
  delay(5);
  int32_t result = (int32_t)readRegisterBMP180(0xF6) << 16;
  result |= (int32_t)readRegisterBMP180(0xF7) << 8;
  result |= (int32_t)readRegisterBMP180(0xF8);
  bmpUP = int32_t(result >> (8-OSS_BMP180));
  //Serial.println(bmpUP);
}

void BMP180:: getbmpT(){
  getCalibrationData();
  getbmpUT();
  int32_t X1 = ((bmpUT - (int32_t)bmpAC6)*(int32_t)bmpAC5) >> 15;
  int32_t X2 = ((int32_t)bmpMC<<11)/(X1+(int32_t)bmpMD);
  int32_t B5 = X1 + X2;
  bmpT = (B5+8) >> 4;
  bmpT /= 10;
  bmpT = (9.0/5.0)*(bmpT)+32;
}

void BMP180:: getbmpB5(){
  getbmpUT();
  int32_t X1 = ((bmpUT - (int32_t)bmpAC6)*(int32_t)bmpAC5) >> 15;
  int32_t X2 = ((int32_t)bmpMC<<11)/(X1+(int32_t)bmpMD);
  bmpB5 = X1 + X2;
}

void BMP180:: getbmpP(){
  getCalibrationData();
  getbmpB5();
  getbmpUP();
  //Serial.println("Start");
  int32_t bmpB6 = bmpB5 - 4000;
  //Serial.println(bmpB6);
  int32_t X1 = ((int32_t)bmpB2*((bmpB6*bmpB6) >> 12))>>11;
  //Serial.println(X1);
  int32_t X2 = ((int32_t)bmpAC2*bmpB6)>>11;
  //Serial.println(X2);
  int32_t X3 = X1+X2;
  //Serial.println(X3);
  int32_t bmpB3 = ((((int32_t)bmpAC1*4+X3)<<OSS_BMP180)+2) >> 2;
  //Serial.println(bmpB3);
  X1 = ((int32_t)bmpAC3*bmpB6)>>13;
  //Serial.println(X1);
  X2 = ((int32_t)bmpB1 * ((bmpB6*bmpB6)>>12))>>16;
  //Serial.println(X2);
  X3 = ((X1+X2)+2)>>2;
  //Serial.println(X3);
  int32_t bmpB4 = ((uint32_t)bmpAC4*(X3+32768L))>>15;
  //Serial.println(bmpB4);
  int32_t bmpB7 = (bmpUP - bmpB3)*(50000UL >>OSS_BMP180);
  //Serial.println(bmpB7);

  if(bmpB7 < 0x80000000){
    bmpP = (bmpB7/bmpB4)*2;
    //Serial.println("here");
  }
  else{
    bmpP = (bmpB7/bmpB4)<<1;
    //Serial.println("eh? here");
  }
    
  //Serial.println(bmpP);
  X1 = (bmpP>>8)*(bmpP>>8);
  //Serial.println(X1);
  X1 = (X1*3038L) >> 16;
  //Serial.println(X1);
  X2 = (-7357L*bmpP) >> 16;
  //Serial.println(X2);
  bmpP = bmpP + ((X1+X2+3791L)>>4);
  //Serial.println(X1+X2+3971L);
  //Serial.println(bmpP);
  //Serial.println("Stop");
}

uint8_t BMP180:: readRegisterBMP180(uint8_t reg){
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

void BMP180::printCalibrationData(){
  char buffer[100];
  sprintf(buffer, "AC1:%d, AC2:%d, AC3:%d, AC4:%u, AC5:%u, AC6:%u, B1:%d, B2:%d, MB:%d, MC:%d, MD:%d", bmpAC1, bmpAC2, bmpAC3, bmpAC4, bmpAC5, bmpAC6, bmpB1, bmpB2, bmpMB, bmpMC, bmpMD);
  Serial.println(buffer);
}

void BMP180::printUT(){
  char buffer[50];
  sprintf(buffer, "UT:%d", bmpUT);
  Serial.println(buffer);
}

void BMP180::printUP(){
  char buffer[50];
  sprintf(buffer, "UP:%"PRId32, bmpUP);
  Serial.println(buffer);
}

void BMP180::printT(){
  char buffer[50];
  sprintf(buffer, "T:%d", bmpT);
  Serial.println(buffer);
}

void BMP180::printB5()
{
  char buffer[50];
  sprintf(buffer, "B5:%d", bmpB5);
  Serial.println(buffer);
}

void BMP180::printP(){
  char buffer[50];
  sprintf(buffer, "P:%"PRId32, bmpP);
  Serial.println(buffer);
}

int32_t BMP180::getP(){
  return bmpP;
}

int8_t BMP180::getT(){
  return (int8_t)bmpT;
}


uint8_t bcd2dec(uint8_t bcd){
  return bcd/16*10 + bcd%16;
}

uint8_t dec2bcd(uint8_t dec){
  return dec/10*16 + dec%10;
}


