#include <Wire.h>

#define BMP180_ADDR 119
#define DS3231_ADDR 104

#define BMP180_CTRL_MEAS 0xF4
#define BMP180_OUT_MSB 0xF6

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

void loop() {
  DS3231 x;
  uint8_t arr[6] = {20, 34, 8, 3, 31, 7};
  x.initDS3231(24, arr);
  while(1){
    x.getDataDS3231();
    x.printDS3231();
    delay(1000);
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

uint8_t bcd2dec(uint8_t bcd){
  return bcd/16*10 + bcd%16;
}

uint8_t dec2bcd(uint8_t dec){
  return dec/10*16 + dec%10;
}


