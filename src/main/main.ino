
void setup() {
  DDRD |= 0b00000010; //TX = output
  DDRD &= 0b11111110; //RX = input
  DHT11init();
  Serial.begin(9600);
}

void loop() {
  getData();
  delay(1000);
  DDRD |= 0b00010000;
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
  unsigned char b = readByte();
  Serial.println(b);
  
  //delayMicroseconds(20);
  DDRD &= 0b11101111;
}

unsigned char readByte(){
  unsigned char result = 0;
  for(int i = 0; i < 8; i++){
    while(!bitRead(PIND,4)); //wait for 50us low signal.
    delayMicroseconds(30);
    if(bitRead(PIND, 4)){ //if still HIGH, then a 1 was transmitted.
      result |= (1 << (7-i)); //add a 1 as the next bit.
    }
    while(bitRead(PIND,4)); //wait for the bit to complete transmission.
  }
  return result;
}

