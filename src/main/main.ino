
void setup() {
  DHT11init();
  Serial.begin(9600);
}


void loop() {
  getData();
  delay(5000);
  DDRD |= 0b00010000;
}

void DHT11init(){
  DDRD |= 0b00010010;
  DDRD &= 0b11111110;
  PORTD |= 0b00010000;
}

void getData(){
  PORTD &= 0b11101111;
  delay(18);
  PORTD |= 0b00010000; 
  DDRD &= 0b11101111; //turn signal pin to input in order to read data.
  double time = micros();
  while(bitRead(PIND, 4)); //wait for DHT response
  while(!bitRead(PIND, 4)); //DHT response
  while(bitRead(PIND, 4));
  time2 = micros() - time2;
  s = "Time2: ";
  s+=time2;
  Serial.println(s);

  //delayMicroseconds(20);
  DDRD &= 0b11101111;
}


