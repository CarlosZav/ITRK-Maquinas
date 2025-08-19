const int inputPin = 2;
const int rele = 13;
int value = 0;
 
void setup() {
  pinMode(inputPin, INPUT_PULLUP);
  pinMode(rele, OUTPUT);
  digitalWrite(rele, HIGH);
  delay(5000);
}
 
void loop(){
  value = digitalRead(inputPin);  //lectura digital de pin
 
  //mandar mensaje a puerto serie en función del valor leido
  if (value == HIGH) {
      delayMicroseconds(4166);
      digitalWrite(rele,LOW);
  }
}