const int senial = 16;
const int valvula = 3;

void setup() {
  // put your setup code here, to run once:
  pinMode(senial, INPUT);

  pinMode(valvula, OUTPUT);
  pinMode(valvula, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:

  if (digitalRead(senial) == LOW) { // El botón está presionado
    digitalWrite(valvula, HIGH);    
  } else if (digitalRead(senial) == HIGH){ // El botón está liberado
    digitalWrite(valvula, LOW);
  }

}
