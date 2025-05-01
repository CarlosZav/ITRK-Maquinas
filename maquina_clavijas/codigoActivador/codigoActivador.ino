const int pinEntrada = 13;
const int pinSalida = 12;

void setup() {
  pinMode(pinSalida, OUTPUT);
  digitalWrite(pinSalida, LOW);  // Asegurarse de que el SSR comience apagado

  pinMode(pinEntrada, INPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(pinEntrada) == HIGH){
    digitalWrite(pinSalida, HIGH);
    Serial.println("Salida HIGH");
  } else{
    digitalWrite(pinSalida, LOW);
    Serial.println("Salida LOW");
  }
}
