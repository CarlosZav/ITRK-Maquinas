const int senial1 = 15; // Pin donde está conectado el botón
const int senial2 = 16; // Pin donde está conectado el botón
const int senialPrincipal = 17;

const int rele1 = 2;
const int rele2 = 3;
const int relePrincipal = 4;

void setup() {
  // Configuramos el pin del botón como entrada con resistencia pull-up
  pinMode(senial1, INPUT);
  pinMode(senial2, INPUT);
  pinMode(senialPrincipal, INPUT);

  pinMode(rele1, OUTPUT);
  pinMode(rele2, OUTPUT);
  pinMode(relePrincipal, OUTPUT);

  digitalWrite(rele1, LOW);
  digitalWrite(rele2, LOW);
  digitalWrite(relePrincipal, LOW);

  // Iniciamos la comunicación serial
  Serial.begin(115200);
  Serial.println("Programa iniciado. Presiona el botón.");

}

void loop() {

  // Enviamos el estado del botón por el puerto serial
  if (digitalRead(senial1) == HIGH && digitalRead(senial2) == LOW) { // El botón está presionado
    digitalWrite(rele1, HIGH);
    digitalWrite(rele2, LOW);
    
  } else if (digitalRead(senial1) == LOW && digitalRead(senial2) == HIGH){ // El botón está liberado
    digitalWrite(rele1, LOW);
    digitalWrite(rele2, HIGH);
  }

  if (digitalRead(senialPrincipal) == HIGH){
    digitalWrite(relePrincipal, HIGH);
  } else if(digitalRead(senialPrincipal) == LOW){
    digitalWrite(relePrincipal, LOW);
  }

}
