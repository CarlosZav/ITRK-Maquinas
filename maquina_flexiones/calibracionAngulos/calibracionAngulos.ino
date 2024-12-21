// Pines del encoder
const int pinA = 5; // Canal A del encoder (GPIO 34)
const int pinB = 4; // Canal B del encoder (GPIO 35)

int PPR = 4187;

int prevPos = 1;

float conteo_ciclos = 0;
          
// Variables globales
volatile int contadorPulsos = 0; // Cuenta de los pulsos
volatile int ultimoEstadoA = 0; // Último estado del pin A

void IRAM_ATTR encoderISR() {
  int estadoA = digitalRead(pinA);
  int estadoB = digitalRead(pinB);

  // Determinar dirección según el cambio de estados
  if (estadoA != ultimoEstadoA) {
    if (estadoA == estadoB) {
      contadorPulsos++; // Girando en sentido horario
    } else {
      contadorPulsos--; // Girando en sentido antihorario
    }
  }
  ultimoEstadoA = estadoA; // Actualizar el estado previo
}

void setup() {
  Serial.begin(115200);

  // Configurar pines del encoder como entradas
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);

  // Leer estado inicial del canal A
  ultimoEstadoA = digitalRead(pinA);

  // Configurar interrupción para el canal A
  attachInterrupt(digitalPinToInterrupt(pinA), encoderISR, CHANGE);

  Serial.println("Iniciando lectura del encoder...");
}

void loop() {
  // Imprimir el contador de pulsos cada 500 ms
  static int ultimoContador = 0;
  if (contadorPulsos != ultimoContador) {

    Serial.print("Pulsos: ");
    Serial.println(contadorPulsos);
    float angulo = (contadorPulsos * 360) / PPR;
    Serial.print("Grados: ");
    Serial.println(angulo);

    if((angulo >= 45) && (prevPos == 1 || prevPos == 3)){
      
      Serial.println("Sentido 1");

      conteo_ciclos = conteo_ciclos + 0.5;
      Serial.print("contador");
      Serial.println(conteo_ciclos);
      prevPos = 2;

    } else if ((angulo <= -45) && (prevPos == 1 || prevPos == 2)){
        
      Serial.println("Sentido2");

      conteo_ciclos = conteo_ciclos + 0.5;
      Serial.print("contador");
      Serial.println(conteo_ciclos);
      prevPos = 3;
    }

    ultimoContador = contadorPulsos;

  }
}
