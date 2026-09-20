#include <Arduino.h>

volatile int ultimoEstadoA = 0; // Último estado del pin A
volatile int contadorPulsos = 0;
const int pinA = 19; // GPIO de la señal A del encoder
const int pinB = 18; // GPIO de la señal B del encoder
const int pulsosPorRevolucion = 1200; // Número de pulsos por vuelta del encoder

int ultimoContadorPulsos = 0; // Variable para almacenar el último valor de contadorPulsos

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

}

void loop() {
  if (contadorPulsos != ultimoContadorPulsos) { // Solo imprime si hay un cambio
    float grados = (contadorPulsos / (float)pulsosPorRevolucion) * 360.0;
    Serial.print(grados);
    Serial.println(" grados");
    Serial.print("Ángulo: ");
    Serial.print("Pulsos: ");
    Serial.println(contadorPulsos);
    
    ultimoContadorPulsos = contadorPulsos; // Actualiza el último valor
  }
}
