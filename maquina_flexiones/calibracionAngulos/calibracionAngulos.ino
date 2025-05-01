#include <Arduino.h>

volatile int contadorPulsos = 0;
const int pinEncoderA = 18; // GPIO de la señal A del encoder
const int pinEncoderB = 19; // GPIO de la señal B del encoder
const int pulsosPorRevolucion = 600; // Número de pulsos por vuelta del encoder

int ultimoContadorPulsos = 0; // Variable para almacenar el último valor de contadorPulsos

void IRAM_ATTR contarPulsos() {
  int estadoB = digitalRead(pinEncoderB);
  if (estadoB == HIGH) {
    contadorPulsos++;
  } else {
    contadorPulsos--;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(pinEncoderA, INPUT);
  pinMode(pinEncoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinEncoderA), contarPulsos, RISING);
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
