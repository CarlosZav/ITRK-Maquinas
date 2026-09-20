#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ArduinoJson.h>
#include <SocketIOclient.h>
#include <Wire.h>
NAU7802 myScale; // Create instance of the NAU7802 class

/* ---------- PID VARIABLES ---------- */
float Kp = 40.0;
float Ki = 200.0;
float Kd = 0.0;

float error, prevError = 0;
float integral = 0;

// Pines del motor a pasos
const int PUL = 18;
const int DIR = 19;
const int ENA = 5;

bool direccion = HIGH;
bool estadoPulso = LOW;

float pasosActuales = 0;

// Valor para saturacion de la señal de respuesta
float pasosMax = 100.0; // Pasos máximos para que se mueva max 45 grados
float pasosMin = 0.0;   // Pasos mínimos

float fuerzaFinal  = 60;
float fuerzaInicial = 30;

float setPoint = 30; // <<< SetPoint inicial
int pasosMotorPid = 0;

float fuerzaMedida = 0.0;
float contadorPasos = 0.0;

int pasosPorVuelta = 800;
int velocidadVuelta = 600;

float intervaloPaso = 5;

unsigned long tiempoAnterior = 0;
unsigned long tiempoControl = 0;
unsigned long tiempoControlSetPoint = 0;

// ================== NUEVAS VARIABLES PARA RAMPA ==================
unsigned long tiempoCambioSet = 0;

const unsigned long tiempoTotalSet = 5000; // 5 segundos

const unsigned long tiempoEspera = 5000; // 5 segundos
const int pasosSet = 5; // 50->100->150->200
const unsigned long intervaloSet = tiempoTotalSet / pasosSet;
const int incrementoSet = (fuerzaFinal - fuerzaInicial) / pasosSet;
// ================================================================

int conteoCambioSet = 0;
int finalizado = 1;
int orientacion = 0;

int multiplicadorPasos = 48;

int conteoCiclosFuerza = 0;

int ciclosFuerza  = 10;

unsigned long tiempoActualFuerza = 0;
unsigned long ultimoTiempoFuerza = 0;

bool subProcesoFuerza = HIGH;

void setup() {

  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENA, OUTPUT);

  digitalWrite(PUL, LOW);
  digitalWrite(DIR, direccion);
  digitalWrite(ENA, LOW); // <<< Habilitar driver

  Serial.begin(115200);
  Wire.begin();

  if (myScale.begin() == false) {
    Serial.println("Scale not detected. Please check wiring. Freezing...");
    while (1);
  }

  Serial.println("Scale detected!");
  tiempoCambioSet = millis(); // <<< Inicializar tiempo de rampa
}

void loop() {

  tiempoActualFuerza = millis();

  if (tiempoActualFuerza - ultimoTiempoFuerza > tiempoTotalSet){
    ultimoTiempoFuerza = tiempoActualFuerza;
    subProcesoFuerza = !subProcesoFuerza;
  }

  if (subProcesoFuerza == HIGH){
    actualizarSetPoint(); // <<< Rampa de setPoint
    controlPID();
  } else{
    if (pasosMotorPid > 0){
      pasosMotorPid--;
      moverMultiplesPasos(multiplicadorPasos, HIGH);
      if (pasosMotorPid > 100) pasosMotorPid = 100;
      else if (pasosMotorPid < 0) pasosMotorPid = 0;
    }
  }
}

// ================== FUNCIÓN RAMPA DE SETPOINT ==================
void actualizarSetPoint() {

  unsigned long tiempoActual = millis();

  if ((tiempoActual - tiempoCambioSet >= intervaloSet) && (setPoint < fuerzaFinal)) {

    setPoint += incrementoSet;

    if (setPoint > fuerzaFinal) {
      setPoint = fuerzaFinal;
    }

    tiempoCambioSet = tiempoActual;

    Serial.print("Nuevo SetPoint: ");
    Serial.println(setPoint);
  }
}
// ===============================================================

void medirFuerza() {

  if (myScale.available() == true) {
    int32_t currentReading = myScale.getReading();
    Serial.print("Reading: ");
    Serial.println(currentReading);

    fuerzaMedida = (currentReading * 0.0002702) - 890.7751; //-10 debe ser 878
    Serial.print("Fuerza Medida: ");
    Serial.println(fuerzaMedida);
  }
}
void controlPID() {

  medirFuerza();

  if ((fuerzaMedida <= setPoint - 3) && (pasosMotorPid < 100)) {

    pasosMotorPid++;
    //aqui falta mulplicar el multiplicador de pAsos
    moverMultiplesPasos(multiplicadorPasos, LOW);

    if (pasosMotorPid > 100) pasosMotorPid = 100;
    else if (pasosMotorPid < 0) pasosMotorPid = 0;

  } else if ((fuerzaMedida >= setPoint + 3) && (pasosMotorPid > 0)) {
    
    pasosMotorPid--;
    moverMultiplesPasos(multiplicadorPasos, HIGH);

    if (pasosMotorPid > 100) pasosMotorPid = 100;
    else if (pasosMotorPid < 0) pasosMotorPid = 0;
  }

  Serial.print("pasos PID: ");
  Serial.println(pasosMotorPid);
}

void moverMultiplesPasos(int cantidad, int sentido) {

  digitalWrite(DIR, sentido);

  for (int i = 0; i < cantidad; i++) {

    digitalWrite(PUL, HIGH);
    delayMicroseconds(3);
    digitalWrite(PUL, LOW);
    delayMicroseconds(3);

    if (sentido == HIGH) contadorPasos++;
    else contadorPasos--;

    Serial.print("Posición Actual: ");
    Serial.println(contadorPasos);
  }
}
