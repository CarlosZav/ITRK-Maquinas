
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>

WiFiMulti WiFiMulti;
SocketIOclient socketIO;

// Pines del encoder
const int encoderPinA = 18;
const int encoderPinB = 19;

//Pines para el control del motor
const int pinPWMA = 21;
const int pinPWMB = 25;
const int pinMotorA = 22;
const int pinMotorB = 23;

float gradosActuales = 0;

// Variables para el contador de pulsos y la velocidad
volatile long pulseCount = 0;
volatile long pulseCountGrades = 0;
unsigned long lastTime = 0;
float velocidadRPM = 0;
int direction = 1;  // 1 para adelante, -1 para atrás

//Valor para saturacion de la señál de repuesta
float velocidadMax = 120.0; //Velocidad Maxima del motor en rpm
float velocidadMin = 20.0; //Velocidad minima a la que el motor se e piez a a mover

//variavles PID
float cv = 0;
float cv1 = 0;
float error = 0;
float error1 = 0;
float error2 = 0;
float kp = 2.5; // 3.26
float ki = 0.002;
float kd = 0.31; //0.31
float tm = 0.1;

int pulsesB = 0;

float angulo = 0.0;

int revolucionesTotales = 0;

int estadoPausa = 0;

float PPR = 4210; // Pulsos por revolucion del encoder

float setVelocidad = 0;
float setRevolucionesCambio = 0;
float setRevoluciones = 0;
float conteoRevoluciones = 0;

int prevPos = 1;

String inicio_prueba = "";
String estado_prueba = "Sin iniciar";

unsigned long tiempo_prueba = 0;
unsigned long tiempo_inicio = 0;
unsigned long tiempo_actual = 0;
unsigned long messageTimestamp = 0;

unsigned long tiempoInicioPausa = 0;
unsigned long tiempoFinPausa = 0;
unsigned long tiempoActualPausado = 0;
unsigned long tiempoPausadoAcumulado = 0;
 
#define USE_SERIAL Serial

// Función de interrupción para contar los pulsos del encoder
void IRAM_ATTR encoderISR() {
  // Lee el estado de los pines A y B
  int stateA = digitalRead(encoderPinA);
  int stateB = digitalRead(encoderPinB);

  // Determina la dirección basada en la secuencia de los pulsos
  if (stateA == HIGH) {
    if (stateB == LOW) {
      direction = 1;  // Rotación en sentido horario
    } else {
      direction = -1; // Rotación en sentido antihorario
    }
  } else {
    if (stateB == HIGH) {
      direction = 1;  // Rotación en sentido horario
    } else {
      direction = -1; // Rotación en sentido antihorario
    }
  }

  // Incrementa o decrementa el contador de pulsos según la dirección
  pulseCount ++;
  pulseCountGrades ++;
  pulsesB ++;
}

void setup() {
  USE_SERIAL.begin(115200);

  USE_SERIAL.setDebugOutput(true);

  // Configura los pines del encoder como entradas
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  pinMode(pinMotorA, OUTPUT);
  digitalWrite(pinMotorA, LOW);

  pinMode(pinMotorB, OUTPUT);
  digitalWrite(pinMotorB, LOW);

  pinMode(pinPWMA, OUTPUT);
  pinMode(pinPWMB, OUTPUT);

  analogWrite(pinPWMA, 0);
  analogWrite(pinPWMB, 0);

  // Configura la interrupción en el pin A del encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);

  conexion_internet();

}

void loop() {
  socketIO.loop();
  // put your main code here, to run repeatedly:
  control();

  mandar_datos();
}

void control(){

  calcularVelocidad();
  controlPID();

  if((inicio_prueba == "NO") && ((setVelocidad != 0) && (setRevolucionesCambio !=0) && (setRevoluciones != 0))){

    digitalWrite(pinMotorA, HIGH);
    digitalWrite(pinMotorB, HIGH);

    tiempo_actual = millis();
    tiempo_prueba = (tiempo_actual - tiempo_inicio - tiempoPausadoAcumulado) / 1000;

    gradosActuales = (pulseCountGrades * 360) / PPR;

    conteoRevoluciones = gradosActuales / 360;
    revolucionesTotales = (pulsesB * 360 / PPR) / 360;
    
    if(revolucionesTotales < setRevoluciones){

      if((conteoRevoluciones >= setRevolucionesCambio) && (prevPos == 1)){
        prevPos = 2;
        gradosActuales = 0;
        conteoRevoluciones = 0;
        pulseCountGrades = 0;
      } else if ((conteoRevoluciones >= setRevolucionesCambio) && (prevPos == 2)){
        prevPos = 1;
        gradosActuales = 0;
        conteoRevoluciones = 0;
        pulseCountGrades = 0;
      }

    } else{

      conteoRevoluciones = 0;
      setRevoluciones = 0;
      estado_prueba = "finalizado";
      setVelocidad = 0;
      tiempoActualPausado = 0;
      tiempoPausadoAcumulado = 0;
      digitalWrite(pinMotorA, LOW);
      digitalWrite(pinMotorB, LOW);

    }

  }

}

void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case sIOtype_DISCONNECT:
            USE_SERIAL.printf("[IOc] Disconnected!\n");
            break;
        case sIOtype_CONNECT:
            USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);
            socketIO.send(sIOtype_CONNECT, "/");
            break;
        case sIOtype_EVENT:
            USE_SERIAL.printf("holaaaa");
            USE_SERIAL.printf("[IOc] get event: %s\n", payload);

            // Deserializar el payload en un objeto JSON
            DynamicJsonDocument doc(1024);
            DeserializationError error = deserializeJson(doc, payload);

            // Manejar posibles errores de deserialización
            if (error) {
                USE_SERIAL.print("Error de parseo: ");
                USE_SERIAL.println(error.c_str());
                return;
            }

            // Obtener el nombre del evento (el primer elemento del array)
            String eventName = doc[0].as<String>();

            USE_SERIAL.printf("Evento recibido: %s\n", eventName.c_str());

            // Comprobar el nombre del evento y obtener los datos correspondientes
            if (eventName == "mensajeSecadorasRot") {

                // El segundo elemento es el objeto que contiene los datos
                JsonObject data = doc[1].as<JsonObject>();

                setRevoluciones = doc[1]["mensaje"]["revolucionesSecadoras"];  // 20
                setRevolucionesCambio = doc[1]["mensaje"]["revCambioSecadoras"]; 
                setVelocidad = doc[1]["mensaje"]["velocidadRevoluciones"];             
                inicio_prueba = doc[1]["mensaje"]["pausarSecadorasRot"].as<String>();  // 20

                tiempo_prueba = 0;
                conteoRevoluciones = 0;

                estado_prueba = "Sistema funcionando";

                // Mostrar los valores en el monitor serie
                Serial.println(setRevoluciones);
                Serial.println(setRevolucionesCambio);
                Serial.println(setVelocidad);
                Serial.println(inicio_prueba);

                if( (setRevoluciones != 0) && (setRevolucionesCambio != 0) && (setVelocidad != 0) ){
                  tiempo_inicio = millis();
                  estadoPausa = 1;
                }

            }else if (eventName == "mensajeSecadorasRotPausar"){
              JsonObject data = doc[1].as<JsonObject>();
              inicio_prueba = doc[1]["mensaje"]["pausarSecadorasRot"].as<String>();  // 20
              Serial.println(inicio_prueba);

              if(inicio_prueba == "SI"){
                tiempoInicioPausa = millis();
                estado_prueba = "Sistema Pausado";
                estadoPausa = 2;

                digitalWrite(pinMotorA, LOW);
                digitalWrite(pinMotorB, LOW);

              } else if (inicio_prueba == "NO") {
                if(estadoPausa == 2){
                tiempoFinPausa = millis();
                tiempoActualPausado = tiempoFinPausa - tiempoInicioPausa;
                tiempoPausadoAcumulado = tiempoPausadoAcumulado + tiempoActualPausado;
                tiempoInicioPausa = 0;
                estado_prueba = "Sistema trabajando";
                estadoPausa = 1;

                digitalWrite(pinMotorA, HIGH);
                digitalWrite(pinMotorB, HIGH);
                }
              }
            }

                // Verificar si las claves existen y obtener los valores con seguridad
            break;
    }
}


void mandar_datos(){
  uint64_t now = millis();

    if(now - messageTimestamp > 2000) {
        messageTimestamp = now;

        // creat JSON message for Socket.IO (event)
        DynamicJsonDocument doc(1024);
        JsonArray array = doc.to<JsonArray>();

        // add evnet name
        // Hint: socket.on('event_name', ....
        array.add("datosEspSecadorasRot");

        // add payload (parameters) for the event
        JsonObject param1 = array.createNestedObject();
        param1["conteo_revSecadorasRot"] = revolucionesTotales;   
        param1["estado_pruebaSecadorasRot"] = estado_prueba;
        param1["tiempo_pruebaSecadorasRot"] = tiempo_prueba;
        param1["velocidad_SecadorasRot"] = velocidadRPM;
        param1["setRevSecadorasRot"] = setRevoluciones;               
    
        // JSON to String (serializion)
        String output;
        serializeJson(doc, output);

        // Send event
        socketIO.sendEVENT(output);

        // Print JSON for debugging
        USE_SERIAL.println(output);
    }
}

void conexion_internet(){
  for(uint8_t t = 4; t > 0; t--) {
          USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
          USE_SERIAL.flush();
          delay(1000);
      }
    WiFiMulti.addAP("victus", "12345678");

    //WiFi.disconnect();
    while(WiFiMulti.run() != WL_CONNECTED) {
        delay(100);
    }

    String ip = WiFi.localIP().toString();
    USE_SERIAL.printf("[SETUP] WiFi Connected %s\n", ip.c_str());

    // server address, port and URL
    socketIO.begin("10.224.54.198", 5000, "/socket.io/?EIO=4");
    // event handler
    socketIO.onEvent(socketIOEvent);
}


void calcularVelocidad(){
  // Obtén el tiempo actual
  unsigned long currentTime = millis();

  // Calcula la velocidad cada 100 ms
  if (currentTime - lastTime >= 100) {
    // Desactiva las interrupciones temporalmente para evitar inconsistencias
    noInterrupts();
    long pulses = labs(pulseCount);
    pulseCount = 0;
    interrupts();

    // Calcula la velocidad en RPM
    velocidadRPM = (pulses * 60000.0) / (PPR * (currentTime - lastTime));

    // Actualiza el tiempo de la última medición
    lastTime = currentTime;

    // Muestra la velocidad en RPM por el monitor serial
    Serial.print("Velocidad: ");
    Serial.print(velocidadRPM);
    Serial.print(" RPM, Dirección: ");
    if (direction == 1) {
      Serial.println("Adelante");
    } else {
      Serial.println("Atrás");
    }
    /*Serial.print("SetPoint = ");
    Serial.println(setVelocidad);
    Serial.print("Velocidad = ");
    Serial.println(velocidadRPM);
    Serial.print("Valor de cv = ");
    Serial.println(cv);
    Serial.print("error");
    Serial.println(error);*/
  }
}

void calcularGrados(){
  gradosActuales = (pulseCountGrades * 360) / PPR;
  /*Serial.print("Pulsos: ");
  Serial.println(pulseCountGrades);
  Serial.print("Grados: ");
  Serial.println(gradosActuales);
  */
}


void controlPID(){

  
  error = setVelocidad - velocidadRPM;

  //Ecuacion control PID discreto
  cv = cv1 + (kp + kd / tm) * error + (-kp+ki*tm - 2*kd/tm) * error1 + (kd/tm)*error2;
  
  error2 = error1;
  error1 = error;

  if (cv > velocidadMax){
    cv = velocidadMax;
  } else if (cv < velocidadMin){
    cv = velocidadMin;
  }

  cv1 = cv;

  if (prevPos == 1){
  analogWrite(pinPWMA, (cv*255) / 120);
  // ledcWriteChannel(canalPWM1, (cv*255) / 120);    // PWM en GPIO 5
  //ledcWriteChannel(canalPWM2, 0);    // PWM en GPIO 5
  analogWrite(pinPWMB, 0);
  } else if(prevPos == 2){
    analogWrite(pinPWMA, 0);
    // ledcWriteChannel(canalPWM1, (cv*255) / 120);    // PWM en GPIO 5
    //ledcWriteChannel(canalPWM2, 0);    // PWM en GPIO 5
    analogWrite(pinPWMB, (cv*255) / 120);
  }
 /*
  Serial.print("SetPoint = ");
  Serial.println(setPoint);
  Serial.print("Velocidad = ");
  Serial.println(velocidadRPM);
  */
}

float modulo(float a, float b) {
  // Calcula el cociente de a / b
  float cociente = a / b;
  
  // Redondea el cociente hacia abajo para obtener la parte entera
  int parteEntera = (int)cociente;
  
  // Calcula el residuo (módulo)
  float residuo = a - (parteEntera * b);
  
  return residuo;
}

