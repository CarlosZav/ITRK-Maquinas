#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <Hash.h>

ESP8266WiFiMulti WiFiMulti;
SocketIOclient socketIO;

// Pines valvulas
int pin_valvulaA = 2;
int pin_valvulaB = 0;

// Pines del encoder
const int pinA = 5; // Canal A del encoder (GPIO 34)
const int pinB = 4; // Canal B del encoder (GPIO 35)

int PPR = 4187; // Pulsos por revolucion del encoder

volatile int contadorPulsos = 0; // Cuenta de los pulsos
volatile int ultimoEstadoA = 0; // Último estado del pin A

int set_anguloA = 45;
int set_anguloB = -45;


int prevPos = 1;

int conteo_ciclos = 0;
int seteo_ciclos = 0;

String inicio_prueba = "";
String estado_prueba = "NoInicio";

unsigned long tiempo_prueba = 0;
unsigned long tiempo_inicio = 0;
unsigned long tiempo_actual = 0;
unsigned long messageTimestamp = 0;
 
#define USE_SERIAL Serial

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
  USE_SERIAL.begin(115200);

  USE_SERIAL.setDebugOutput(true);

  // Configurar pines del encoder como entradas
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);

  // Leer estado inicial del canal A
  ultimoEstadoA = digitalRead(pinA);

  // Configurar interrupción para el canal A
  attachInterrupt(digitalPinToInterrupt(pinA), encoderISR, CHANGE);

  pinMode(pin_valvulaA, OUTPUT);
  digitalWrite(pin_valvulaA, LOW);
  pinMode(pin_valvulaB, OUTPUT);
  digitalWrite(pin_valvulaB, LOW);

  conexion_internet();

}

void loop() {

  socketIO.loop();
  // put your main code here, to run repeatedly:

  control();

  mandar_datos();

}

void control(){
  
  if(inicio_prueba == "NO"){

    tiempo_actual = millis();
  
    tiempo_prueba = (tiempo_actual - tiempo_inicio) / 1000; 

    if((conteo_ciclos < seteo_ciclos) && (seteo_ciclos > 0)){

      static int ultimoContador = 0;

      if (contadorPulsos != ultimoContador) {

        Serial.print("Pulsos: ");
        Serial.println(contadorPulsos);
        float angulo = (contadorPulsos * 360) / PPR;
        Serial.print("Grados: ");
        Serial.println(angulo);

        if((angulo >= set_anguloA) && (prevPos == 1 || prevPos == 3)){
          
          digitalWrite(pin_valvulaA, HIGH);
          digitalWrite(pin_valvulaB, LOW);

          Serial.println("Sentido 1");

          conteo_ciclos = conteo_ciclos + 0.5;
          Serial.print("contador");
          Serial.println(conteo_ciclos);
          prevPos = 2;

        } else if ((angulo <= set_anguloB) && (prevPos == 1 || prevPos == 2)){
            
          digitalWrite(pin_valvulaA, LOW);
          digitalWrite(pin_valvulaB, HIGH);

          Serial.println("Sentido2");

          conteo_ciclos = conteo_ciclos + 0.5;
          Serial.print("contador");
          Serial.println(conteo_ciclos);
          prevPos = 3;
        }

        ultimoContador = contadorPulsos;

      }

    } else {
      conteo_ciclos = 0;
      seteo_ciclos = 0;
      estado_prueba = "finalizado";
      prevPos = 1;
      digitalWrite(pin_valvulaA, LOW);
      digitalWrite(pin_valvulaB, LOW);
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
            if (eventName == "mensajeFlexiones") {

                // El segundo elemento es el objeto que contiene los datos
                JsonObject data = doc[1].as<JsonObject>();

                seteo_ciclos = doc[1]["mensaje"]["seteo_ciclosF"];  // 20
                set_anguloA = doc[1]["mensaje"]["seteoAnguloA"];
                set_anguloB = doc[1]["mensaje"]["seteoAnguloB"];
                inicio_prueba = doc[1]["mensaje"]["pausarF"].as<String>();  // 20
                // Mostrar los valores en el monitor serie
                Serial.println(seteo_ciclos);
                Serial.println(inicio_prueba);

                if( (seteo_ciclos != 0) && (set_anguloA != 0) && (set_anguloB != 0) ){
                  tiempo_inicio = millis();
                  digitalWrite(pin_valvulaA, HIGH);
                  digitalWrite(pin_valvulaB, LOW);
                }

            }else if (eventName == "mensajeFlexiones_pausar"){
              JsonObject data = doc[1].as<JsonObject>();
              inicio_prueba = doc[1]["mensaje"]["pausarF"].as<String>();  // 20
              Serial.println(inicio_prueba);
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
        array.add("datos_espFlexiones");

        // add payload (parameters) for the event
        JsonObject param1 = array.createNestedObject();
        param1["conteo_ciclos"] = conteo_ciclos;   
        param1["tiempo_transcurrido"] = tiempo_prueba;
        param1["Estado"] = estado_prueba;          
    
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

  // disable AP
    if(WiFi.getMode() & WIFI_AP) {
        WiFi.softAPdisconnect(true);
    }

    WiFiMulti.addAP("333", "12345678");

    //WiFi.disconnect();
    while(WiFiMulti.run() != WL_CONNECTED) {
        delay(100);
    }

    String ip = WiFi.localIP().toString();
    USE_SERIAL.printf("[SETUP] WiFi Connected %s\n", ip.c_str());

    // server address, port and URL
    socketIO.begin("192.168.137.90", 5000, "/socket.io/?EIO=4");
    // event handler
    socketIO.onEvent(socketIOEvent);
}

