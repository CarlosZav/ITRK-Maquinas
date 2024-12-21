#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <Hash.h>

ESP8266WiFiMulti WiFiMulti;
SocketIOclient socketIO;

int pin_sensorA = 16;
int pin_sensorB = 5;
int pin_valvulaA = 4;
int pin_valvulaB = 0;

bool posicionA;
bool posicionB;
bool prevposicionB = false;
int conteo_ciclos = 0;
int seteo_ciclos = 0;
unsigned long seteo_tiempoON = 0;
unsigned long seteo_tiempoOFF = 0;


String inicio_prueba = "";
String estado_prueba = "NoInicio";
unsigned long tiempo_prueba = 0;
unsigned long tiempo_inicio = 0;
unsigned long tiempo_actual = 0;
unsigned long messageTimestamp = 0;

#define USE_SERIAL Serial

void setup() {
  USE_SERIAL.begin(115200);
  USE_SERIAL.setDebugOutput(true);

  // put your setup code here, to run once:
  pinMode(pin_sensorA, INPUT_PULLUP);
  pinMode(pin_sensorB, INPUT_PULLUP);

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

  if((conteo_ciclos < seteo_ciclos) && (seteo_ciclos > 0)){
    
    tiempo_actual = millis();
 
    tiempo_prueba = (tiempo_actual - tiempo_inicio) / 1000; 

    posicionA = digitalRead(pin_sensorA) == LOW;
    posicionB = digitalRead(pin_sensorB) == LOW;

    if((posicionA) && (prevposicionB == true) && (tiempo_conteoB >= seteo_tiempoOFF)){
      digitalWrite(pin_valvulaA, HIGH);
      digitalWrite(pin_valvulaB, LOW);
      prevposicionB = false;

      tiempo_conteoA = millis(); 

      tiempo_conteoB = 0;
    }

    if((posicionB) && (prevposicionB == false) && (tiempo_conteoA >= seteo_tiempoON)){
      digitalWrite(pin_valvulaA, LOW);
      digitalWrite(pin_valvulaB, HIGH);

      conteo_ciclos++;
      Serial.print("contador");
      Serial.println(conteo_ciclos);
      prevposicionB = true;

      tiempo_conteoB = millis(); 

      tiempo_conteoA = 0;
    }

    

  } else {
    conteo_ciclos = 0;
    seteo_ciclos = 0;
    estado_prueba = "finalizado";
    prevposicionB = false;
    digitalWrite(pin_valvulaA, LOW);
    digitalWrite(pin_valvulaB, LOW);
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
                inicio_prueba = doc[1]["mensaje"]["indicacionF"].as<String>();  // 20
                seteo_tiempoON = doc[1]["mensaje"]["seteo_tiempoON"];  // 20
                seteo_tiempoOFF = doc[1]["mensaje"]["seteo_tiempoOFF"];  // 20
                // Mostrar los valores en el monitor serie
                Serial.println(seteo_ciclos);
                Serial.println(inicio_prueba);
                Serial.println(seteo_tiempoON);
                Serial.println(seteo_tiempoOFF);

                if(seteo_ciclos != 0){
                  tiempo_inicio = millis();
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

    WiFiMulti.addAP("222", "12345678");

    //WiFi.disconnect();
    while(WiFiMulti.run() != WL_CONNECTED) {
        delay(100);
    }

    String ip = WiFi.localIP().toString();
    USE_SERIAL.printf("[SETUP] WiFi Connected %s\n", ip.c_str());

    // server address, port and URL
    socketIO.begin("192.168.137.92", 5000, "/socket.io/?EIO=4");
    // event handler
    socketIO.onEvent(socketIOEvent);
}