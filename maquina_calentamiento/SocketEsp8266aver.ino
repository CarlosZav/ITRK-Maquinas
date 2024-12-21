#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <ArduinoJson.h>

#include <WebSocketsClient.h>
#include <SocketIOclient.h>

#include <Hash.h>

ESP8266WiFiMulti WiFiMulti;
SocketIOclient socketIO;

int ssrPin = 5;

// Variables para controlar el tiempo de conmutación
unsigned long tiempoEncendido = 0;  // Tiempo de encendido (valor predeterminado 5 segundos)
unsigned long tiempoApagado = 0;    // Tiempo de apagado (valor predeterminado 3 segundos)
unsigned long tiempoEncendido2 = 0;  // Tiempo de encendido (valor predeterminado 5 segundos)
unsigned long tiempoApagado2 = 0;    // Tiempo de apagado (valor predeterminado 3 segundos)
unsigned long hola = 0;
unsigned long messageTimestamp = 0;

unsigned long tiempoActual = 0;
unsigned long ultimaConmutacion = 0;
bool ssrEncendido = false;
int conteoCiclos = 0;
int seteo_ciclos = 0;
unsigned long seteo_tiempoApagado = 0;
unsigned long seteo_tiempoPrendido = 0;
int seteo_ciclos2 = 0;

//Variables para medir corriente con el sensor
const int pinSensor = A0;  // Pin para el sensor de corriente
int valorADC = 0;
float voltajeSensor = 0;
float corriente = 0;

#define USE_SERIAL Serial

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
            if (eventName == "hola2") {
                USE_SERIAL.printf("Evento 'hola2' recibido\n");

                // El segundo elemento es el objeto que contiene los datos
                JsonObject data = doc[1].as<JsonObject>();

                seteo_ciclos = doc[1]["mensaje"]["seteo_ciclos"];  // 20
                tiempoApagado = doc[1]["mensaje"]["tiempo_apagado"];  // 30
                tiempoEncendido = doc[1]["mensaje"]["tiempo_prendido"];  // 40

                // Mostrar los valores en el monitor serie
                Serial.println(seteo_ciclos);
                Serial.println(tiempoApagado);
                Serial.println(tiempoEncendido);
            }

                // Verificar si las claves existen y obtener los valores con seguridad
                
            break;
    }
}

void setup() {

      // Inicializar el pin del SSR como salida
    pinMode(ssrPin, OUTPUT);
    digitalWrite(ssrPin, LOW);  // Asegurarse de que el SSR comience apagado

    // USE_SERIAL.begin(921600);
    USE_SERIAL.begin(115200);

    //Serial.setDebugOutput(true);
    USE_SERIAL.setDebugOutput(true);

    USE_SERIAL.println();
    USE_SERIAL.println();
    USE_SERIAL.println();

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

void loop() {

  socketIO.loop();

  realizar_ciclos();

  medir_sensor();

  mandar_datos();

}

void realizar_ciclos(){
  // Obtener el tiempo actual
  tiempoActual = millis();

  if (seteo_ciclos != seteo_ciclos2){
    seteo_ciclos2 = seteo_ciclos;
  }

  if (tiempoApagado != tiempoApagado2){
    tiempoApagado2 = tiempoApagado;
  }

  if (tiempoEncendido != tiempoEncendido2){
    tiempoEncendido2 = tiempoEncendido;
  }

  //Apagado hasta que se mande un valor por el usuario
  if((tiempoEncendido2 > 0) && (tiempoApagado2 > 0 ) && (conteoCiclos < seteo_ciclos2) && (seteo_ciclos2 > 0)){


      hola = tiempoActual - ultimaConmutacion;

    // Verificar si ha pasado el tiempo para conmutar el estado del SSR
    if (ssrEncendido) {
       // Si está encendido y ha pasado el tiempo de encendido
      if (hola >= tiempoEncendido2) {
        digitalWrite(ssrPin, LOW);  // Apagar el SSR
        ssrEncendido = false;  // Cambiar el estado a apagado
        Serial.println("APAGADOOOO");
        ultimaConmutacion = tiempoActual;  // Registrar el tiempo de la última conmutación
        conteoCiclos++;
        Serial.print("Ciclos: ");
        Serial.println(conteoCiclos);
        Serial.println(seteo_ciclos);
      }
    } 
    else {

      // Si está apagado y ha pasado el tiempo de apagado
      if ((hola >= tiempoApagado2)){
        digitalWrite(ssrPin, HIGH);  // Encender el SSR
        ssrEncendido = true;  // Cambiar el estado a encendido
        Serial.println("ENCENDIDOOOOO");
        ultimaConmutacion = tiempoActual;  // Registrar el tiempo de la última conmutación
            //  Aumentar +1 el conteo de ciclo
      }
    }

  } 
   if (conteoCiclos >= seteo_ciclos2){
    conteoCiclos = 0;
    seteo_ciclos2 = 0;
    seteo_ciclos = 0;
    tiempoEncendido2 = 0;
    tiempoApagado2 = 0;
    ssrEncendido = false;
    digitalWrite(ssrPin, LOW);  // Apagar el SSR
  }

}

void mandar_datos(){
  uint64_t now = millis();

    if(now - messageTimestamp > 1000) {
        messageTimestamp = now;

        // creat JSON message for Socket.IO (event)
        DynamicJsonDocument doc(1024);
        JsonArray array = doc.to<JsonArray>();

        // add evnet name
        // Hint: socket.on('event_name', ....
        array.add("datos_esp");

        // add payload (parameters) for the event
        JsonObject param1 = array.createNestedObject();
        param1["estado_ssr"] = ssrEncendido;
        param1["conteo_ciclos"] = conteoCiclos;    
        param1["tiempo_transcurrido"] = hola;
        param1["corriente"] = corriente;          
    
        // JSON to String (serializion)
        String output;
        serializeJson(doc, output);

        // Send event
        socketIO.sendEVENT(output);

        // Print JSON for debugging
        USE_SERIAL.println(output);
    }
}

void medir_sensor(){
  // Leer el valor analógico del pin del sensor
  valorADC = analogRead(pinSensor);

  Serial.println(valorADC);
  // Convertir el valor ADC a voltaje
  voltajeSensor = (valorADC * 3.3) / 1024;
  // Calcular la corriente a partir del voltaje del sensor
  // Restamos el voltaje base (mitad de referencia para el ACS712)
  corriente = (voltajeSensor - (3.3 / 2)) / 0.04;

  yield();

  delay(100);

}
