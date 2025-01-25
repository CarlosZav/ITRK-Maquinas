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
const int pin_valvulaA = 2; // Apagado arriba, Prendido baja

int setTiempoPrendido = 0;
int setTiempoApagado = 0;

float conteo_ciclos = 0;
int seteo_ciclos = 0;

int prevPos = 1;

String inicio_prueba = "";
String estado_prueba = "Sin iniciar";

unsigned long tiempoPruebaA = 0;
unsigned long tiempoPruebaB = 0;

unsigned long tiempo_prueba = 0;
unsigned long tiempo_inicio = 0;
unsigned long tiempo_actual = 0;
unsigned long messageTimestamp = 0;

unsigned long tiempoInicioPausa = 0;
unsigned long tiempoFinPausa = 0;
unsigned long tiempoActualPausado = 0;
unsigned long tiempoPausadoAcumulado = 0;

bool estadoValvula = false;
bool enPausa = false;

int verificarTiempoArriba;
int verificarTiempoBajo;
 
#define USE_SERIAL Serial

void setup() {
  USE_SERIAL.begin(115200);

  USE_SERIAL.setDebugOutput(true);

  pinMode(pin_valvulaA, OUTPUT);
  digitalWrite(pin_valvulaA, LOW);

  conexion_internet();

}

void loop() {

  socketIO.loop();

  control();

  mandar_datos();

}

void control(){
  
  if((inicio_prueba == "NO") && (setTiempoApagado !=0 && setTiempoPrendido !=0 )) {

    tiempo_actual = millis();

    tiempo_prueba = (tiempo_actual - tiempo_inicio - tiempoPausadoAcumulado) / 1000;
    
    estado_prueba = "Sistema funcionando";

    tiempoPruebaA = tiempo_prueba *1000 - tiempoPruebaB;

    if((conteo_ciclos < seteo_ciclos ) && (seteo_ciclos > 0)){

      if ((tiempoPruebaA >= setTiempoPrendido) && (prevPos == 1)) { // Aquí se apaga la valvula
        digitalWrite(pin_valvulaA, LOW);

        tiempoPruebaB = tiempoPruebaB + setTiempoPrendido;

        prevPos = 2;
        conteo_ciclos = conteo_ciclos + 0.5;
        Serial.println("Abajo");
        Serial.print("Ciclos: ");
        Serial.println(conteo_ciclos);

      } else if((tiempoPruebaA >= setTiempoApagado) && (prevPos == 2)){ // Aquí se activa la valvula
        digitalWrite(pin_valvulaA, HIGH);
        tiempoPruebaB = tiempoPruebaB + setTiempoApagado;
        prevPos = 1;
        conteo_ciclos = conteo_ciclos + 0.5;
        Serial.println("Arriba");
        Serial.print("Ciclos: ");
        Serial.println(conteo_ciclos);
      }

    } else {
      tiempoPruebaB = 0;
      tiempoPruebaA = 0;
      conteo_ciclos = 0;
      seteo_ciclos = 0;
      estado_prueba = "finalizado";
      prevPos = 1;
      setTiempoPrendido = 0;
      setTiempoApagado = 0;
      tiempoActualPausado = 0;
      tiempoPausadoAcumulado = 0;
      digitalWrite(pin_valvulaA, LOW);
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
            if (eventName == "mensajePlanchas") {

                // El segundo elemento es el objeto que contiene los datos
                JsonObject data = doc[1].as<JsonObject>();

                seteo_ciclos = doc[1]["mensaje"]["setCiclos"];  // 20
                setTiempoApagado = doc[1]["mensaje"]["setTiempoElevado"];
                setTiempoPrendido = doc[1]["mensaje"]["setTiempoBajo"];
                inicio_prueba = doc[1]["mensaje"]["pausar"].as<String>();  // 20

                tiempo_prueba = 0;

                // Mostrar los valores en el monitor serie
                Serial.println(seteo_ciclos);
                Serial.println(setTiempoPrendido);
                Serial.println(setTiempoApagado);
                Serial.println(inicio_prueba);

                if( (seteo_ciclos != 0) && (setTiempoPrendido != 0) && (setTiempoApagado != 0) ){
                  tiempo_inicio = millis();
                  digitalWrite(pin_valvulaA, LOW);

                  conteo_ciclos = 0;
                  prevPos = 1;
                  tiempoActualPausado = 0;
                  tiempoPausadoAcumulado = 0;
                  tiempoPruebaB = 0;
                  tiempoPruebaA = 0;
                }

            }else if (eventName == "mensajePlanchasPausar"){
              JsonObject data = doc[1].as<JsonObject>();
              inicio_prueba = doc[1]["mensaje"]["pausar"].as<String>();  // 20
              Serial.println(inicio_prueba);

              if(inicio_prueba == "SI" && !enPausa){
                tiempoInicioPausa = millis();
                estado_prueba = "Sistema Pausado";
                enPausa = true;

              } else if (inicio_prueba == "NO" && enPausa) {
                tiempoFinPausa = millis();
                enPausa = false;
                tiempoActualPausado = tiempoFinPausa - tiempoInicioPausa;
                tiempoPausadoAcumulado = tiempoPausadoAcumulado + tiempoActualPausado;

                estado_prueba = "Sistema trabajando";
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
        array.add("datosEspPlanchas");

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

    //Totalplay-2.4G-d150
    //sqmZpzt5LPggsFkQ
    WiFiMulti.addAP("ITK-Servidor", "atazavcan");

    //WiFi.disconnect();
    while(WiFiMulti.run() != WL_CONNECTED) {
        delay(100);
    }

    String ip = WiFi.localIP().toString();
    USE_SERIAL.printf("[SETUP] WiFi Connected %s\n", ip.c_str());

    // server address, port and URL
    socketIO.begin("192.168.0.101", 5000, "/socket.io/?EIO=4");
    // event handler
    socketIO.onEvent(socketIOEvent);
}