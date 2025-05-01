#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>

WiFiMulti WiFiMulti;
SocketIOclient socketIO;

float angulo = 0.0;

int estadoPausa = 0;

// Pines valvulas
int pin_valvulaA = 32; // 
int pin_valvulaB = 33; //
int pinValvulaPrincipal = 25;

// Pines del encoder
const int pinA = 18; // Canal A del encoder (GPIO 34)
const int pinB = 19; // Canal B del encoder (GPIO 35)

int PPR = 1200; // Pulsos por revolucion del encoder

volatile int contadorPulsos = 0; // Cuenta de los pulsos
volatile int ultimoEstadoA = 0; // Último estado del pin A

int set_anguloA = 0;
int set_anguloB = 0;

int prevPos = 1;

float conteo_ciclos = 0;
int seteo_ciclos = 0;

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

  pinMode(pinValvulaPrincipal, OUTPUT);
  digitalWrite(pinValvulaPrincipal, LOW);

  conexion_internet();

}

void loop() {

  socketIO.loop();
  // put your main code here, to run repeatedly:

  control();

  mandar_datos();

}

void control(){
  
  if(inicio_prueba == "NO") {

    tiempo_actual = millis();

    tiempo_prueba = (tiempo_actual - tiempo_inicio - tiempoPausadoAcumulado) / 1000; 

    if((conteo_ciclos < seteo_ciclos) && (seteo_ciclos > 0)){

      digitalWrite(pinValvulaPrincipal, HIGH);

      static int ultimoContador = 0;

      if (contadorPulsos != ultimoContador) {

        Serial.print("Pulsos: ");
        Serial.println(contadorPulsos);
        angulo = (contadorPulsos * 360) / PPR;
        Serial.print("Grados: ");

        Serial.println(angulo);

        if (set_anguloA == set_anguloB){
          digitalWrite(pinValvulaPrincipal, LOW);
          estado_prueba = "Error ambos angulos son iguales";
        } else {

          if (set_anguloA > set_anguloB){

          if((angulo >= set_anguloA) && (prevPos == 1 || prevPos == 3)){
            
            digitalWrite(pin_valvulaA, HIGH);
            digitalWrite(pin_valvulaB, LOW);

            conteo_ciclos = conteo_ciclos + 0.5;
            Serial.print("contador");
            Serial.println(conteo_ciclos);
            prevPos = 2;

          } else if ((angulo <= set_anguloB) && (prevPos == 1 || prevPos == 2)){
              
            digitalWrite(pin_valvulaA, LOW);
            digitalWrite(pin_valvulaB, HIGH);

            conteo_ciclos = conteo_ciclos + 0.5;
            Serial.print("contador");
            Serial.println(conteo_ciclos);
            prevPos = 3;
          }

          } else if (set_anguloA < set_anguloB){

            if((angulo >= set_anguloB) && (prevPos == 1 || prevPos == 3)){
            
            digitalWrite(pin_valvulaA, HIGH);
            digitalWrite(pin_valvulaB, LOW);

            conteo_ciclos = conteo_ciclos + 0.5;
            Serial.print("contador");
            Serial.println(conteo_ciclos);
            prevPos = 2;

          } else if ((angulo <= set_anguloA) && (prevPos == 1 || prevPos == 2)){
              
            digitalWrite(pin_valvulaA, LOW);
            digitalWrite(pin_valvulaB, HIGH);

            conteo_ciclos = conteo_ciclos + 0.5;
            Serial.print("contador");
            Serial.println(conteo_ciclos);
            prevPos = 3;
          }

          }

        }

        ultimoContador = contadorPulsos;
      
      }

    } else {
      conteo_ciclos = 0;
      seteo_ciclos = 0;
      estado_prueba = "finalizado";
      set_anguloA = 0;
      set_anguloB = 0;
      tiempoActualPausado = 0;
      tiempoPausadoAcumulado = 0;
      inicio_prueba = "";
      digitalWrite(pinValvulaPrincipal, LOW);

      /*
      angulo = (contadorPulsos * 360) / PPR;

      if(angulo != 0 && controlFinal == 1){
        digitalWrite(pin_valvulaA, HIGH);
        digitalWrite(pin_valvulaB, LOW);
      } else if(angulo == 0){
        digitalWrite(pin_valvulaA, LOW);
        digitalWrite(pin_valvulaB, LOW);
        controlFinal == 2;
        digitalWrite(pinValvulaPrincipal, LOW);
      }*/
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

                int set_anguloA_copia = doc[1]["mensaje"]["seteoAnguloA"]; 
                set_anguloA = set_anguloA_copia -0; // angulo 2 Podria borrar esto -23
                
                int set_anguloB_copia = doc[1]["mensaje"]["seteoAnguloB"];
                set_anguloB = (set_anguloB_copia -0) * (-1); // angulo 1 -16

                inicio_prueba = doc[1]["mensaje"]["pausarF"].as<String>();  // 20

                tiempo_prueba = 0;
                conteo_ciclos = 0;

                estado_prueba = "Sistema funcionando";
                // Mostrar los valores en el monitor serie
                Serial.println(seteo_ciclos);
                Serial.println(set_anguloA);
                Serial.println(set_anguloB);
                Serial.println(inicio_prueba);

                if(seteo_ciclos != 0){
                  tiempo_inicio = millis();
                  estadoPausa = 1;
                }

            }else if (eventName == "mensajeFlexiones_pausar"){
              JsonObject data = doc[1].as<JsonObject>();
              inicio_prueba = doc[1]["mensaje"]["pausarF"].as<String>();  // 20
              Serial.println(inicio_prueba);

              if(inicio_prueba == "SI"){

                tiempoInicioPausa = millis();
                estado_prueba = "Sistema Pausado";
                estadoPausa = 2;

              } else if (inicio_prueba == "NO") {

                if(estadoPausa == 2){
                tiempoFinPausa = millis();
                tiempoActualPausado = tiempoFinPausa - tiempoInicioPausa;
                tiempoPausadoAcumulado = tiempoPausadoAcumulado + tiempoActualPausado;
                tiempoInicioPausa = 0;
                estado_prueba = "Sistema trabajando";
                estadoPausa = 1;
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
