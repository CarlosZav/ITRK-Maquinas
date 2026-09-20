#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ArduinoJson.h>
//#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <EEPROM.h>
   
#include <Wire.h> 
//#define EEPROM_SIZE 8
 
WiFiMulti WiFiMulti;
SocketIOclient socketIO;

int reductorVelocidad = 20;
 
const int PUL = 19;
const int DIR = 18;
const int ENA = 17;
const int RELE1 = 15;
const int RELE2 = 4;
const int RELE3 = 16;
const int RELE4 = 33;
const int ventosa = 5;
 
 
int tipoPrueba = 0; // 1 para rotaciones y 2 para Flexiones 0 inicial
  
String eventName = "";
String inicio_prueba = "";
String estado_prueba = "Sin iniciar";
 
int estadoPausa = 0;
unsigned long tiempo_prueba = 0;
unsigned long tiempo_inicio = 0;
unsigned long tiempo_actual = 0;
unsigned long messageTimestamp = 0;
 
unsigned long tiempoInicioPausa = 0;
unsigned long tiempoFinPausa = 0;
unsigned long tiempoActualPausado = 0;
unsigned long tiempoPausadoAcumulado = 0;
 
//Variables para funcion secadoras FLEXIONES
float conteoFlexiones = 0;
float ciclosHornos = 0;
float anguloA = 0.0;
float anguloB = 0.0;
float velocidadFPM = 0.0;
unsigned long intervaloPasoFlex = 0.0;
int prevPosFlex = 0;
unsigned long setVelocidad = 0.0;
float pasosCicloFlex = 0.0;
float tiempoPiston1Hornos = 0.0;
float tiempoPiston2Hornos = 0.0;
float tiempoPiston3Hornos = 0.0;
float tiempoPiston4Hornos = 0.0;
 
//Variables para el control del motor a pasos ROTACIONES
const int pasosPorCiclo = 800;
unsigned long tiempoAnterior = 0;
unsigned long intervaloPaso = 0.0; // Microsegundos, ajusta para cambiar velocidad 312.5 para 1 rps
bool estadoPulso = LOW;
bool direccion = HIGH;
unsigned long tiempoInicioDireccion = 0;
int prevPos = 1;
unsigned long RPM = 0;
int numeroRotaciones = 0;
int conteoRotaciones = 0;
int numeroRotacionesCambio = 0;
int conteoRotacionesCambio = 0;
 
//Variables para la calibracion
String sentido = "";
int gradosCalibrar = 0;
int pasosCero = 0;
int contadorPasosCalibracion = 0;
int contadorPasosCopia = 0;
unsigned long tiempoAnteriorCalibrar = 0;
float pasosCalibrar = 0.0;
int intervaloCalibrar = 620;
 
//Variables para relevador (valvulas)
unsigned long tiempoRele = 0;
int estadoRele = 0;  
bool cicloReleCompleto = false;
 
unsigned long duracionRele = 1000;   // Tiempo que cada relevador permanece encendido (en ms)
unsigned long intervaloEntreReles = 1000; // Tiempo entre un relevador y el siguiente (en ms)
 
float fuerzaMedida = 0.0;
float contadorPasos = 0.0;
int pasosPorVuelta = 800;
int velocidadVuelta = 600;
float intervaloPasoFuerza = 5;
unsigned long tiempoControl = 0;

float cuentaAngulo = 0.0;
float anguloFinal = 0.0;

bool estadoVentosa = false;
 
 
#define USE_SERIAL Serial
 
void Task1(void *pvParameters){
 
  while(1){
    socketIO.loop();
    mandar_datos();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
 
}
 
void setup() {
  USE_SERIAL.begin(115200);
 
  USE_SERIAL.setDebugOutput(true);
 
 // EEPROM.begin(EEPROM_SIZE); // Inicializar EEPROM
 
  pinMode(RELE1, OUTPUT);
  pinMode(RELE2, OUTPUT);
  pinMode(RELE3, OUTPUT);
  pinMode(RELE4, OUTPUT);
  digitalWrite(RELE1, LOW);
  digitalWrite(RELE2, LOW);
  digitalWrite(RELE3, LOW);
  digitalWrite(RELE4, LOW);
 
  pinMode(ventosa, OUTPUT);
  digitalWrite(ventosa, LOW);
 
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENA, OUTPUT);
 
  digitalWrite(ENA, LOW); // Habilitar el driver
  digitalWrite(DIR, direccion);
 
  Wire.begin();
 
  xTaskCreatePinnedToCore(Task1, "Task1", 10000, NULL, 1, NULL, 0); // Núcleo 0
 
  conexion_internet();
 
}
 
void loop() {
 
  unsigned long tiempoActual = micros();
 
    if (tipoPrueba == 1) {
      //controlFuerza();
    } else if (tipoPrueba == 2){
      controlFlexiones();
    } else if (tipoPrueba == 3){
      controlCalibrar();
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
            eventName = doc[0].as<String>();
 
            USE_SERIAL.printf("Evento recibido: %s\n", eventName.c_str());
 
            // Comprobar el nombre del evento y obtener los datos correspondientes
            if (eventName == "mensajeHornos"){
 
              // El segundo elemento es el objeto que contiene los datos
              JsonObject data = doc[1].as<JsonObject>();
 
              ciclosHornos= doc[1]["mensaje"]["ciclosHornos"];  // 20
              int anguloACopy = doc[1]["mensaje"]["anguloApertura"];
              setVelocidad = doc[1]["mensaje"]["velocidadHornos"];            
              inicio_prueba = doc[1]["mensaje"]["pausarHornos"].as<String>();  // 20
              tiempoPiston1Hornos = doc[1]["mensaje"]["tiempoPiston1Hornos"];
              tiempoPiston2Hornos = doc[1]["mensaje"]["tiempoPiston2Hornos"];  
              tiempoPiston3Hornos = doc[1]["mensaje"]["tiempoPiston3Hornos"];  
              tiempoPiston4Hornos = doc[1]["mensaje"]["tiempoPiston4Hornos"];    
             
              anguloA = anguloACopy * reductorVelocidad;
 
              tiempo_prueba = 0;
              conteoFlexiones = 0;
 
              estado_prueba = "Sistema funcionando";
 
              // Mostrar los valores en el monitor serie
              Serial.println(ciclosHornos);
              Serial.println(setVelocidad);
              Serial.println(inicio_prueba);
              Serial.println(tiempoPiston1Hornos);
              Serial.println(tiempoPiston2Hornos);
              Serial.print("anguloFinal:  ");
              Serial.println(anguloFinal);
 
              digitalWrite(ventosa, HIGH);
 
              if( (ciclosHornos != 0) && ((anguloFinal != 0)) && setVelocidad != 0){
                tiempo_inicio = millis();
                estadoPausa = 1;
                tiempoInicioDireccion = micros();
                calcularIntervaloPasoFlexiones();
                tipoPrueba = 2;
 
                tiempoPausadoAcumulado = 0; // Reiniciar el tiempo pausado acumulado
              }
 
            } else if (eventName ==  "mensajeHornosPausar"){
 
              JsonObject data = doc[1].as<JsonObject>();
              inicio_prueba = doc[1]["mensaje"]["pausarHornos"].as<String>();  // 20
              Serial.println(inicio_prueba);
 
              if(inicio_prueba == "SI"){
                tiempoInicioPausa = millis();
                estado_prueba = "Sistema Pausado";
                estadoPausa = 2;
 
                digitalWrite(ventosa, HIGH);
 
              } else if (inicio_prueba == "NO") {
 
                if(estadoPausa == 2){
                  tiempoFinPausa = millis();
                  tiempoActualPausado = tiempoFinPausa - tiempoInicioPausa;
                  tiempoPausadoAcumulado = tiempoPausadoAcumulado + tiempoActualPausado;
                  tiempoInicioPausa = 0;
                  estado_prueba = "Sistema trabajando";
                  estadoPausa = 1;
 
                  digitalWrite(ventosa, HIGH);
 
                }
 
              }
            } else if (eventName == "mensajeCalibrarHornos"){

              JsonObject data = doc[1].as<JsonObject>();
              int gradosCalibrarCopy = doc[1]["mensaje"]["gradosCalibrar"];  // 20
              sentido = doc[1]["mensaje"]["sentido"].as<String>(); 

              gradosCalibrar = gradosCalibrarCopy * reductorVelocidad;

              Serial.print("Sentido");
              Serial.println(sentido);
              Serial.print("Grados");
              Serial.println(gradosCalibrar);
              tipoPrueba = 3;

              contadorPasosCalibracion = 0;

              gradosCalibrar = gradosCalibrarCopy * reductorVelocidad;

              pasosCalibrar = (gradosCalibrar  * 800)/360;

              if (sentido == "Horario"){
                cuentaAngulo = cuentaAngulo - gradosCalibrar;
                digitalWrite(DIR, HIGH);
                direccion = HIGH;
              } else if (sentido == "Antihorario"){
                cuentaAngulo = cuentaAngulo + gradosCalibrar;
                digitalWrite(DIR, LOW);
                direccion = LOW;
              } else if (sentido == "EstablecerCero"){
                
                contadorPasosCopia = 0;
                tiempo_prueba = 0;
                pasosCero = 0;
                contadorPasos = 0;
                direccion = LOW;
                tiempoInicioDireccion = 0;
                prevPosFlex = 1;
                setVelocidad = 0;
                anguloA = 0;
                anguloB = 0;
                estado_prueba = "finalizado";
                tiempoActualPausado = 0;
                tiempoPausadoAcumulado = 0;
                cuentaAngulo = 0.0;
                gradosCalibrar = 0.0;

                Serial.println("0 ESTABLECIDO");
 
                DynamicJsonDocument docSend(512);
  
                JsonArray arraySend = docSend.to<JsonArray>();
  
                arraySend.add("calibrarEspConfirmacionHornos");
                JsonObject msg = arraySend.createNestedObject();
  
                msg["conexion"] = "calibrarEspConfirmacionHornos";
  
                String output;
                serializeJson(docSend, output);
                socketIO.sendEVENT(output);
                USE_SERIAL.println(output);

              } else if (sentido == "EstablecerFinal"){
                anguloFinal = cuentaAngulo;
                Serial.print("Angulo  Final:  ");
                Serial.println(anguloFinal/reductorVelocidad);

                if (cuentaAngulo < 0){
                  anguloFinal = cuentaAngulo * -1;
                  direccion = LOW;
                  digitalWrite(DIR, LOW);
                } else {
                  anguloFinal = cuentaAngulo;
                  direccion = HIGH;
                  digitalWrite(DIR, HIGH);
                }

                // poner un if aqui para ver la doreccion a la que se moverá
                //digitalWrite(DIR, LOW); 
                tipoPrueba = 3;
                contadorPasosCalibracion = 0;
                gradosCalibrar = anguloFinal;

                pasosCalibrar = (gradosCalibrar  * 800)/360;

                DynamicJsonDocument docSend(512);
  
                JsonArray arraySend = docSend.to<JsonArray>();
  
                arraySend.add("calibrarEspConfirmacionHornos");
                JsonObject msg = arraySend.createNestedObject();
  
                msg["conexion"] = "calibrarEspConfirmacionHornos";
  
                String output;
                serializeJson(docSend, output);
                socketIO.sendEVENT(output);
                USE_SERIAL.println(output);

              
              } else if ((sentido == "ventosa") && (estadoVentosa == false)){

                digitalWrite(ventosa, HIGH);
                estadoVentosa = !estadoVentosa;

              } else if ((sentido == "ventosa") && (estadoVentosa == true)){

                digitalWrite(ventosa, LOW);
                estadoVentosa = !estadoVentosa;

              } else if (sentido == "apagar1"){
                digitalWrite(RELE1, LOW);
              } else if (sentido == "apagar2"){
                digitalWrite(RELE2, LOW);
              } else if (sentido == "prender1"){
                digitalWrite(RELE1, HIGH);
              } else if (sentido == "prender2"){
                digitalWrite(RELE2, HIGH);
              }
            } 
            // Verificar si las claves existen y obtener los valores con seguridad
            break;
    }
}
 
void mandar_datos(){
 
  if( tipoPrueba == 1){
 
  } else if (tipoPrueba == 2){
 
    uint64_t now = millis();
 
    if(now - messageTimestamp > 500) {
      messageTimestamp = now;
 
      // creat JSON message for Socket.IO (event)
      DynamicJsonDocument doc(1024);
      JsonArray array = doc.to<JsonArray>();
 
      // add evnet name
      // Hint: socket.on('event_name', ....
      array.add("datosEspHornos");
 
      // add payload (parameters) for the event
      JsonObject param1 = array.createNestedObject();
      param1["conteoCiclosHornos"] = conteoFlexiones;  
      param1["estadoHornos"] = estado_prueba;
      param1["tiempoHornos"] = tiempo_prueba;
      param1["velocidadHornos"] = setVelocidad;
      param1["ciclosHornos"] = ciclosHornos;              
 
      // JSON to String (serializion)+
      String output;
      serializeJson(doc, output);
 
      // Send event
      socketIO.sendEVENT(output);
 
      // Print JSON for debugging
      USE_SERIAL.println(output);
    }
   
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
 
    // server address, port and URL0
    socketIO.begin("192.168.0.101", 5000, "/socket.io/?EIO=4"); // 192.168.0.101
    // event handler
    socketIO.onEvent(socketIOEvent);
 
}
  
void calcularIntervaloPasoFlexiones(){
  
  pasosCicloFlex = (anguloFinal * pasosPorCiclo) / 360;

  Serial.print("Pasos Total = ");
  Serial.println(pasosCicloFlex);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           

  intervaloPasoFlex = (60000000 / ( 2 * setVelocidad * pasosCicloFlex )) / 2;

  Serial.print("Intervalo en uS ");
  Serial.println(intervaloPasoFlex);
  
}
 
void controlFlexiones(){

  if(cicloReleCompleto == false){
    rutinaPistones();
  } else if((inicio_prueba == "NO") && ((setVelocidad != 0) && (anguloFinal !=0) && (ciclosHornos != 0))){
    tiempo_actual = millis();
    tiempo_prueba = (tiempo_actual - tiempo_inicio - tiempoPausadoAcumulado) / 1000;

    unsigned long tiempoActual = micros();

    if ((conteoFlexiones < ciclosHornos) && (ciclosHornos != 0)){

      if ((prevPosFlex == 1) && (tiempoActual - tiempoAnterior >= intervaloPasoFlex)){
        
        tiempoAnterior = tiempoActual;
        estadoPulso = !estadoPulso;
        digitalWrite(PUL, estadoPulso);

        if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
          contadorPasos++;
        }

        if(contadorPasos >= pasosCicloFlex / 4){
          digitalWrite(RELE2, LOW);
        }

        if(contadorPasos >= pasosCicloFlex){
          prevPosFlex = 2;
          direccion = !direccion;
          digitalWrite(DIR, direccion);
          contadorPasos = 0;
        }

      } else if ((prevPosFlex == 2 ) && (tiempoActual - tiempoAnterior >= intervaloPasoFlex)){

        tiempoAnterior = tiempoActual;
        estadoPulso = !estadoPulso;
        digitalWrite(PUL, estadoPulso);

        if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
          contadorPasos++;
        }

        if(contadorPasos >= pasosCicloFlex ){
          prevPosFlex = 1;
          direccion = !direccion;
          digitalWrite(DIR, direccion);
          conteoFlexiones ++;
          contadorPasos = 0;
          cicloReleCompleto = false;
          estadoRele = 0;
        }
      }
    }else{

      unsigned long tiempoActual = micros();

      digitalWrite(ventosa, LOW);
      contadorPasos = 0;
      direccion = HIGH;
      tiempoInicioDireccion = 0;
      prevPosFlex = 1;
      setVelocidad = 0;
      anguloA = 0;
      anguloB = 0;
      estado_prueba = "finalizado";
      tiempoActualPausado = 0;
      tiempoPausadoAcumulado = 0;
      estadoRele = 0;
      estadoVentosa = false;
    }
  }
 
}
 
void controlCalibrar(){
 // hola
 
  unsigned long tiempoActualCalibrar = micros();
 
  if ((gradosCalibrar != 0) && (pasosCalibrar != 0)){
    if ((contadorPasosCalibracion < pasosCalibrar) ){
      if (tiempoActualCalibrar - tiempoAnteriorCalibrar >= intervaloCalibrar){
        tiempoAnteriorCalibrar = tiempoActualCalibrar;
        estadoPulso = !estadoPulso;
        digitalWrite(PUL, estadoPulso);
 
        if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
          contadorPasosCalibracion++;
        }
      }
    }
    else {
      gradosCalibrar = 0;
      tiempoActualCalibrar = 0;
      tiempoAnteriorCalibrar = 0;
      contadorPasosCalibracion = 0;

      if(direccion == HIGH){
        direccion = LOW;
        digitalWrite(DIR, LOW);
      } else{
        direccion = HIGH;
        digitalWrite(DIR, HIGH);
      }
    }
  }
}
   
void rutinaPistones(){
  // --- SECUENCIA DE RELEVADORES ANTES DE LAS FLEXIONES ---
  unsigned long tiempoAhoraRele = millis();
 
  switch (estadoRele) {
    case 0:
      tiempoRele = tiempoAhoraRele;
      if(tiempoPiston1Hornos != 0){
        estadoRele = 1;
      } else if (tiempoPiston2Hornos != 0){
        estadoRele = 2;
      } else if (tiempoPiston3Hornos != 0){
        estadoRele = 3;
      } else if (tiempoPiston4Hornos != 0){
        estadoRele = 4;
      } else{
        cicloReleCompleto = true;
      }
      break;
 
    case 1:
      if (tiempoAhoraRele - tiempoRele >= (tiempoPiston1Hornos * 1000)) {
        digitalWrite(RELE1, HIGH);
        tiempoRele = tiempoAhoraRele;
        estadoRele = 10;
        Serial.println("RELE1 ACTIVADO");
      }
      break;
 
    case 10:
      if (tiempoAhoraRele - tiempoRele >= duracionRele) {
        digitalWrite(RELE1, LOW);
        tiempoRele = tiempoAhoraRele;
       
        if(tiempoPiston2Hornos != 0){
          estadoRele = 2;
        } else if (tiempoPiston3Hornos != 0){
          estadoRele = 3;
        } else if (tiempoPiston4Hornos != 0){
          estadoRele = 4;
        } else{
          cicloReleCompleto = true;
        }
 
        Serial.println("RELE1 DESACTIVADO");
      }
      break;
 
    case 2:
      if (tiempoAhoraRele - tiempoRele >= (tiempoPiston2Hornos*1000)) {
        digitalWrite(RELE2, HIGH);
        tiempoRele = tiempoAhoraRele;
        estadoRele = 20;
        Serial.println("RELE2 ACTIVADO");
      }
      break;
 
    case 20:
      if (tiempoAhoraRele - tiempoRele >= duracionRele) {
        digitalWrite(RELE2, HIGH);
        tiempoRele = tiempoAhoraRele;
 
        if(tiempoPiston3Hornos != 0){
          estadoRele = 3;
        } else if (tiempoPiston4Hornos != 0){
          estadoRele = 4;
        } else{
          cicloReleCompleto = true;
        }
  
        Serial.println("RELE2 DESACTIVADO");
      }
      break;
 
    case 3:
      if (tiempoAhoraRele - tiempoRele >= (tiempoPiston3Hornos*1000)) {
        digitalWrite(RELE3, HIGH);
        tiempoRele = tiempoAhoraRele;
        estadoRele = 30;
        Serial.println("RELE3 ACTIVADO");
      }
      break;
 
    case 30:
      if (tiempoAhoraRele - tiempoRele >= duracionRele) {
        digitalWrite(RELE3, LOW);
        tiempoRele = tiempoAhoraRele;
 
        if(tiempoPiston4Hornos != 0){
          estadoRele = 4;
        } else{
          cicloReleCompleto = true;
        }
 
        Serial.println("RELE3 DESACTIVADO");
      }
      break;
 
    case 4:
      if (tiempoAhoraRele - tiempoRele >= (tiempoPiston4Hornos*1000)) {
        digitalWrite(RELE4, HIGH);
        tiempoRele = tiempoAhoraRele;
        estadoRele = 40;
        Serial.println("RELE4 ACTIVADO");
      }
      break;
 
    case 40:
      if (tiempoAhoraRele - tiempoRele >= duracionRele) {
        digitalWrite(RELE4, LOW);
        cicloReleCompleto = true; // Terminado, pasar a la flexión
        Serial.println("RELE4 DESACTIVADO");
      }
      break;
  }
 
}