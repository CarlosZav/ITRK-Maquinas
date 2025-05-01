#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <EEPROM.h>

//#define EEPROM_SIZE 8

WiFiMulti WiFiMulti;
SocketIOclient socketIO;

const int PUL = 18;
const int DIR = 19;
const int ENA = 5;

int tipoPrueba = 0; // 1 para rotaciones y 2 para Flexiones 0 inicial

//Variables para revoluciones funcion
float setRevolucionesCambio = 0;
float setRevoluciones = 0;
float conteoRevoluciones = 0;
int revolucionesTotales = 0;

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
int conteoFlexiones = 0;
int flexionesSecadoras = 0;
float anguloA = 0.0;
float anguloB = 0.0;
float velocidadFPM = 0.0;
unsigned long intervaloPasoFlex = 0.0;
int prevPosFlex = 0;
unsigned long setVelocidad = 0.0;
float pasosAnguloA = 0.0;
float pasosAnguloB = 0.0;
float pasosCicloFlex = 0.0;

//Variables para el control del motor a pasos ROTACIONES
const int pasosPorCiclo = 1600;
unsigned long tiempoAnterior = 0;
unsigned long intervaloPaso = 0.0; // Microsegundos, ajusta para cambiar velocidad 312.5 para 1 rps
bool estadoPulso = LOW;
int contadorPasos = 0;
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
int intervaloCalibrar = 625;

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

  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENA, OUTPUT);

  digitalWrite(ENA, LOW); // Habilitar el driver
  digitalWrite(DIR, direccion);

  //contadorPasosCopia = EEPROM.read(0);
  //Serial.print("dato guardado");
 // Serial.println(contadorPasosCopia);

  // Crear tareas (hilos) con FreeRTOS
  xTaskCreatePinnedToCore(Task1, "Task1", 10000, NULL, 1, NULL, 0); // Núcleo 0

  conexion_internet();

}

void loop() {

  unsigned long tiempoActual = micros();

  /*if (contadorPasosCopia != 0 && tipoPrueba == 0){ // Aquí modificarIntervaloPaso
    if (tiempoActual - tiempoAnterior >= 625){
      if (contadorPasosCopia < 0){
        digitalWrite(DIR, HIGH);
        tiempoAnterior = tiempoActual;
        estadoPulso = !estadoPulso;
        digitalWrite(PUL, estadoPulso);

        if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
          contadorPasosCopia++;
        }
      } else if (contadorPasosCopia > 0){
        digitalWrite(DIR, LOW);
        tiempoAnterior = tiempoActual;
        estadoPulso = !estadoPulso;
        digitalWrite(PUL, estadoPulso);

        if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
          contadorPasosCopia--;
        }
      }
    }
  } else {*/
    if (tipoPrueba == 1) {
      controlRotaciones();
    } else if (tipoPrueba == 2){
      controlFlexiones();
    } else if (tipoPrueba == 3){
      controlCalibrar();
    }
  //}
}

void controlRotaciones(){

  if((inicio_prueba == "NO") && ((RPM != 0) && (numeroRotacionesCambio !=0) && (numeroRotaciones != 0))){

    unsigned long tiempoActual = micros();

    tiempo_actual = millis();
    tiempo_prueba = (tiempo_actual - tiempo_inicio - tiempoPausadoAcumulado) / 1000;

    if (conteoRotaciones < numeroRotaciones){

      if ((prevPos == 1) && (tiempoActual - tiempoAnterior >= intervaloPaso)){

        tiempoAnterior = tiempoActual;
        estadoPulso = !estadoPulso;
        digitalWrite(PUL, estadoPulso);

        if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
          contadorPasos++;
          contadorPasosCopia ++;
        }

        if(contadorPasos >= pasosPorCiclo){
          //prevPos = 2;
          //direccion = !direccion;
          //digitalWrite(DIR, direccion);
          conteoRotaciones ++;
          contadorPasos = 0;
        }

        if(conteoRotaciones - conteoRotacionesCambio >= numeroRotacionesCambio){
          prevPos = 2;
          direccion = !direccion;
          digitalWrite(DIR, direccion);
          conteoRotacionesCambio = conteoRotaciones;
        }

      } else if ((prevPos == 2 ) && (tiempoActual - tiempoAnterior >= intervaloPaso)){

        tiempoAnterior = tiempoActual;
        estadoPulso = !estadoPulso;
        digitalWrite(PUL, estadoPulso);

        if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
          contadorPasos++;
          contadorPasosCopia--;
        }

        if(contadorPasos >= pasosPorCiclo){
          //prevPos = 1;
          //direccion = !direccion;
          //digitalWrite(DIR, direccion);
          conteoRotaciones ++;
          contadorPasos = 0;
        }

        if(conteoRotaciones - conteoRotacionesCambio>= numeroRotacionesCambio){
          prevPos = 1;
          direccion = !direccion;
          digitalWrite(DIR, direccion);
          conteoRotacionesCambio = conteoRotaciones;
        }
      }
    }
    else{

      unsigned long tiempoActual = micros();

      if (contadorPasosCopia != 0 && tipoPrueba == 0){ // Aquí modificarIntervaloPaso
        if (tiempoActual - tiempoAnterior >= intervaloPaso){
          if (contadorPasosCopia < 0){
            digitalWrite(DIR, HIGH);
            tiempoAnterior = tiempoActual;
            estadoPulso = !estadoPulso;
            digitalWrite(PUL, estadoPulso);

            if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
              contadorPasosCopia++;
              //EEPROM.write(0, contadorPasosCopia); // Escribir el nuevo valor
              //EEPROM.commit(); // Guardar cambios en la memoria flash
            }
          } else if (contadorPasosCopia > 0){
            digitalWrite(DIR, LOW);
            tiempoAnterior = tiempoActual;
            estadoPulso = !estadoPulso;
            digitalWrite(PUL, estadoPulso);

            if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
              contadorPasosCopia--;
              //EEPROM.write(0, contadorPasosCopia); // Escribir el nuevo valor
              //EEPROM.commit(); // Guardar cambios en la memoria flash
            }
          }
        }
      } else {

        contadorPasos = 0;
        direccion = HIGH;
        tiempoInicioDireccion = HIGH;
        prevPos = 1;
        RPM = 0;
        numeroRotaciones = 0;
        numeroRotacionesCambio = 0;
        conteoRotacionesCambio = 0;
        estado_prueba = "finalizado";
        tiempoActualPausado = 0;
        tiempoPausadoAcumulado = 0;
      }
      
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
            eventName = doc[0].as<String>();

            USE_SERIAL.printf("Evento recibido: %s\n", eventName.c_str());

            // Comprobar el nombre del evento y obtener los datos correspondientes
            if (eventName == "mensajeSecadorasRot") {

                // El segundo elemento es el objeto que contiene los datos
                JsonObject data = doc[1].as<JsonObject>();

                numeroRotaciones = doc[1]["mensaje"]["revolucionesSecadoras"];  // 20
                numeroRotacionesCambio = doc[1]["mensaje"]["revCambioSecadoras"]; 
                RPM = doc[1]["mensaje"]["velocidadRevoluciones"];             
                inicio_prueba = doc[1]["mensaje"]["pausarSecadorasRot"].as<String>();  // 20

                tiempo_prueba = 0;
                //conteoRotaciones = 0;

                estado_prueba = "Sistema funcionando";

                // Mostrar los valores en el monitor serie
                Serial.println(numeroRotaciones);
                Serial.println(numeroRotacionesCambio);
                Serial.println(RPM);
                Serial.println(inicio_prueba);

                conteoRotaciones = 0;

                if( (numeroRotaciones != 0) && (numeroRotacionesCambio != 0) && (RPM != 0) ){
                  tiempo_inicio = millis();
                  estadoPausa = 1;
                  tiempoInicioDireccion = micros();
                  calcularIntervaloPaso();
                  tipoPrueba = 1;
                  tiempoPausadoAcumulado = 0; // Reiniciar el tiempo pausado acumulado
                }

            }else if (eventName == "mensajeSecadorasRotPausar"){
              JsonObject data = doc[1].as<JsonObject>();
              inicio_prueba = doc[1]["mensaje"]["pausarSecadorasRot"].as<String>();  // 20
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

            } else if (eventName == "mensajeSecadorasFlex"){

              // El segundo elemento es el objeto que contiene los datos
              JsonObject data = doc[1].as<JsonObject>();

              flexionesSecadoras= doc[1]["mensaje"]["flexionesSecadoras"];  // 20
              anguloA = doc[1]["mensaje"]["anguloA"]; 
              anguloB = doc[1]["mensaje"]["anguloB"];
              setVelocidad = doc[1]["mensaje"]["setVelocidadSecadorasFlex"];             
              inicio_prueba = doc[1]["mensaje"]["pausarSecadorasFlex"].as<String>();  // 20

              tiempo_prueba = 0;
              conteoFlexiones = 0;

              estado_prueba = "Sistema funcionando";

              // Mostrar los valores en el monitor serie
              Serial.println(flexionesSecadoras);
              Serial.println(anguloA);
              Serial.println(anguloB);
              Serial.println(setVelocidad);
              Serial.println(inicio_prueba);

              if( (flexionesSecadoras != 0) && (anguloA != 0) && (anguloB != 0) && setVelocidad != 0){
                tiempo_inicio = millis();
                estadoPausa = 1;
                tiempoInicioDireccion = micros();
                calcularIntervaloPasoFlexiones();
                tipoPrueba = 2;

                tiempoPausadoAcumulado = 0; // Reiniciar el tiempo pausado acumulado
              }

            } else if (eventName ==  "mensajeSecadorasFlexPausar"){

              JsonObject data = doc[1].as<JsonObject>();
              inicio_prueba = doc[1]["mensaje"]["pausarSecadorasFlex"].as<String>();  // 20
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
            } else if (eventName == "mensajeCalibrarSecadoras"){
              JsonObject data = doc[1].as<JsonObject>();
              gradosCalibrar = doc[1]["mensaje"]["gradosCalibrar"];  // 20
              sentido = doc[1]["mensaje"]["sentido"].as<String>(); 

              Serial.print("Sentido");
              Serial.println(sentido);
              Serial.print("Grados");
              Serial.println(gradosCalibrar);
              tipoPrueba = 3;

              contadorPasosCalibracion = 0;

              pasosCalibrar = (gradosCalibrar * 1600)/360;

              if (sentido == "Horario"){
                digitalWrite(DIR, LOW);
              } else if (sentido == "Antihorario"){
                digitalWrite(DIR, HIGH);
              } else if (sentido == "EstablecerCero"){
                contadorPasosCopia = 0;
              }
          
            }
            // Verificar si las claves existen y obtener los valores con seguridad
            break;
    }
}

void mandar_datos(){

  if( tipoPrueba == 1){
    uint64_t now = millis();

    if(now - messageTimestamp > 500) {
      messageTimestamp = now;

      // creat JSON message for Socket.IO (event)
      DynamicJsonDocument doc(1024);
      JsonArray array = doc.to<JsonArray>();

      // add evnet name
      // Hint: socket.on('event_name', ....
      array.add("datosEspSecadorasRot");

      // add payload (parameters) for the event
      JsonObject param1 = array.createNestedObject();
      param1["conteo_revSecadorasRot"] = conteoRotaciones;   
      param1["estado_pruebaSecadorasRot"] = estado_prueba;
      param1["tiempo_pruebaSecadorasRot"] = tiempo_prueba;
      param1["velocidad_SecadorasRot"] = RPM;
      param1["setRevSecadorasRot"] = numeroRotaciones;               
  
      // JSON to String (serializion)
      String output;
      serializeJson(doc, output);

      // Send event
      socketIO.sendEVENT(output);

      // Print JSON for debugging
      USE_SERIAL.println(output);
    }

  } else if (tipoPrueba == 2){

    uint64_t now = millis();

    if(now - messageTimestamp > 500) {
      messageTimestamp = now;

      // creat JSON message for Socket.IO (event)
      DynamicJsonDocument doc(1024);
      JsonArray array = doc.to<JsonArray>();

      // add evnet name
      // Hint: socket.on('event_name', ....
      array.add("datosEspSecadorasFlex");

      // add payload (parameters) for the event
      JsonObject param1 = array.createNestedObject();
      param1["conteoFlexSecadoras"] = conteoFlexiones;   
      param1["estadoSecadorasFlex"] = estado_prueba;
      param1["tiempoSecadorasFlex"] = tiempo_prueba;
      param1["velocidadFlexiones"] = setVelocidad;
      param1["flexionesSecadoras"] =flexionesSecadoras;               
  
      // JSON to String (serializion)
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
    WiFiMulti.addAP("victus", "12345678");

    //WiFi.disconnect();
    while(WiFiMulti.run() != WL_CONNECTED) {
        delay(100);
    }

    String ip = WiFi.localIP().toString();
    USE_SERIAL.printf("[SETUP] WiFi Connected %s\n", ip.c_str());

    // server address, port and URL
    socketIO.begin("10.224.55.216", 5000, "/socket.io/?EIO=4"); // 192.168.0.101
    // event handler
    socketIO.onEvent(socketIOEvent);
}

void calcularIntervaloPaso(){
  intervaloPaso = ((60000000) / (RPM * pasosPorCiclo)) / 2;
  Serial.println(intervaloPaso);
}

void calcularIntervaloPasoFlexiones(){

  pasosAnguloA = (anguloA*pasosPorCiclo)/360; //Equivalente a pasos por revolcuiones = pasosPorCiclo
  Serial.print("Pasos Angulo A = ");
  Serial.println(pasosAnguloA);
 
  pasosAnguloB = (anguloB*pasosPorCiclo)/360;
   Serial.print("Pasos Angulo B = ");
  Serial.println(pasosAnguloB);

  pasosCicloFlex = pasosAnguloA + pasosAnguloB; 
   Serial.print("Pasos Total = ");
  Serial.println(pasosCicloFlex);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           

  intervaloPasoFlex = 60000000 / ( 2 * setVelocidad * pasosCicloFlex );

   Serial.print("Intervalo en uS ");
  Serial.println(intervaloPasoFlex);
}

void controlFlexiones(){

  if((inicio_prueba == "NO") && ((setVelocidad != 0) && (anguloA !=0) && (anguloB !=0) && (flexionesSecadoras != 0))){
    tiempo_actual = millis();
    tiempo_prueba = (tiempo_actual - tiempo_inicio - tiempoPausadoAcumulado) / 1000;

    unsigned long tiempoActual = micros();

    if ((conteoFlexiones < flexionesSecadoras) && (flexionesSecadoras != 0)){

      if ((prevPosFlex == 0) && (tiempoActual - tiempoAnterior >= intervaloPasoFlex)){

        tiempoAnterior = tiempoActual;
        estadoPulso = !estadoPulso;
        digitalWrite(PUL, estadoPulso);

        if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
          contadorPasos++;
        }

        if(contadorPasos >= pasosAnguloA){
          prevPosFlex = 2;
          direccion = !direccion;
          digitalWrite(DIR, direccion);
          conteoFlexiones ++;
          contadorPasos = 0;
        }

      } else if ((prevPosFlex == 1) && (tiempoActual - tiempoAnterior >= intervaloPasoFlex)){

        tiempoAnterior = tiempoActual;
        estadoPulso = !estadoPulso;
        digitalWrite(PUL, estadoPulso);

        if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
          contadorPasos++;
        }

        if(contadorPasos >= pasosCicloFlex){
          prevPosFlex = 2;
          direccion = !direccion;
          digitalWrite(DIR, direccion);
          conteoFlexiones ++;
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
          prevPos = 1;
          direccion = !direccion;
          digitalWrite(DIR, direccion);
          conteoFlexiones ++;
          contadorPasos = 0;
        }
      }
    }else{

      unsigned long tiempoActual = micros();

      if (contadorPasosCopia != 0 && tipoPrueba == 0){ // Aquí modificarIntervaloPaso
        if (tiempoActual - tiempoAnterior >= intervaloPaso){
          if (contadorPasosCopia < 0){
            digitalWrite(DIR, HIGH);
            tiempoAnterior = tiempoActual;
            estadoPulso = !estadoPulso;
            digitalWrite(PUL, estadoPulso);

            if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
              contadorPasosCopia++;
              //EEPROM.write(0, contadorPasosCopia); // Escribir el nuevo valor
              //EEPROM.commit(); // Guardar cambios en la memoria flash
            }
          } else if (contadorPasosCopia > 0){
            digitalWrite(DIR, LOW);
            tiempoAnterior = tiempoActual;
            estadoPulso = !estadoPulso;
            digitalWrite(PUL, estadoPulso);

            if (!estadoPulso) { // Contar solo flancos bajos para evitar doble conteo
              contadorPasosCopia--;
              //EEPROM.write(0, contadorPasosCopia); // Escribir el nuevo valor
              //EEPROM.commit(); // Guardar cambios en la memoria flash
            }
          }
        }
      } else{
        contadorPasos = 0;
        direccion = HIGH;
        tiempoInicioDireccion = 0;
        prevPosFlex = 0;
        setVelocidad = 0;
        flexionesSecadoras = 0;
        anguloA = 0;
        anguloB = 0;
        estado_prueba = "finalizado";
        tiempoActualPausado = 0;
        tiempoPausadoAcumulado = 0;
      }
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
    }
  }
}


