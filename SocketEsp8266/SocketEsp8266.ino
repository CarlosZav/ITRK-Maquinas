#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
 
const char* ssid = "Carlos galaxy s10";        // Cambia con tu SSID
const char* password = "ijus4866"; // Cambia con tu contraseña
const char* serverUrl = "http://192.168.97.162:5000/obtener-sensor-data-esp";
const char* serverUrl_ciclos_data = "http://192.168.97.162:5000/obtener-ciclos-data-esp";
const char* serverUrl_obtener_datos = "http://192.168.97.162:5000/enviar-data-esp";

// Pin al que está conectado el SSR
int ssrPin = 5;  // Pin GPIO5 en ESP8266

//Pin y variable del sensor de corriente
int sensorPin = A0;
int sensorValue = 0;

// Variables para controlar el tiempo de conmutación
unsigned long tiempoEncendido = 0;  // Tiempo de encendido (valor predeterminado 5 segundos)
unsigned long tiempoApagado = 0;    // Tiempo de apagado (valor predeterminado 3 segundos)
unsigned long tiempoActual = 0;
unsigned long ultimaConmutacion = 0;
bool ssrEncendido = false;

int encendidoDia = 0, encendidoHora = 0, encendidoMinuto = 0, encendidoSegundo = 0;
int apagadoDia = 0, apagadoHora = 0, apagadoMinuto = 0, apagadoSegundo = 0;
int numCiclos = 0, conteoCiclos = 0;

int ciclos = 0;
int ciclos_transcurridos = 0;
int ciclos_faltantes = 0;
String estado_SSR = "prendido";
 
WiFiClient wifiClient;
HTTPClient http;
 
void setup() {

      // Inicializar el pin del SSR como salida
  pinMode(ssrPin, OUTPUT);
  digitalWrite(ssrPin, LOW);  // Asegurarse de que el SSR comience apagado
    
  Serial.begin(115200);
   WiFi.begin(ssid, password);
   
    // Espera hasta que la ESP8266 se conecte al WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando al WiFi...");
  }

  Serial.println("Conectado al WiFi");
}
 
void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        // Nota: ahora pasamos 'wifiClient' como primer argumento
        datos_sensor();
        datos_ciclos();
        obtener_datos();
    }
 
    delay(1000); // Envía datos cada segundo
}

void datos_sensor(){
  http.begin(wifiClient, serverUrl);
    http.addHeader("Content-Type", "application/json");
 
    // Simulamos un valor de sensor
    //sensorValue = sensorValue +1;
 
    float sensorValue = 25.5;
    // Crear el JSON manualmente como un string
    String jsonString = "{\"value\": ";
    jsonString += sensorValue;  // Agregar el valor del sensor
    jsonString += "}";
 
    // Imprimir el JSON en el monitor serie
    Serial.println(jsonString);
    
    // Crea el JSON con el valor del sensor
    //String jsonData = "{\"value\": " + String(sensorValue) + "}";
       
    int httpResponseCode = http.POST(jsonString);
 
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Respuesta del servidor: " + response);
    } else {
      Serial.println("Error en la solicitud POST");
    }
 
    http.end();
}

void datos_ciclos(){
  http.begin(wifiClient, serverUrl_ciclos_data);
    http.addHeader("Content-Type", "application/json");
 
    // Crear el JSON manualmente como un string
    String jsonString = "{";
    jsonString += "\"ciclos\": " + String(ciclos) + ",";
    jsonString += "\"ciclos_transcurridos\": " + String(ciclos_transcurridos) + ",";
    jsonString += "\"ciclos_faltantes\": " + String(ciclos_faltantes) + ",";
    jsonString += "\"estado\":  \"" + estado_SSR + "\"";
    jsonString += "}";
 
    // Imprimir el JSON en el monitor serie
    Serial.println(jsonString);
     
    // Crea el JSON con el valor del sensor
    //String jsonData = "{\"value\": " + String(sensorValue) + "}";
       
    int httpResponseCode = http.POST(jsonString);
 
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Respuesta del servidor: " + response);
    } else {
      Serial.println("Error en la solicitud POST");
    }
 
  http.end();
}

void obtener_datos(){
  // Iniciar la conexión al servidor
  http.begin(wifiClient, serverUrl_obtener_datos);

  // Hacer una solicitud GET
  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    String payload = http.getString();
    Serial.println("Respuesta del servidor:");
    Serial.println(payload);

    // Analizar el JSON usando ArduinoJson
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (!error) {
      // Extraer datos del JSON
      numCiclos = doc["set_ciclos"];
      tiempoApagado = doc["set_tiempo_apagado"];
      tiempoEncendido = doc["set_tiempo_encendido"];

      // Imprimir los datos recibidos
      Serial.println("Datos recibidos:");
      Serial.print("Ciclos seteados: ");
      Serial.println(numCiclos);
      Serial.print("Tiempo apagado seteo: ");
      Serial.println(tiempoApagado);
      Serial.print("Tiempo prendido seteo: ");
      Serial.println(tiempoEncendido);
    } else {
      Serial.println("Error al analizar JSON");
    }
  } else {
    Serial.print("Error en la solicitud GET: ");
    Serial.println(httpResponseCode);
  }

  // Terminar la conexión HTTP
  http.end();
}

void control_SSR(){
  // Obtener el tiempo actual
  tiempoActual = millis();

  //Apagado hasta que se mande un valor por el usuario
  if((tiempoEncendido > 0) && (tiempoApagado > 0 ) && (conteoCiclos < numCiclos) && (numCiclos > 0)){


      unsigned long hola = tiempoActual - ultimaConmutacion;

    // Verificar si ha pasado el tiempo para conmutar el estado del SSR
    if (ssrEncendido) {
       // Si está encendido y ha pasado el tiempo de encendido
      if (hola >= tiempoEncendido) {
        digitalWrite(ssrPin, LOW);  // Apagar el SSR
        ssrEncendido = false;  // Cambiar el estado a apagado
        Serial.println("APAGADOOOO");
        ultimaConmutacion = tiempoActual;  // Registrar el tiempo de la última conmutación
        conteoCiclos++;
        Serial.print("Ciclos: ");
        Serial.println(conteoCiclos);
        Serial.println(numCiclos);
      }
    } 
    else {

      // Si está apagado y ha pasado el tiempo de apagado
      if ((hola >= tiempoApagado)){
        digitalWrite(ssrPin, HIGH);  // Encender el SSR
        ssrEncendido = true;  // Cambiar el estado a encendido
        Serial.println("ENCENDIDOOOOO");
        ultimaConmutacion = tiempoActual;  // Registrar el tiempo de la última conmutación
            //  Aumentar +1 el conteo de ciclo
      }
    }

  } else if (conteoCiclos >= numCiclos){
    conteoCiclos = 0;
    numCiclos = 0;
    tiempoEncendido = 0;
    tiempoApagado = 0;
    ssrEncendido = false;
    digitalWrite(ssrPin, LOW);  // Apagar el SSR
  }
}
