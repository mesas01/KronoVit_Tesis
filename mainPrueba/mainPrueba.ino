#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "SparkFun_BMA400_Arduino_Library.h"
#include <RTClib.h>
#include <Adafruit_GC9A01A.h>
#include <Adafruit_GFX.h>
#include <time.h>
#include "esp_sleep.h"

//---------------------------------WIFI-----------------------------------------------------------------------------------
const char* ssid = "SM-DESKTOP"; //"SMN"; //"SLB";
const char* password = "12345678";//"speedemon";//"12345678";
//------------------------------------THINGSBOARD--------------------------------------------------------------------------
const char* mqtt_server = "demo.thingsboard.io";
const char* token = "eng5yt35919bjyycq3i7";
const char* topic = "v1/devices/me/telemetry";
//---------------------------------WIFI----------------------------------------------------------------------------------
WiFiClient espClient;
PubSubClient client(espClient);
//-----------------------------------BMA400------------------------------------------------------------------------------
BMA400 accelerometer;
uint8_t i2cAddress = BMA400_I2C_ADDRESS_DEFAULT;
//---------------------------------Configuración para NTP tomar hora de servidor-----------------------------------------
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -18000; // UTC-5 para Colombia
const int   daylightOffset_sec = 0; // 0 pq No hay horario de verano en Colombia
//---------------------------------Configuración que mida V de bateria------------------------------------------------
const unsigned long batteryReadInterval = 60000; // Intervalo de 1 minuto (60.000 milisegundos)
unsigned long lastBatteryReadMillis = 0; // Guardará la última vez que se leyó la batería
float porcentajeBateria = 0; // Almacena el porcentaje de batería
//---------------------------------Configuración que mida met cada minuto------------------------------------------------
unsigned long lastActivityTime = 0; // Guarda la última vez que se ejecutó la función
const long interval = 60000; // Intervalo de tiempo (60 segundos o 60000 milisegundos)
//---------------------------------BATERIA-------------------------------------------------------------------------
#define BATTERY_PIN 25 // Cambia esto por el pin correcto conectado a B_MON
#define ADC_RESOLUTION 12 // Resolución del ADC de 12 bits
#define ADC_ATTENUATION ADC_0db // Atenuación de 0 dB
//---------------------------------RTC & DISPLAY-------------------------------------------------------------------------
RTC_DS3231 rtc;
#define TFT_CS     5
#define TFT_RST    33
#define TFT_DC     12
//---------------------------------suspension de pantalla-------------------------------------------------------------------------
#define PIN_LED 9 // Asumiendo que el PIN_LED es el IO9 pin que controla el display
#define BTN2 10 // El pin donde se conecta el botón BTN2 alerta
#define BTN3 13 // El pin donde se conecta el botón BTN3 encender pantalla

bool pantallaEncendida = false;
unsigned long tiempoPantallaEncendida = 0;
const unsigned long tiempoEncendido = 15000; // 15 segundos en milisegundos


Adafruit_GC9A01A tft = Adafruit_GC9A01A(TFT_CS, TFT_DC, TFT_RST);

// Nuevas variables para el manejo de promedios para transmitir cada 30 min
int contadorMinutos = 0;
float METsPorPeriodo[3] = {0, 0, 0}; // Almacena los METs de cada periodo de 10 minutos
int periodoActual = 0; // Indica el periodo actual de 10 minutos dentro de los 30 minutos
bool actividadSedentaria = true;// Indica si en algún periodo se superó el umbral de 1.5 METs
bool banderaActividad = false; // Indica si en algún periodo se superó el umbral de 1.5 METs


//---------------------------FUNCIONES------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  //pruebas
  analogReadResolution(ADC_RESOLUTION);
  analogSetPinAttenuation(BATTERY_PIN, ADC_ATTENUATION);//
  
  

  pinMode(BTN2, INPUT);  // Configura BTN2 como entrada
  pinMode(BTN3, INPUT);
  pinMode(PIN_LED, OUTPUT); // Configura PIN_LED como salida
  digitalWrite(PIN_LED, LOW); // Asegúrate de que la pantalla comience apagada

  if (!rtc.begin()) {
    Serial.println("No se encontró el RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC perdió energía, se establece la fecha y hora actuales");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(GC9A01A_BLACK);

  if (accelerometer.beginI2C(i2cAddress) != BMA400_OK) {
    Serial.println("Error: BMA400 not connected, check wiring and I2C address!");
    delay(1000);
  }
  Serial.println("BMA400 connected!");

  setup_wifi(); // Connect to WiFi and sync time
  client.setServer(mqtt_server, 1883);
}


void syncNTPTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Fallo al obtener la hora");
    return;
  }
  Serial.println("Hora sincronizada con servidor NTP");

  // Crear instancia DateTime con la hora NTP obtenida
  DateTime newTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

  // Verificar si la instancia DateTime es válida
  if (newTime.year() >= 2020) { // Ejemplo de verificación, ajusta según tu necesidad
    Serial.println("Nueva hora del RTC válida, procediendo a ajustar...");
    rtc.adjust(newTime);
    Serial.println("RTC ajustado con la nueva hora.");
  } else {
    Serial.println("La nueva hora del RTC no es válida, ajuste no realizado.");
  }
}


//Funcion para la conexion wifi
void setup_wifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) { // Intenta hasta 10 veces
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");
    syncNTPTime(); // Sincroniza solo si WiFi está conectado
  } else {
    Serial.println("Failed to connect to WiFi.");
  }
}


//Funcion para reconectar wifi en caso de perdida
void reconnect() {
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", token, NULL)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // No bloquea el loop; permite que se intente nuevamente en la próxima iteración
    }
  }
}


//Funcion para enviar los datos

// Función para enviar los datos
void enviarDatos(float MET, int periodo, bool finalizarCiclo) {
    DateTime now = rtc.now(); // Obtener la hora actual del RTC
    String timestamp = String(now.year(), DEC) + "-" +
                       String(now.month(), DEC) + "-" +
                       String(now.day(), DEC) + " T-> " +
                       String(now.hour(), DEC) + ":" +
                       String(now.minute(), DEC) + ":" +
                       String(now.second(), DEC);

    if (!finalizarCiclo) {
        // Enviar MET de un periodo específico junto con la hora
        String payload = "{\"METsPeriodo" + String(periodo + 1) + "\": " + String(MET, 2) + ", \"timestamp\": \"" + timestamp + "\"}";
        if (client.publish(topic, (char*)payload.c_str())) {
            Serial.println("Datos enviados exitosamente: " + payload);
        } else {
            Serial.println("Error al enviar datos");
        }
    } else {
        // En el final del ciclo, evaluar si la actividad sedentaria es verdadera
        if (actividadSedentaria) {
            // Si es actividad sedentaria, enviar el MET como 1.4 y también el MET del periodo 3
            String payloadSedentario = "{\"MET\": \"1.4\", \"timestamp\": \"" + timestamp + "\"}";
            if (client.publish(topic, (char*)payloadSedentario.c_str())) {
                Serial.println("Datos enviados exitosamente: " + payloadSedentario);
            } else {
                Serial.println("Error al enviar datos de actividad sedentaria");
            }
        }

        // Enviar siempre el MET del periodo 3 al finalizar el ciclo
        String payloadPeriodo = "{\"METsPeriodo3\": " + String(METsPorPeriodo[2], 2) + ", \"timestamp\": \"" + timestamp + "\"}";
        if (client.publish(topic, (char*)payloadPeriodo.c_str())) {
            Serial.println("Datos enviados exitosamente: " + payloadPeriodo);
        } else {
            Serial.println("Error al enviar datos del periodo 3");
        }
        
        // Resetear para el próximo ciclo de 30 minutos
        for (int i = 0; i < 3; i++){
          METsPorPeriodo[i] = 0;
        } 
        periodoActual = 0;
        actividadSedentaria = true; // Resetear el estado de actividad sedentaria para el nuevo ciclo
    }
}



float calculateBatteryPercentage(float voltage) {
  const float maxVolt = 0.22; // Voltaje correspondiente al 100% de carga
  const float minVolt = 0.14; // Voltaje correspondiente al 1% de carga
  return (voltage - minVolt) / (maxVolt - minVolt) * 100;
}


//Función para medir el nivel de la batería y enviarlo a ThingsBoard cada minuto
void enviarNivelBateriaAThingsBoard() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastBatteryReadMillis >= batteryReadInterval) {
    lastBatteryReadMillis = currentMillis; // Actualiza la última vez que se leyó la batería

    // Configurar la resolución del ADC y la atenuación
    analogReadResolution(12); // 12 bits de resolución
    analogSetAttenuation(ADC_0db); // Atenuación de 0 dB

    uint32_t adcValue = 0;
    for (int i = 0; i < 16; i++) {
      adcValue += analogRead(BATTERY_PIN); // Asegúrate de que BATTERY_PIN es el pin correcto
    }
    adcValue /= 16; // Promedio de las lecturas para reducir el ruido

    // Convertir el valor del ADC a voltaje para B_MON
    float voltageBMON = (float)adcValue * (1.1 / 4095.0); // Convierte el valor del ADC a voltaje en B_MON

    // Mapear el voltaje de B_MON al rango de voltaje de la batería
    float voltajeBateria = mapf(voltageBMON, 0.14, 0.20, 3.0, 4.2);

    // Calcular el porcentaje de batería
    float porcentajeBateria = mapf(voltageBMON, 0.14, 0.20, 1, 100);

    // Preparar el payload JSON para enviar
    String payload = "{\"BMON\": " + String(voltageBMON, 2) + ", \"voltajeBateria\": " + String(voltajeBateria, 2) + ", \"porcentajeBateria\": " + String(porcentajeBateria) + "}";

    // Imprimir en el puerto serie y enviar a ThingsBoard
    Serial.print("BMON: "); Serial.print(voltageBMON, 2); Serial.print("V ");
    Serial.print("Voltaje Batería: "); Serial.print(voltajeBateria, 2); Serial.print("V ");
    Serial.print("Porcentaje Batería: "); Serial.print(porcentajeBateria); Serial.println("%");
    
    if (client.publish("v1/devices/me/telemetry", (char*)payload.c_str())) {
      Serial.println("Datos enviados a ThingsBoard: " + payload);
    } else {
      Serial.println("Error al enviar datos a ThingsBoard.");
    }
  }
}

// Función para mapear un float a otro rango
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}





//Funcion para medir y clasificar la actividad
void medirYClasificarActividad() {
    // Asegurarse de mantener la conexión MQTT
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Obtener datos de aceleración del BMA400
    accelerometer.getSensorData();
    float aceleracionX = accelerometer.data.accelX;
    float aceleracionY = accelerometer.data.accelY;
    float aceleracionZ = accelerometer.data.accelZ;

    // Calcular los METs basados en la aceleración (aquí necesitarías tu propia lógica)
    float VM = sqrt(sq(aceleracionX) + sq(aceleracionY) + sq(aceleracionZ));
    float ENMO = VM - 1.0; // Ejemplo simplificado, ajusta según tu necesidad
    float METs = (0.032 * ENMO + 7.28) / 3.5; // Esta es una fórmula ficticia, sustitúyela por la tuya

    // Acumular METs en el periodo actual
    METsPorPeriodo[periodoActual] += METs;

    // Incrementar contador de minutos cada minuto
    contadorMinutos++;
    if (contadorMinutos % 10 == 0) { // Cada 10 minutos
        // Calcular promedio de METs para el periodo actual y enviar
        float promedioMETs = METsPorPeriodo[periodoActual] / 10;
        enviarDatos(promedioMETs, periodoActual, false); // false porque no es el final del ciclo de 30 minutos

        // Preparar para el próximo periodo
        periodoActual = periodoActual++;
        if (periodoActual == 3) { // Después de completar los tres periodos de 10 minutos
          periodoActual = 0;
          for (int i = 0; i < 3; i++) {
            if (METsPorPeriodo[i] > 1.5) {
              actividadSedentaria = true;//cambiar a false para pruebas reales
              break;
            }
          }
          enviarDatos(0, -1, true); // Enviar la bandera de actividad sedentaria
          // Reiniciar los METs acumulados para el próximo ciclo de 30 minutos
          memset(METsPorPeriodo, 0, sizeof(METsPorPeriodo));
        }
        // Reiniciar el contador de METs para el nuevo periodo
        contadorMinutos = 0;
    }
}


void mostrarFechaHora() {
  DateTime now = rtc.now();

  // Limpia la pantalla para el nuevo contenido
  tft.fillScreen(GC9A01A_BLACK);

  // Ajusta el tamaño del texto para la hora y estado WiFi
  tft.setTextSize(4);
  tft.setTextColor(GC9A01A_WHITE);
  tft.setCursor(20, 100);

  // Estado WiFi
  tft.println(WiFi.status() == WL_CONNECTED ? "Conectado" : "Desconectado");
  tft.print(" "); // Espacio extra, si es necesario, puede ser eliminado

  // Muestra la hora actual
  if (now.hour() < 10) tft.print("0");
  tft.print(now.hour(), DEC);
  tft.print(":");
  if (now.minute() < 10) tft.print("0");
  tft.print(now.minute(), DEC);
  tft.println();

  // Ajusta el tamaño del texto para el porcentaje de batería
  tft.setTextSize(2); // Hace el texto más pequeño

  // Calcula el ancho del texto del porcentaje de batería para centrarlo
  String textoBateria = String(int(porcentajeBateria)) + "%";
  int16_t x1, y1;
  uint16_t ancho, alto;
  tft.getTextBounds(textoBateria, 0, 0, &x1, &y1, &ancho, &alto);

  // Centra el texto del porcentaje de batería en la pantalla
  int centroPantalla = tft.width() / 2;
  int posicionX = centroPantalla - (ancho / 2);

  // Muestra el porcentaje de batería centrado en la parte superior
  tft.setCursor(posicionX, 20); // Ajusta el valor en Y si es necesario
  tft.print(textoBateria);
}


void encenderPantalla() {
  digitalWrite(PIN_LED, HIGH); // Enciende la retroiluminación
  pantallaEncendida = true;
  tiempoPantallaEncendida = millis(); // Guarda el momento en que se encendió
}


void apagarPantalla() {
  digitalWrite(PIN_LED, LOW); // Apaga la retroiluminación
  pantallaEncendida = false;
}


void manejarPantalla() {
  bool estadoBoton = digitalRead(BTN3); // Lee el estado actual del botón

  // Si se presiona el botón y la pantalla está apagada, enciende la pantalla
  if (estadoBoton == HIGH && !pantallaEncendida) {
    encenderPantalla();
  }
  // Si la pantalla está encendida y ha pasado el tiempo de encendido, apaga la pantalla
  else if (pantallaEncendida && millis() - tiempoPantallaEncendida > tiempoEncendido) {
    apagarPantalla();
  }
}


void manejarAlertaBoton2() {
  static unsigned long buttonPressedTime = 0;
  static bool buttonHeld = false;
  
  // Verificar si el botón está presionado
  if (digitalRead(BTN2) == HIGH) {
    if (!buttonHeld) {
      // Inicia el seguimiento del tiempo si no se ha presionado antes
      buttonPressedTime = millis();
      buttonHeld = true;
    }
    
    // Comprueba si el botón se ha mantenido presionado el tiempo suficiente para enviar la alerta
    if (millis() - buttonPressedTime > 6000) { //6000, dejarlo presionado 1 segundo
      enviarAlertaUsuario();
      buttonHeld = false; // Resetea el estado para evitar múltiples alertas
    }
  } else {
    buttonHeld = false; // Resetea si el botón se ha soltado
  }
}


void enviarAlertaUsuario() {
  String payload = "{\"ALERTA USUARIO\": 1}";
  if (client.publish(topic, (char*)payload.c_str())) {
    Serial.println("Alerta de usuario enviada a ThingsBoard");
    digitalWrite(PIN_LED, HIGH);

    // Cambiar colores de la pantalla
    tft.fillScreen(GC9A01A_RED); // Cambiar el fondo a rojo
    tft.setTextColor(GC9A01A_BLACK); // Cambiar el color del texto a negro
    tft.setTextSize(4); // Aumentar el tamaño del texto para hacerlo más grande

    // Calcular la posición del texto para "ALERTA" y centrarlo
    String alertMsg = "ALERTA";
    int16_t x1, y1;
    uint16_t width, height;
    tft.getTextBounds(alertMsg, 0, 0, &x1, &y1, &width, &height);
    tft.setCursor((tft.width() - width) / 2, (tft.height() / 2) - height);

    // Mostrar "ALERTA"
    tft.println(alertMsg);

    // Calcular la posición del texto para "ENVIADA" y centrarlo debajo de "ALERTA"
    String sentMsg = "ENVIADA";
    tft.getTextBounds(sentMsg, 0, 0, &x1, &y1, &width, &height);
    tft.setCursor((tft.width() - width) / 2, (tft.height() / 2) + (height / 2)); // Ajustar el interlineado aquí

    // Mostrar "ENVIADA"
    tft.println(sentMsg);

    // Esperar 5 segundos con el mensaje en pantalla
    unsigned long alertTime = millis();
    while(millis() - alertTime < 5000) {
      // Mantiene la actualización de MQTT mientras muestra el mensaje
      if (!client.connected()) {
        reconnect();
      }
      client.loop();
    }

    // Restaurar la visualización previa después de 5 segundos
    tft.fillScreen(GC9A01A_BLACK); // Cambiar el fondo a negro
    digitalWrite(PIN_LED, LOW);
  } else {
    Serial.println("Error al enviar la alerta de usuario");
  }
}




void enviarAceleracionesAThingsBoard() {
    // Asegurarse de mantener la conexión MQTT activa
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Leer datos del acelerómetro
    accelerometer.getSensorData();
    float aceleracionX = accelerometer.data.accelX;
    float aceleracionY = accelerometer.data.accelY;
    float aceleracionZ = accelerometer.data.accelZ;

    // Mostrar datos en el Serial
    Serial.print("Aceleracion X: ");
    Serial.print(aceleracionX);
    Serial.print(" Y: ");
    Serial.print(aceleracionY);
    Serial.print(" Z: ");
    Serial.println(aceleracionZ);

    // Crear el payload JSON
    String payload = "{\"aceleracion_x\": ";
    payload += aceleracionX;
    payload += ", \"aceleracion_y\": ";
    payload += aceleracionY;
    payload += ", \"aceleracion_z\": ";
    payload += aceleracionZ;
    payload += "}";

    // Enviar datos a ThingsBoard
    if (client.publish(topic, (char*)payload.c_str())) {
        Serial.println("Datos de aceleracion enviados a ThingsBoard.");
    } else {
        Serial.println("Error al enviar datos de aceleracion a ThingsBoard.");
    }
}




void loop() {
  // Mantener la conexión MQTT
  if (!client.connected()) {
      reconnect();
  }
  client.loop();

  enviarAceleracionesAThingsBoard();
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastActivityTime >= interval) {
    // Actualiza la última vez que se ejecutó la función
    lastActivityTime = currentMillis;

    // Ejecutar la función de medición y clasificación de actividad
    medirYClasificarActividad();
  }
    

  // Mostrar la hora y el estado de conexión constantemente
  mostrarFechaHora();

  //calcula el nivel de bateria y lo manda a tb
  enviarNivelBateriaAThingsBoard();

  //mira si el usuario necesita mandar una alerta urgente
  manejarAlertaBoton2();

  //mira si el usuario quiere prender la pantalla
  manejarPantalla();

  delay(100);
  
}