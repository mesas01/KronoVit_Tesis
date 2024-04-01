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
const char* ssid = "SM-DESKTOP";
const char* password = "12345678";
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

unsigned long lastSendTime = 0; // Controla el último envío
const long sendInterval = 6000; // Intervalo entre envíos (60 segundos)
int paqueteNumero = 1; // Contador de paquetes


Adafruit_GC9A01A tft = Adafruit_GC9A01A(TFT_CS, TFT_DC, TFT_RST);

// Nuevas variables para el manejo de promedios para transmitir cada 30 min
int contadorMinutos = 0;
float METsPorPeriodo[3] = {0, 0, 0}; // Almacena los METs de cada periodo de 10 minutos
int periodoActual = 0; // Indica el periodo actual de 10 minutos dentro de los 30 minutos
bool banderaActividad = false; // Indica si en algún periodo se superó el umbral de 1.5 METs


//---------------------------FUNCIONES------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  digitalWrite(PIN_LED, HIGH);

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
void enviarDatos(String payload) {
    if (!client.connected()) {
        reconnect(); // Intenta reconectar si no está conectado
    }
    client.loop(); // Mantén la conexión MQTT viva

    if (client.publish(topic, payload.c_str())) {
        Serial.println("Payload enviado: " + payload);
    } else {
        Serial.println("Error al enviar el payload.");
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




void medirAceleracion(float& x, float& y, float& z) {
    // Asegúrate de que esta línea está en el setup: accelerometer.beginI2C();
    accelerometer.getSensorData(); // Actualiza los datos del sensor
    
    x = accelerometer.data.accelX;
    y = accelerometer.data.accelY;
    z = accelerometer.data.accelZ;

    Serial.print("Aceleracion X: "); Serial.println(x);
    Serial.print("Aceleracion Y: "); Serial.println(y);
    Serial.print("Aceleracion Z: "); Serial.println(z);
}



String obtenerTimestamp(DateTime now) {
    char timestamp[20];
    // Cambia el formato aquí para incluir solo la hora, minutos y segundos
    sprintf(timestamp, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    return String(timestamp);
}



String formatearPayload(String met, String timestamp, int numeroPaquete, float acelX, float acelY, float acelZ, bool sedentario, float bateria) {
    String payload = "{\"met\": \"" + met + "\", \"timestamp\": \"" + timestamp +
                     "\", \"paquete nro\": " + numeroPaquete +
                     ", \"aceleracion x\": " + acelX +
                     ", \"aceleracion y\": " + acelY +
                     ", \"aceleracion z\": " + acelZ +
                     ", \"comportamiento sedentario\": " + (sedentario ? "true" : "false") +
                     ", \"porcentaje bateria\": " + bateria + "%}";
    return payload;
}



// Variables Globales
int lastSecond = -1; // Almacena el último segundo de envío

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
    mostrarFechaHora();

    DateTime now = rtc.now();
    
    // Obtén los segundos actuales
    int currentSecond = now.second();

    // Verifica si ha pasado al menos un segundo desde el último envío
    if (lastSecond != currentSecond) {
        lastSecond = currentSecond; // Actualiza el último segundo de envío

        // Medir aceleración
        float aceleracionX, aceleracionY, aceleracionZ;
        medirAceleracion(aceleracionX, aceleracionY, aceleracionZ);

        // Simula obtener un porcentaje de batería
        float porcentajeBateria = 95.0;

        // Formatear y enviar el payload
        String timestamp = obtenerTimestamp(now);
        String payload = formatearPayload("4", timestamp, paqueteNumero++, aceleracionX, aceleracionY, aceleracionZ, false, porcentajeBateria);
        enviarDatos(payload);
    }
    delay(10); // Un pequeño delay para evitar saturar el CPU
}

