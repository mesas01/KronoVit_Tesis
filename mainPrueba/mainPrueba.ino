#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "SparkFun_BMA400_Arduino_Library.h"
#include <RTClib.h>
#include <Adafruit_GC9A01A.h>
#include <Adafruit_GFX.h>
#include <time.h>

//---------------------------------WIFI-----------------------------------------------------------------------------------
const char* ssid = "SM-DESKTOP";//"SMN"; //"SLB";
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
//---------------------------------RTC & DISPLAY-------------------------------------------------------------------------
RTC_DS3231 rtc;
#define TFT_CS     5
#define TFT_RST    33
#define TFT_DC     12
//---------------------------------suspension de pantalla-------------------------------------------------------------------------
#define PIN_LED 9 // Asumiendo que el PIN_LED es el IO2
#define BTN3 13 // El pin donde se conecta el botón BTN2
bool pantallaEncendida = false;
unsigned long tiempoPantallaEncendida = 0;
const unsigned long tiempoEncendido = 15000; // 15 segundos en milisegundos
bool necesitaApagarPantalla = false;


Adafruit_GC9A01A tft = Adafruit_GC9A01A(TFT_CS, TFT_DC, TFT_RST);

// Nuevas variables para el manejo de promedios para transmitir cada 30 min
float sumMETs = 0;
int contadorSegundos = 0, contadorMinutos = 0, contador10Minutos = 0;
float promedioMETs30Min[3] = {0, 0, 0};
int indice30Min = 0;
float METsPorPeriodo[3] = {0, 0, 0}; // Almacena los METs de cada periodo de 10 minutos
int periodoActual = 0; // Indica el periodo actual de 10 minutos dentro de los 30 minutos
bool banderaActividad = false; // Indica si en algún periodo se superó el umbral de 1.5 METs



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
//Funcion para enviar los datos
void enviarDatos(float MET, int periodo, bool finalizarCiclo) {
    DateTime now = rtc.now(); // Obtener la hora actual del RTC
    String timestamp = String(now.year(), DEC) + "-" + 
                       String(now.month(), DEC) + "-" + 
                       String(now.day(), DEC) + " T" + 
                       String(now.hour(), DEC) + ":" + 
                       String(now.minute(), DEC) + ":" + 
                       String(now.second(), DEC);

    String payload;
    if (!finalizarCiclo) {
        // Enviar MET de un periodo específico junto con la hora
        payload = "{\"METsPeriodo" + String(periodo + 1) + "\": " + String(MET, 2) + ", \"timestamp\": \"" + timestamp + "\"}";
    } else {
        // Comprobar si todos los periodos están debajo de 1.5
        bool todosDebajo = true;
        for (int i = 0; i < 3; i++) {
            if (METsPorPeriodo[i] > 1.5) {
                todosDebajo = true;//cambiar a false para su funcionamiento correcto;
                break;
            }
        }
        
        if (todosDebajo) {
            payload = "{\"MET\": \"1.4\", \"timestamp\": \"" + timestamp + "\"}";
        } else {
            // Si no, enviar la bandera de actividad normalmente
            payload = "{\"BanderaActividad\": " + String(todosDebajo ? "true" : "false") + ", \"timestamp\": \"" + timestamp + "\"}";
        }
        
        // Resetear para el próximo ciclo de 30 minutos
        for (int i = 0; i < 3; i++) METsPorPeriodo[i] = 0;
        periodoActual = 0;
    }

    if (client.publish(topic, (char*)payload.c_str())) {
        Serial.println("Datos enviados exitosamente: " + payload);
    } else {
        Serial.println("Error al enviar datos");
    }
}




//Funcion para medir el nivel de la bateria 
void medirNivelBateria() {
  // Definir los voltajes máximo y mínimo de la batería
  const float voltajeMaximo = 4.15; // Ajustar al voltaje máximo de tu batería
  const float voltajeMinimo = 3.3; // Ajustar al voltaje mínimo de tu batería
  
  uint32_t Vbatt = 0;
  for(int i = 0; i < 16; i++) {
    Vbatt += analogRead(25); // Asumiendo A18 es tu pin de lectura de batería
  }
  float Vbattf = (Vbatt / 16.0) * (3.3 / 4095.0) * 2; // Ajusta esta fórmula según tu circuito

  // Calcular el porcentaje de batería basado en los voltajes máximo y mínimo
  float porcentajeBateria = (Vbattf - voltajeMinimo) / (voltajeMaximo - voltajeMinimo) * 100;
  porcentajeBateria = constrain(porcentajeBateria, 0, 100); // Asegura que el porcentaje esté entre 0 y 100

  // Actualiza la pantalla TFT con el porcentaje de batería
  tft.setCursor(70, 50); // Ajusta la posición según sea necesario
  tft.setTextSize(2);
  tft.setTextColor(GC9A01A_WHITE);
  tft.print("Bateria: ");
  tft.print(porcentajeBateria, 0);
  tft.println("%");
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
        periodoActual = (periodoActual + 1) % 3; // Rotar entre 0, 1, y 2 para los periodos de 30 minutos
        if (periodoActual == 0) { // Después de completar los tres periodos de 10 minutos
            // Evaluar si se activa la bandera de actividad sedentaria
            bool actividadSedentaria = false;
            for (int i = 0; i < 3; i++) {
                if (METsPorPeriodo[i] > 1.5) {
                    actividadSedentaria = false;//cambiar a true para pruebas reales
                    break;
                }
            }
            enviarDatos(0, -1, actividadSedentaria); // Enviar la bandera de actividad sedentaria
            // Reiniciar los METs acumulados para el próximo ciclo de 30 minutos
            memset(METsPorPeriodo, 0, sizeof(METsPorPeriodo));
        }
        // Reiniciar el contador de METs para el nuevo periodo
        contadorMinutos = 0;
    }
}



void setup() {
  Serial.begin(115200);
  Wire.begin();

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


//Función para medir el nivel de la batería y enviarlo a ThingsBoard cada minuto
void enviarNivelBateriaAThingsBoard() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastBatteryReadMillis >= batteryReadInterval) {
    lastBatteryReadMillis = currentMillis; // Actualiza la última vez que se leyó la batería

    uint32_t Vbatt = 0;
    for (int i = 0; i < 16; i++) {
      Vbatt += analogReadMilliVolts(25);
    }
    float Vbattf = Vbatt * 21.0 / 16 / 1000.0; // Conversión a voltios

    // Asumiendo un rango operativo de 3.0V (min) a 4.2V (max)
    float voltajeMinimo = 3.0;
    float voltajeMaximo = 4.2;
    porcentajeBateria = (Vbattf - voltajeMinimo) / (voltajeMaximo - voltajeMinimo) * 100;
    porcentajeBateria = constrain(porcentajeBateria, 1, 100); // Asegurar que esté entre 1 y 100

    String payload = "{\"voltajeBateria\": " + String(Vbattf, 3) + "}";
    
    if (client.publish("v1/devices/me/telemetry", (char*)payload.c_str())) {
      Serial.println("Nivel de batería enviado a ThingsBoard: " + String(Vbattf, 3) + "V");
    } else {
      Serial.println("Error al enviar el nivel de batería a ThingsBoard.");
    }
  }
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





void loop() {
  // Mantener la conexión MQTT
  if (!client.connected()) {
      reconnect();
  }
  client.loop();
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastActivityTime >= interval) {
    // Actualiza la última vez que se ejecutó la función
    lastActivityTime = currentMillis;

    // Ejecutar la función de medición y clasificación de actividad
    medirYClasificarActividad();
  }
    

  // Mostrar la hora y el estado de conexión constantemente
  mostrarFechaHora();

  enviarNivelBateriaAThingsBoard();

  manejarPantalla();

  delay(1000);

  
}


