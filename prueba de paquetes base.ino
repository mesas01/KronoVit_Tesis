#include <time.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <map>

// Configuración WiFi
const char* ssid = "SM-DESKTOP";
const char* password = "12345678";

// Configuración del servidor ThingsBoard
const char* mqtt_server = "demo.thingsboard.io";
const char* token = "qkry73ibqja3gqez8vc7"; // Sustituye esto por tu token real
const char* topic = "v1/devices/me/telemetry";

unsigned long lastSendTime = 0; // Almacena la última vez que se envió un mensaje
const long sendInterval = 1000; // Intervalo entre mensajes (1000 milisegundos = 1 segundo)
int contadorAsistencia = 1;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -5 * 3600; // UTC-5 para Colombia
const int   daylightOffset_sec = 0; // Ajusta este valor si tu zona tiene horario de verano


WiFiClient espClient;
PubSubClient client(espClient);

// Pines para el RFID y LEDs
#define SS_PIN 21
#define RST_PIN 4
const int ledPin1 = 12;
const int ledPin2 = 13;
const int ledPin3 = 14;

MFRC522 rfid(SS_PIN, RST_PIN); // Instancia MFRC522

// Mapa para llevar la cuenta de las asistencias
std::map<String, int> userAttendance;

void setup_wifi() {
  Serial.begin(115200);
  Serial.println("Conectando a WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi conectado");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando a MQTT...");
    if (client.connect("ESP32Client", token, NULL)) {
      Serial.println("conectado");
    } else {
      Serial.print("fallo, rc=");
      Serial.print(client.state());
      Serial.println(" intento en 5 segundos");
      delay(5000);
    }
  }
}

void setup() {
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  SPI.begin();
  rfid.PCD_Init();
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

String getUIDString() {
  String uidString = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    uidString += (rfid.uid.uidByte[i] < 0x10 ? "0" : "") + String(rfid.uid.uidByte[i], HEX);
  }
  uidString.toUpperCase();
  return uidString;
}

String getTagName(String uid) {
    if (uid.equals("FA108A7D")) {
        return "santiago adulto mayor 2";
    } else if (uid.equals("325A991E")) {
        return "hugo adulto mayor 1";
    } else if (uid.equals("42C4B889")) {
        return "Adulto mayor 3";
    }
    return "Desconocido";
}


void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}



void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= sendInterval) {
    lastSendTime = currentMillis; // Actualiza el último tiempo de envío

    // Prender LEDs
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, LOW);
    digitalWrite(ledPin3, HIGH);

    // Obtén la hora actual del ESP32
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    char timestamp[9];
    strftime(timestamp, sizeof(timestamp), "%H:%M:%S", &timeinfo);
    
    // Prepara el payload con el contador de asistencia actual
    String payload = "{\"timestamp\":\"" + String(timestamp) + 
                     "\", \"RFID_tag\":\"nombre AM\", \"contador de asistencia\":" + 
                     String(contadorAsistencia) + "}";
    Serial.println("Enviando datos a ThingsBoard...");
    if (client.publish(topic, (char*) payload.c_str())) {
      Serial.println("Datos enviados con éxito: " + payload);
      contadorAsistencia++; // Incrementa el contador de asistencia después de un envío exitoso
    } else {
      Serial.println("Error al enviar datos.");
    }

    // Espera un momento antes de apagar los LEDs para que sean visibles
    delay(100); // Ajusta este tiempo según necesites

    // Apagar LEDs
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin3, LOW);
  }
}

