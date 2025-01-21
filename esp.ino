#include <WiFi.h>
#include <PubSubClient.h>

// Configuración de WiFi y MQTT
WiFiClient esp32Client;
PubSubClient mqttClient(esp32Client);

const char* ssid     = "Red abierta UCUENCA";    // Tu SSID WiFi
const char* password = "";           // Tu contraseña WiFi

char* server = "broker.emqx.io";               // Dirección del servidor MQTT
int port = 1883;                               // Puerto MQTT

char* topicSerialData = "VELOCIDAD_RPM";       // Tema para publicar datos desde el Arduino

// Configuración de pines para Serial2
#define RX2 16  // Pin RX del ESP32 conectado al TX del Arduino
#define TX2 17  // Pin TX del ESP32 conectado al RX del Arduino

// Función para inicializar WiFi
void wifiInit() {
    Serial.print("Conectándose a ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\nConectado a WiFi");
    Serial.print("Dirección IP: ");
    Serial.println(WiFi.localIP());
}

// Función para manejar la comunicación con MQTT
void handleMQTT(String data) {
    if (mqttClient.connected()) {
        mqttClient.publish(topicSerialData, data.c_str());
        Serial.println("Publicado en MQTT: " + data);
    } else {
        Serial.println("MQTT no está conectado");
    }
}

// Función para reconectar al servidor MQTT
void reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Intentando conectarse a MQTT...");
        if (mqttClient.connect("esp32Client")) {
            Serial.println("Conectado a MQTT");
        } else {
            Serial.print("Fallo, rc=");
            Serial.print(mqttClient.state());
            Serial.println(". Intentando de nuevo en 5 segundos");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(9600);       // Inicializa Serial para el monitor de la PC
    Serial2.begin(9600, SERIAL_8N1, 16, 17);  // Inicializa Serial2 para el Arduino
    delay(10);

    Serial.println("Iniciando ESP32...");
    wifiInit();
    mqttClient.setServer(server, port);
}

void loop() {
    // Asegura que la conexión a MQTT esté activa
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();

    // Leer datos desde el Arduino (Serial2)
    if (Serial2.available()) {
        String msg = Serial2.readStringUntil('\n');
        msg.trim(); // Limpia espacios en blanco
        Serial.println("Datos recibidos del Arduino: " + msg);

        // Publicar datos en MQTT
        handleMQTT(msg);

        // Respuesta opcional al Arduino
        Serial2.println("Mensaje recibido por ESP32");
    }

    delay(100);  // Breve pausa para evitar sobrecarga
}
