#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Informations pour le Wi-Fi et le serveur MQTT
const char* ssid = "iot";
const char* password = "iotisis;";
const char* mqtt_server = "192.168.3.250";  // Remplacer par l'adresse de ton serveur MQTT

WiFiClient espClient;
PubSubClient client(espClient);

// Structure pour recevoir les données
typedef struct struct_message {
  int id;        // Identifiant unique (1 ou 2)
  float x;       // Température
  float y;       // Humidité
} struct_message;

// Créer une instance pour stocker les données reçues
struct_message sensorData[2];

// Variables pour gérer le délai entre les envois
unsigned long previousMillis = 0; // Pour stocker l'heure du dernier envoi
const unsigned long interval = 10000; // Intervalle en millisecondes (10 secondes)

// Adresse MAC des capteurs
uint8_t moteAddress[2][6] = {
  {0xEC, 0x62, 0x60, 0x11, 0x97, 0xA0},  // Identifiant pour le capteur 1
  {0x24, 0xDC, 0xC3, 0x14, 0x3D, 0x70}   // Identifiant pour le capteur 2
};

// Fonction callback exécutée lors de la réception des données via ESP-NOW
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  struct_message receivedData;
  memcpy(&receivedData, incomingData, sizeof(receivedData)); // Copier les données reçues

  // Trouver l'indice du capteur correspondant et stocker les données
  if (receivedData.id == 1) {
    sensorData[0] = receivedData;
  } else if (receivedData.id == 2) {
    sensorData[1] = receivedData;
  }

  Serial.printf("ID : %d, Température : %.2f °C, Humidité : %.2f %%\n", receivedData.id, receivedData.x, receivedData.y);
}

// Configuration initiale du Wi-Fi
void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connecté.");
}

// Reconnexion au serveur MQTT si nécessaire
void reconnect() {
  while (!client.connected()) {
    Serial.print("Connexion au serveur MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("connecté.");
    } else {
      Serial.print("échec, rc=");
      Serial.print(client.state());
      Serial.println(" nouvelle tentative dans 5 secondes.");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Configuration du Wi-Fi
  WiFi.mode(WIFI_STA);
  setup_wifi();

  // Initialisation d'ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erreur lors de l'initialisation d'ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv); // Callback pour réception des données

  // Ajouter les capteurs comme peer dans ESP-NOW
  for (int i = 0; i < 2; i++) {
    esp_now_peer_info_t peerInfo;
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    memcpy(peerInfo.peer_addr, moteAddress[i], 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Erreur lors de l'ajout du peer");
      return;
    }
  }

  // Configuration MQTT
  client.setServer(mqtt_server, 1883);
}

void loop() {
  // Assurer la connexion au serveur MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Gestion du délai d'envoi
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Publier les données des capteurs
    for (int i = 0; i < 2; i++) {
      if (sensorData[i].id != 0) {  // Si des données ont été reçues
        char tempString[8], humString[8];
        dtostrf(sensorData[i].x, 1, 2, tempString);
        dtostrf(sensorData[i].y, 1, 2, humString);

        // Publier les données via MQTT
        String topicTemp = "esp32/board" + String(i) + "/temperature";
        String topicHum = "esp32/board" + String(i) + "/humidity";

        Serial.println("Envoi des données via MQTT...");
        client.publish(topicTemp.c_str(), tempString);
        client.publish(topicHum.c_str(), humString);

        // Réinitialiser les données reçues après publication
        sensorData[i] = {};
      }
    }
  }
}
