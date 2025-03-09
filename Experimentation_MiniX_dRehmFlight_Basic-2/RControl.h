//###############################################################
// D'après Test_Recepteur_29-12-2024.ino pour le drone ESP32-S2
//###############################################################

#include <WiFi.h>
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#include <vector>

#define ESPNOW_WIFI_CHANNEL 6

// Remote signal timeout: 1 seconde
#define SIGNAL_TIMEOUT 1000
static unsigned long lastRecvTime = 0;
bool radio_ON = false;

// Structure de reception données(doit correspondre a celle d'envoi)
typedef struct struct_message {
  uint16_t Ch1   ;//  : 11;// Trotlle
  uint16_t Ch2   ;//  : 11;// Yaw
  uint16_t Ch3   ;//  : 11;// Pitch
  uint16_t Ch4   ;//  : 11;// Roll
  uint16_t Ch5   ;//  : 11;// Bouton1
  uint16_t Ch6   ;//  : 11;// Bouton2
  uint16_t Ch7   ;//  : 11;// Aux1
  uint16_t Ch8   ;//  : 11;// Aux2
  uint8_t spare  ;//  :  8;
} struct_message;

// creation de la structure du message => nom = myData
struct_message myData;

//#####################################################
// Creating a new class that inherits from the ESP_NOW_Peer class is required.
class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
  public:
    // Constructor of the class
    ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel,
                       wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr,
                             channel, iface, lmk) {}

    // Destructor of the class
    ~ESP_NOW_Peer_Class() {}

    //------------------------------------------------------
    // Function to register the master peer
    bool add_peer() {
      if (!add()) {
        log_e("Failed to register the broadcast peer");
        return false;
      }
      return true;
    }

    //------------------------------------------------------
    // Function to print the received messages from the master
    void onReceive(const uint8_t *data, size_t len, bool broadcast) {
      memcpy(&myData, data, sizeof(myData));
      channel_1_pwm = myData.Ch1;// Trotlle
      channel_4_pwm = myData.Ch2;// Yaw
      channel_3_pwm = myData.Ch3;// Pitch
      channel_2_pwm = myData.Ch4;// Roll
      channel_5_pwm = myData.Ch7;// Aux1
      channel_6_pwm = myData.Ch8;// Aux2
//      channel_7_pwm = myData.Ch5;// Bouton 1
//      channel_8_pwm = myData.Ch6;// Bouton 2
      lastRecvTime  = millis();
    }
};

//=====================================================
/* Global Variables */
// List of all the masters. It will be populated when a new master is registered
std::vector<ESP_NOW_Peer_Class> masters;

//=====================================================
// Callback called when an unknown peer sends a message
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
#ifdef DEBUG
    Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
    Serial.println("Registering the peer as a master");
#endif
    ESP_NOW_Peer_Class new_master(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

    masters.push_back(new_master);
    if (!masters.back().add_peer()) {
#ifdef DEBUG
      Serial.println("Failed to register the new master");
#endif
      return;
    }
  } else {
    // The slave will only receive broadcast messages
    log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
    log_v("Ignoring the message");
  }
}

//=====================================================
// Vérifie s'il y a une connection avec l'émetteur
void checkRemote() {
  unsigned long Now = millis();
  if ( Now - lastRecvTime < SIGNAL_TIMEOUT ) radio_ON = true;
  else radio_ON = false;
}

//#####################################################
void RCSetup() {
  // Initialise le Wi-Fi esp_now
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

#ifdef DEBUG
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);
#endif

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
#ifdef DEBUG
    Serial.println("Failed to initialize ESP-NOW");
    Serial.println("Reeboting in 5 seconds...");
#endif
    delay(5000);
    ESP.restart();
  }

  // Register the new peer callback
  ESP_NOW.onNewPeer(register_new_master, NULL);
}

//#####################################################
