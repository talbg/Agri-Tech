
#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH THE MAC Address of your receiver
uint8_t Address1[] = {0x84, 0xCC, 0xA8, 0x7B, 0xBA, 0x04};
uint8_t Address2[] = {0x84, 0xCC, 0xA8, 0x7A, 0xC6, 0x10};
uint8_t Address3[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Address4[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Address5[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Address6[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Address7[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Address8[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Address9[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Address10[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct struct_message {
  int id;
  byte value1;
  int value2;
  bool value3;
} struct_message;

// Create a struct_message to hold outgoing sensor readings
struct_message sendData;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
struct_message board3;
struct_message board4;
struct_message board5;
struct_message board6;
struct_message board7;
struct_message board8;
struct_message board9;
struct_message board10;

// Create an array with all the structures
struct_message boardsStruct[] = {board1, board2, board3, board4, board5, board6, board7, board8, board9, board10};

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);

  // Update the structures with the new incoming data
  boardsStruct[myData.id - 1].value1 = myData.value1;
  boardsStruct[myData.id - 1].value2 = myData.value2;
  boardsStruct[myData.id - 1].value3 = myData.value3;
  Serial.printf("value1: %d \n", boardsStruct[myData.id - 1].value1);
  Serial.printf("value2: %d \n", boardsStruct[myData.id - 1].value2);
  Serial.printf("value3: %d \n", boardsStruct[myData.id - 1].value3);
  Serial.println();
}

void setup()
{
  // Init Serial Monitor
  Serial.begin(115200);

  InitEspNow();

}

void loop()
{
  sendDataToAll();
  delay(2000);

  // Acess the variables for board 1
  //*
    byte value1_board1 = boardsStruct[0].value1;
    int value2_board1 = boardsStruct[0].value2;
    bool value3_board1 = boardsStruct[0].value3;
  //*/
}

void sendDataToAll()
{
  //!!!ID for each Board!!!
  sendData.id = 1;


  //* How to set Data
  sendData.value1 = 88;
  sendData.value2 = 556;
  sendData.value3 = false;
  //*/

  esp_err_t result = esp_now_send(0, (uint8_t *) &sendData, sizeof(struct_message));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void InitEspNow()
{
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // register peer
  esp_now_peer_info_t peerInfo;
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // register first peer
  memcpy(peerInfo.peer_addr, Address1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // register second peer
  memcpy(peerInfo.peer_addr, Address2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register third peer
  memcpy(peerInfo.peer_addr, Address3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register fourth peer
  memcpy(peerInfo.peer_addr, Address4, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register fifth peer
  memcpy(peerInfo.peer_addr, Address5, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register sixth peer
  memcpy(peerInfo.peer_addr, Address6, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register next peer
  memcpy(peerInfo.peer_addr, Address7, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register next peer
  memcpy(peerInfo.peer_addr, Address8, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register next peer
  memcpy(peerInfo.peer_addr, Address9, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  /// register next peer
  memcpy(peerInfo.peer_addr, Address10, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}
