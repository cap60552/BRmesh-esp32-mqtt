#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"
#include "BLEBeacon.h"
#include <WiFi.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <ArduinoHA.h>
#include <String>
#include <vector>
#include <cstdio>

//////////////////////////////////////////////////////
// CONFIGURATION
// WiFi and MQTT settings are read at boot from /config.json on LittleFS.
// Edit data/config.json, then run PlatformIO "Upload Filesystem Image".
//////////////////////////////////////////////////////
#define CONFIG_PATH "/config.json"

struct Config {
  std::string wifiSsid;
  std::string wifiPassword;
  std::string mqttBroker;      // IP address or hostname
  uint16_t mqttPort = 1883;
  std::string mqttUsername;    // empty = no auth
  std::string mqttPassword;
};
// Global so the strings live forever: ArduinoHA keeps pointers to them.
Config config;

uint8_t my_key[4];
byte mac[6];
WiFiClient client;
HADevice device;
HAMqtt* mqtt;
BLEAdvertising* pAdvertising;
const uint8_t default_key[] = { 0x5e, 0x36, 0x7b, 0xc4 };
struct LightType {
  std::string name;
  uint8_t code[2];
};
std::vector<LightType> lightTypes = {
  {"Smart", {0x39, 0xae}}, // 44601 - AE39 - smart light
  {"RGBW" , {0xa1, 0xa8}}, // 43169 - A8A1 - RGBW light
  {"RGB"  , {0xa0, 0xa8}}, // 43168 - A8A0 - RGB light
};
#define BLESCAN_DURATION 5     // seconds to look for unpaired lights at boot
#define PAIR_ATTEMPTS 5        // how many times to retry sending the key to a light
#define PAIR_SCAN_SECONDS 2    // how long to listen for the light's reply each attempt
#define COMMAND_ADVERTISE_MS 500 // how long each on/off/colour command is broadcast
#define MAX_LIGHTS 31          // HAMqtt has 32 entity slots; one is the rescan button
struct LightDevice {
  BLEAdvertisedDevice device;
  bool hasBleAddr = false;     // seen over BLE during this boot's scan
  bool fromStorage = false;    // loaded from flash (paired on an earlier boot)
  uint8_t lightMac[6] = {0};   // light's own MAC from its advertisement (stable, unique)
  uint8_t type[2] = {0};
  bool isRegistered = false;
  std::string id;
  HALight* light = nullptr;
  std::string name;
  uint8_t number = 0;
  // the bridge's own record of what it last told the light
  uint8_t brightness = 127;    // 1..127
  bool isOn = false;
  bool hasColor = false;       // false = plain white mode (0x22 commands)
  uint8_t colorData[5] = {0};  // bytes 3..7 of the 0x72 command: B, R, G, temp lo, temp hi
};
struct StoredLight {
  uint8_t mac[6];
  uint8_t type[2];
  uint8_t number;
};
Preferences prefs;
std::vector<LightDevice> myLights;
#define BUFFER_SIZE 200
char str_buffer[BUFFER_SIZE];
BLEScan* pBLEScan;
const int ledPin = 2;
const int resetPin = 0; // BOOT button on most ESP32 dev boards
HAButton* rescanButton = nullptr;
// set by the HA button callback, handled in loop() so the long BLE scan
// doesn't run inside the MQTT message handler
volatile bool rescanRequested = false;

bool doesStringMatchBytes(std::string str, const u_int8_t* bytes) {
  bool result = true;
  for (int i = 0; i < str.length(); i++) {
    if ((uint8_t)str[i] != bytes[i]) {
      result = false;
      break;
    }
  }
  return result;
}

uint8_t SEND_SEQ = 0;
uint8_t SEND_COUNT = 1;
const uint8_t DEFAULT_BLE_FASTCON_ADDRESS[] = { 0xC1, 0xC2, 0xC3 };
const uint8_t addrLength = 3;

void dump(const uint8_t* data, int length)
{
  for (int i = 0; i < length; i++)
  {
    Serial.printf("%2.2X", data[i]);
  }
}

void dump(std::string str)
{
  for (int i = 0; i < str.size(); i++)
  {
    Serial.printf("%2.2X", str[i]);
  }
}

uint8_t package_ble_fastcon_body(int i, int i2, uint8_t sequence, uint8_t safe_key, int forward, const uint8_t* data, int length, const uint8_t* key, uint8_t*& payload)
{
  if (length > 12)
  {
    Serial.print("data too long");
    payload = 0;
    return 0;
  }
  uint8_t payloadLength = 4 + 12;
  payload = (uint8_t*)malloc(payloadLength);
  payload[0] = (i2 & 0b1111) << 0 | (i & 0b111) << 4 | (forward & 0xff) << 7;
  payload[1] = sequence & 0xff;
  payload[2] = safe_key;
  payload[3] = 0; // checksum
  // fill payload with zeros
  for (int j = 4; j < payloadLength; j++) payload[j]=0;
  memcpy(payload + 4, data, length);

  uint8_t checksum = 0;
  for (int j = 0; j < length + 4; j++)
  {
    if (j == 3) continue;
    checksum = (checksum + payload[j]) & 0xff;
  }
  payload[3] = checksum;
  for (int j = 0; j < 4; j++) {
    payload[j] = default_key[j & 3] ^ payload[j];
  }
  for (int j = 0; j < 12; j++) {
    payload[4 + j] = key[j & 3] ^ payload[4 + j];
  }
  return payloadLength;
}

uint8_t get_payload_with_inner_retry(int i, const uint8_t* data, int length, int i2, const uint8_t* key, int forward, uint8_t*& payload) {
  SEND_COUNT++;
  SEND_SEQ = SEND_COUNT;
  Serial.print("data: "); dump(data, length); Serial.print("\n");
  Serial.print("key: "); dump(key, 4); Serial.print("\n");
  Serial.printf("sequence: %d\n", SEND_SEQ);
  uint8_t safe_key = 0xff;
  bool hasKey = false;
  for (int i = 0; i < 4; i++) {
    if (key[i] != 0) {
      hasKey = true;
      break;
    }
  }
  if (hasKey) safe_key = key[3];
  uint8_t result = package_ble_fastcon_body(i, i2, SEND_SEQ, safe_key, forward, data, length, key, payload);
  if (!hasKey) {
    // set the data content to the default key
    for (int i = 4; i < 16; i++) {
      payload[i] = default_key[i & 3];
    }
  }
  return result;
}

void whiteningInit(uint8_t val, uint8_t* ctx)
{
  ctx[0] = 1;
  ctx[1] = (val >> 5) & 1;
  ctx[2] = (val >> 4) & 1;
  ctx[3] = (val >> 3) & 1;
  ctx[4] = (val >> 2) & 1;
  ctx[5] = (val >> 1) & 1;
  ctx[6] = val & 1;
}

void whiteningEncode(const uint8_t* data, int len, uint8_t* ctx, uint8_t* result)
{
  memcpy(result, data, len);
  for (int i = 0; i < len; i++) {
    int ctx3 = ctx[3];
    int ctx5 = ctx[5];
    int ctx6 = ctx[6];
    int ctx4 = ctx[4];
    int ctx52 = ctx5 ^ ctx[2];
    int ctx41 = ctx4 ^ ctx[1];
    int ctx63 = ctx6 ^ ctx3;
    int ctx630 = ctx63 ^ ctx[0];

    int c = result[i];
    result[i] = ((c & 0x80) ^ ((ctx52 ^ ctx6) << 7))
      + ((c & 0x40) ^ (ctx630 << 6))
      + ((c & 0x20) ^ (ctx41 << 5))
      + ((c & 0x10) ^ (ctx52 << 4))
      + ((c & 0x08) ^ (ctx63 << 3))
      + ((c & 0x04) ^ (ctx4 << 2))
      + ((c & 0x02) ^ (ctx5 << 1))
      + ((c & 0x01) ^ (ctx6 << 0));

    ctx[2] = ctx41;
    ctx[3] = ctx52;
    ctx[4] = ctx52 ^ ctx3;
    ctx[5] = ctx630 ^ ctx4;
    ctx[6] = ctx41 ^ ctx5;
    ctx[0] = ctx52 ^ ctx6;
    ctx[1] = ctx630;
  }
}

uint8_t reverse_8(uint8_t d)
{
  uint8_t result = 0;
  for (uint8_t k = 0; k < 8; k++) {
    result |= ((d >> k) & 1) << (7 - k);
  }
  return result;
}

uint16_t reverse_16(uint16_t d) {
  uint16_t result = 0;
  for (uint8_t k = 0; k < 16; k++) {
    result |= ((d >> k) & 1) << (15 - k);
  }
  return result;
}

uint16_t crc16(const uint8_t* addr, const uint8_t* data, uint8_t dataLength)
{
  uint16_t crc = 0xffff;
  for (int8_t i = addrLength - 1; i >= 0; i--)
  {
    crc ^= addr[i] << 8;
    for (uint8_t ii = 0; ii < 4; ii++) {
      uint16_t tmp = crc << 1;
      if ((crc & 0x8000) != 0) tmp ^= 0x1021;
      crc = tmp << 1;
      if ((tmp & 0x8000) != 0) crc ^= 0x1021;
    }
  }
  for (uint8_t i = 0; i < dataLength; i++) {
    crc ^= reverse_8(data[i]) << 8;
    for (uint8_t ii = 0; ii < 4; ii++) {
      uint16_t tmp = crc << 1;
      if ((crc & 0x8000) != 0) tmp ^= 0x1021;
      crc = tmp << 1;
      if ((tmp & 0x8000) != 0) crc ^= 0x1021;
    }
  }
  crc = ~reverse_16(crc) & 0xffff;
  return crc;
}

uint8_t get_rf_payload(const uint8_t* addr, const uint8_t* data, uint8_t dataLength, uint8_t*& rfPayload)
{
  uint8_t data_offset = 0x12;
  uint8_t inverse_offset = 0x0f;
  uint8_t result_data_size = data_offset + addrLength + dataLength+2;
  uint8_t* resultbuf = (uint8_t*)malloc(result_data_size);
  memset(resultbuf, 0, result_data_size);

  resultbuf[0x0f] = 0x71;
  resultbuf[0x10] = 0x0f;
  resultbuf[0x11] = 0x55;

  for (uint8_t j = 0; j < addrLength; j++) {
    resultbuf[data_offset + addrLength - j - 1] = addr[j];
  }

  for (int j = 0; j < dataLength; j++) {
    resultbuf[data_offset + addrLength + j] = data[j];
  }

  for (int i = inverse_offset; i < inverse_offset + addrLength + 3; i++) {
    resultbuf[i] = reverse_8(resultbuf[i]);
  }

  int crc = crc16(addr, data, dataLength);
  resultbuf[result_data_size-2] = crc & 0xff;
  resultbuf[result_data_size-1] = (crc >> 8) & 0xff;
  rfPayload = resultbuf;
  return result_data_size;
}

uint8_t do_generate_command(int i, const uint8_t* data, uint8_t length, const uint8_t* key, int forward, int use_default_adapter, int i2, uint8_t*& rfPayload)
{
  if (i2 < 0) i2 = 0;
  uint8_t* payload = 0;
  uint8_t payloadLength = get_payload_with_inner_retry(i, data, length, i2, key, forward, payload);
  uint8_t* rfPayloadTmp = 0;
  Serial.print("payload: "); dump(payload, payloadLength); Serial.print("\n");
  uint8_t rfPayloadLength = get_rf_payload(DEFAULT_BLE_FASTCON_ADDRESS, payload, payloadLength, rfPayloadTmp);
  free(payload);
  uint8_t ctx[7];
  whiteningInit(0x25, &ctx[0]);
  uint8_t* result = (uint8_t*)malloc(rfPayloadLength);
  whiteningEncode(rfPayloadTmp, rfPayloadLength, ctx, result);
  rfPayload = (uint8_t*)malloc(rfPayloadLength-15);
  memcpy(rfPayload, result + 15, rfPayloadLength - 15);
  Serial.print("rf payload: "); dump(rfPayload, rfPayloadLength-15); Serial.print("\n");
  free(result);
  free(rfPayloadTmp);
  return rfPayloadLength-15;
}

std::string getServiceData(uint8_t rfPayloadLength, uint8_t* rfPayload)
{
  uint8_t ble_adv_data[] = { 0x02, 0x01, 0x1A, 0x1B, 0xFF, 0xF0, 0xFF };
  uint8_t* advPacket = (uint8_t*)malloc(rfPayloadLength + sizeof(ble_adv_data));
  memcpy(advPacket, ble_adv_data, sizeof(ble_adv_data));
  memcpy(advPacket + sizeof(ble_adv_data), rfPayload, rfPayloadLength);
  Serial.print("send: "); dump(advPacket, rfPayloadLength + sizeof(ble_adv_data)); Serial.print("\n");
  uint8_t dataLength = rfPayloadLength + sizeof(ble_adv_data);
  std::string serviceData = "";
  serviceData += (char)(dataLength-4);
  for (int i = 4; i < dataLength; i++) serviceData += (char)advPacket[i];
  free(advPacket);
  return serviceData;
}

void single_control(const uint8_t* key, const uint8_t* data)
{
  uint8_t* rfPayload = 0;
  uint8_t rfPayloadLength = do_generate_command(5, data, 8, key, true /* forward */, true /* use_default_adapter */, 0, rfPayload);
  std::string serviceData = getServiceData(rfPayloadLength, rfPayload);
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  oAdvertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED 0x04
  oAdvertisementData.addData(serviceData);
  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->setMinInterval(50);
  pAdvertising->setMaxInterval(50);
  pAdvertising->start();
  delay(COMMAND_ADVERTISE_MS);
  pAdvertising->stop();
  free(rfPayload);
  Serial.println("");
}

//////////////////////////////////////////////////////
// LIGHT STORAGE / LOOKUP
//////////////////////////////////////////////////////

// Find a light by its Home Assistant unique ID. Returns nullptr instead of
// throwing (an uncaught throw reboots the ESP32).
LightDevice* findLight(const char* id)
{
  for (size_t i = 0; i < myLights.size(); i++) {
    if (myLights[i].id == id) return &myLights[i];
  }
  return nullptr;
}

std::string macToId(const uint8_t* m)
{
  char buf[13];
  snprintf(buf, sizeof(buf), "%02x%02x%02x%02x%02x%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
  return std::string(buf);
}

void saveLights()
{
  prefs.begin("brmesh", false);
  uint8_t count = 0;
  for (size_t i = 0; i < myLights.size(); i++) {
    if (!myLights[i].isRegistered) continue;
    StoredLight s;
    memcpy(s.mac, myLights[i].lightMac, 6);
    s.type[0] = myLights[i].type[0];
    s.type[1] = myLights[i].type[1];
    s.number = myLights[i].number;
    char k[8];
    snprintf(k, sizeof(k), "l%u", count);
    prefs.putBytes(k, &s, sizeof(s));
    count++;
  }
  prefs.putUChar("count", count);
  prefs.end();
  Serial.printf("Saved %u light(s) to flash\n", count);
}

void loadLights()
{
  prefs.begin("brmesh", true);
  uint8_t count = prefs.getUChar("count", 0);
  for (uint8_t i = 0; i < count; i++) {
    char k[8];
    snprintf(k, sizeof(k), "l%u", i);
    StoredLight s;
    if (prefs.getBytes(k, &s, sizeof(s)) != sizeof(s)) continue;
    LightDevice light;
    memcpy(light.lightMac, s.mac, 6);
    light.type[0] = s.type[0];
    light.type[1] = s.type[1];
    light.number = s.number;
    light.isRegistered = true;
    light.fromStorage = true;
    myLights.push_back(light);
    Serial.printf("Loaded light %u, MAC %s\n", s.number, macToId(s.mac).c_str());
  }
  prefs.end();
}

void loadOrCreateKey()
{
  prefs.begin("brmesh", false);
  if (prefs.getBytes("key", my_key, 4) != 4) {
    uint32_t new_key = esp_random();
    my_key[0] = new_key & 0xFF;
    my_key[1] = (new_key >> 8) & 0xFF;
    my_key[2] = (new_key >> 16) & 0xFF;
    my_key[3] = (new_key >> 24) & 0xFF;
    prefs.putBytes("key", my_key, 4);
    Serial.print("Created and saved new mesh key: ");
  } else {
    Serial.print("Loaded mesh key from flash: ");
  }
  dump(my_key, 4); Serial.print("\n");
  prefs.end();
}

void clearStorage()
{
  prefs.begin("brmesh", false);
  prefs.clear();
  prefs.end();
  Serial.println("Cleared stored key and lights");
}

//////////////////////////////////////////////////////
// HOME ASSISTANT CALLBACKS
//////////////////////////////////////////////////////

// Send the "colour" command (0x72) using the bridge's own record of this
// light's brightness and last colour / colour temperature.
void sendColor(LightDevice* light)
{
  uint8_t data[] = { 0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  data[1] = light->number;
  // 0x80 = "on" bit, as in the ESPHome fastcon component (scross01/esphome-fastcon)
  data[2] = 0x80 | (light->brightness & 127);
  memcpy(data + 3, light->colorData, 5);
  single_control(my_key, data);
}

// LEDs are linear but eyes aren't, so a mix like (255,180,180) looks almost
// white. Gamma-correcting each channel keeps mixed colours saturated.
// Set COLOR_GAMMA to 1.0 to send HA's values unchanged.
#define COLOR_GAMMA 2.2f
uint8_t gammaCorrect(uint8_t v)
{
  if (v == 0 || v == 255) return v;
  uint8_t out = (uint8_t)(powf(v / 255.0f, COLOR_GAMMA) * 255.0f + 0.5f);
  return out == 0 ? 1 : out; // don't drop a channel HA asked for entirely
}

void onStateCommand(bool state, HALight* sender)
{
  LightDevice* light = findLight(sender->uniqueId());
  if (!light) { Serial.printf("Unknown light %s\n", sender->uniqueId()); return; }
  Serial.printf("Light %s (#%u) state: %d\n", sender->uniqueId(), light->number, state);
  // Home Assistant re-sends "ON" along with every colour/brightness change.
  // Skip it when the light is already on so it can't interfere with the colour.
  if (state && light->isOn) {
    Serial.println("  already on, not resending");
    sender->setState(state);
    return;
  }
  uint8_t data[] = { 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  data[1] = light->number;
  if (state) data[2] = 0x80;
  single_control(my_key, data);
  light->isOn = state;
  sender->setState(state); // report state back to the Home Assistant
}

void onBrightnessCommand(uint8_t brightness, HALight* sender)
{
  LightDevice* light = findLight(sender->uniqueId());
  if (!light) { Serial.printf("Unknown light %s\n", sender->uniqueId()); return; }
  Serial.printf("Light %s (#%u) brightness: %u\n", sender->uniqueId(), light->number, brightness);
  light->brightness = brightness & 127;
  if (light->brightness == 0) light->brightness = 1;
  if (light->hasColor) {
    // keep the current colour, just change its brightness
    sendColor(light);
  } else {
    uint8_t data[] = { 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    data[1] = light->number;
    data[2] = light->brightness;
    single_control(my_key, data);
  }
  light->isOn = true;
  sender->setBrightness(brightness); // report brightness back to the Home Assistant
}

void onColorTemperatureCommand(uint16_t temperature, HALight* sender)
{
  LightDevice* light = findLight(sender->uniqueId());
  if (!light) { Serial.printf("Unknown light %s\n", sender->uniqueId()); return; }
  Serial.printf("Light %s (#%u) color temp: %u\n", sender->uniqueId(), light->number, temperature);
  memset(light->colorData, 0, 5);
  light->colorData[3] = temperature & 127;
  light->colorData[4] = (temperature >> 8) & 127;
  light->hasColor = true;
  sendColor(light);
  light->isOn = true;
  sender->setColorTemperature(temperature); // report color temperature back to the Home Assistant
}

void onRGBColorCommand(HALight::RGBColor color, HALight* sender)
{
  LightDevice* light = findLight(sender->uniqueId());
  if (!light) { Serial.printf("Unknown light %s\n", sender->uniqueId()); return; }
  Serial.printf("Light %s (#%u) RGB: %u,%u,%u at brightness %u\n", sender->uniqueId(), light->number,
    color.red, color.green, color.blue, light->brightness);
  memset(light->colorData, 0, 5);
  light->colorData[0] = gammaCorrect(color.blue);
  light->colorData[1] = gammaCorrect(color.red);
  light->colorData[2] = gammaCorrect(color.green);
  light->hasColor = true;
  sendColor(light);
  light->isOn = true;
  sender->setRGBColor(color); // report color back to the Home Assistant
}

// Create the Home Assistant entity for a light. The ID/name strings are
// strdup'd so the pointers ArduinoHA keeps stay valid forever (ArduinoHA does
// not copy them, and std::string storage inside a std::vector can move).
void createHALight(LightDevice& l)
{
  l.id = "brmesh_" + macToId(l.lightMac);
  l.name = "BRMesh Light " + std::to_string(l.number);
  const char* idPtr = strdup(l.id.c_str());
  const char* namePtr = strdup(l.name.c_str());

  std::string typeName = "";
  for (size_t j = 0; j < lightTypes.size(); j++) {
    if (l.type[0] == lightTypes[j].code[0] && l.type[1] == lightTypes[j].code[1]) {
      typeName = lightTypes[j].name;
      break;
    }
  }
  HALight* light;
  if (typeName == "RGBW") {
    light = new HALight(idPtr, HALight::BrightnessFeature | HALight::ColorTemperatureFeature | HALight::RGBFeature);
    light->onBrightnessCommand(onBrightnessCommand);
    light->onColorTemperatureCommand(onColorTemperatureCommand);
    light->onRGBColorCommand(onRGBColorCommand);
    light->setBrightnessScale(127);
    light->setCurrentBrightness(127);
  } else if (typeName == "RGB") {
    light = new HALight(idPtr, HALight::BrightnessFeature | HALight::RGBFeature);
    light->onBrightnessCommand(onBrightnessCommand);
    light->onRGBColorCommand(onRGBColorCommand);
    light->setBrightnessScale(127);
    light->setCurrentBrightness(127);
  } else {
    // "Smart" - no additional features
    light = new HALight(idPtr);
  }
  light->setName(namePtr);
  light->onStateCommand(onStateCommand);
  l.light = light;
  Serial.printf("Created HA light %s (mesh #%u, %s)\n", idPtr, l.number, typeName.c_str());
}

//////////////////////////////////////////////////////
// DISCOVERY / PAIRING
//////////////////////////////////////////////////////

class AddDeviceCallback: public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice foundDevice)
  {
    std::string mData = foundDevice.getManufacturerData();
    if (mData.size() != 18) return;
    std::string type = mData.substr(12, 2);
    std::string key = mData.substr(14, 4);
    bool knownType = false;
    for (size_t i = 0; i < lightTypes.size(); i++) {
      if (doesStringMatchBytes(type, lightTypes[i].code)) { knownType = true; break; }
    }
    if (!knownType || !doesStringMatchBytes(key, default_key)) return;

    uint8_t lmac[6];
    for (int i = 0; i < 6; i++) lmac[i] = (uint8_t)mData[6 + i];

    Serial.printf("Found unpaired light: BLE %s, MAC %s, RSSI %d\n",
      foundDevice.getAddress().toString().c_str(), macToId(lmac).c_str(), foundDevice.getRSSI());

    for (size_t i = 0; i < myLights.size(); i++) {
      // already seen in this scan, or a stored light that was factory reset
      if (memcmp(myLights[i].lightMac, lmac, 6) == 0) {
        if (myLights[i].fromStorage && !myLights[i].hasBleAddr) {
          Serial.println("  -> this light is stored but was reset, will re-pair it");
          myLights[i].device = foundDevice;
          myLights[i].hasBleAddr = true;
          myLights[i].isRegistered = false; // needs the key again
        }
        return;
      }
    }
    if (myLights.size() >= MAX_LIGHTS) {
      Serial.println("  -> too many lights, ignoring this one");
      return;
    }
    LightDevice light;
    light.device = foundDevice;
    light.hasBleAddr = true;
    memcpy(light.lightMac, lmac, 6);
    light.type[0] = type[0];
    light.type[1] = type[1];
    myLights.push_back(light);
  }
};

// index into myLights of the light currently being paired
int pairingIndex = -1;
uint8_t pairingNumber = 0;

class AddLightCallback: public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice foundDevice)
  {
    if (pairingIndex < 0) return;
    LightDevice& l = myLights[pairingIndex];
    if (l.isRegistered) return;
    if (!foundDevice.getAddress().equals(l.device.getAddress())) return;

    // Copy into a real buffer. The original code took .c_str() of a temporary
    // std::string here, which is a dangling pointer -> random light numbers.
    std::string mData = foundDevice.getManufacturerData();
    if (mData.size() != 18) return;
    std::string key = mData.substr(14, 4);
    if (doesStringMatchBytes(key, default_key)) return; // not taken the new key yet

    uint8_t clean[12];
    for (int j = 0; j < 12; j++) clean[j] = my_key[j & 3] ^ (uint8_t)mData[6 + j];
    Serial.print("  Light accepted new key, decoded data: "); dump(clean, 12); Serial.print("\n");
    if (clean[1] != pairingNumber) {
      Serial.printf("  NOTE: light reported number %u, we requested %u (using requested)\n", clean[1], pairingNumber);
    }
    l.number = pairingNumber;
    l.isRegistered = true;
  }
};

AddDeviceCallback addDeviceCallback;
AddLightCallback addLightCallback;

uint8_t nextFreeNumber()
{
  uint8_t n = 1;
  bool used = true;
  while (used && n < 250) {
    used = false;
    for (size_t i = 0; i < myLights.size(); i++) {
      if ((myLights[i].isRegistered || myLights[i].fromStorage) && myLights[i].number == n) { used = true; n++; break; }
    }
  }
  return n;
}

void scan()
{
  Serial.println("Send wake command, scanning for unpaired lights");
  uint8_t data[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  const uint8_t key[] = { 0x00, 0x00, 0x00, 0x00 };
  uint8_t* rfPayload = 0;
  uint8_t rfPayloadLength = do_generate_command(0, data, 6, key, false, true, 0, rfPayload);
  std::string serviceData = getServiceData(rfPayloadLength, rfPayload);
  free(rfPayload);
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  oAdvertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED 0x04
  oAdvertisementData.addData(serviceData);
  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->setMinInterval(50);
  pAdvertising->setMaxInterval(50);
  pAdvertising->start();
  pBLEScan->setAdvertisedDeviceCallbacks(&addDeviceCallback, true /* want duplicates */);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(BLESCAN_DURATION, false);
  pBLEScan->clearResults();
  pAdvertising->stop();
}

bool pairLight(int index, uint8_t lightNumber)
{
  LightDevice& light = myLights[index];
  std::string mData = light.device.getManufacturerData();
  Serial.printf("Pairing light MAC %s as #%u\n", macToId(light.lightMac).c_str(), lightNumber);

  uint8_t data[12];
  memcpy(data, light.lightMac, 6); // mac address
  data[6] = lightNumber; // light id - we're requesting that it's set to this
  data[7] = 0x01; // group id
  data[8] = my_key[0];
  data[9] = my_key[1];
  data[10] = my_key[2];
  data[11] = my_key[3];

  pairingIndex = index;
  pairingNumber = lightNumber;
  pBLEScan->setAdvertisedDeviceCallbacks(&addLightCallback, true /* want duplicates */);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  pBLEScan->setActiveScan(true);

  for (int attempt = 1; attempt <= PAIR_ATTEMPTS && !light.isRegistered; attempt++) {
    Serial.printf("  attempt %d/%d\n", attempt, PAIR_ATTEMPTS);
    uint8_t* rfPayload = 0;
    uint8_t rfPayloadLength = do_generate_command(2, data, 12, default_key, false, true, 0, rfPayload);
    std::string serviceData = getServiceData(rfPayloadLength, rfPayload);
    free(rfPayload);
    BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
    oAdvertisementData.setFlags(0x04);
    oAdvertisementData.addData(serviceData);
    pAdvertising->setAdvertisementData(oAdvertisementData);
    pAdvertising->setMinInterval(50);
    pAdvertising->setMaxInterval(50);
    pAdvertising->start();
    pBLEScan->start(PAIR_SCAN_SECONDS, false);
    pBLEScan->clearResults();
    pAdvertising->stop();
    if (!light.isRegistered) delay(300);
  }
  pairingIndex = -1;
  if (!light.isRegistered) Serial.println("  FAILED to pair this light (power-cycle it, then press 'Rescan for Lights' in HA or reboot the ESP32)");
  return light.isRegistered;
}

void addNewLights()
{
  scan();
  // wait before adding lights, as they seem to need a brief pause after the scan
  delay(1000);
  bool changed = false;
  for (size_t i = 0; i < myLights.size(); i++) {
    if (myLights[i].isRegistered || !myLights[i].hasBleAddr) continue;
    // keep the old number for a stored light that was reset, so HA entity stays the same
    uint8_t n = (myLights[i].fromStorage && myLights[i].number != 0) ? myLights[i].number : nextFreeNumber();
    if (pairLight(i, n)) changed = true;
    delay(500);
  }
  // drop lights we saw but failed to pair (unless they're stored ones)
  for (int i = (int)myLights.size() - 1; i >= 0; i--) {
    if (!myLights[i].isRegistered && !myLights[i].fromStorage) myLights.erase(myLights.begin() + i);
  }
  if (changed) saveLights();
}

// Create HA entities for known lights that don't have one yet.
// Returns how many were created.
int createMissingHALights()
{
  int created = 0;
  for (size_t i = 0; i < myLights.size(); i++) {
    LightDevice& l = myLights[i];
    if (l.light == nullptr && (l.isRegistered || l.fromStorage)) {
      createHALight(l);
      created++;
    }
  }
  return created;
}

void startMqtt()
{
  IPAddress brokerIp;
  if (brokerIp.fromString(config.mqttBroker.c_str())) {
    if (config.mqttUsername.empty()) mqtt->begin(brokerIp, config.mqttPort);
    else mqtt->begin(brokerIp, config.mqttPort, config.mqttUsername.c_str(), config.mqttPassword.c_str());
  } else {
    // not an IP, treat it as a hostname
    if (config.mqttUsername.empty()) mqtt->begin(config.mqttBroker.c_str(), config.mqttPort);
    else mqtt->begin(config.mqttBroker.c_str(), config.mqttPort, config.mqttUsername.c_str(), config.mqttPassword.c_str());
  }
}

void onRescanCommand(HAButton* sender)
{
  Serial.println("Rescan requested from Home Assistant");
  rescanRequested = true;
}

// Look for new (factory-key) lights while running, pair them and add them to HA.
void rescanForLights()
{
  digitalWrite(ledPin, HIGH);
  addNewLights();
  int created = createMissingHALights();
  if (created > 0) {
    // ArduinoHA only publishes discovery configs and subscribes to command
    // topics when it connects, so reconnect to announce the new lights.
    Serial.printf("Added %d new light(s), reconnecting MQTT so HA sees them\n", created);
    mqtt->disconnect();
    startMqtt();
  } else {
    Serial.println("Rescan done, no new lights");
  }
  digitalWrite(ledPin, LOW);
}

//////////////////////////////////////////////////////
// CONFIG FILE
//////////////////////////////////////////////////////

bool loadConfig()
{
  if (!LittleFS.begin(false)) {
    Serial.println("ERROR: could not mount LittleFS. Did you run 'Upload Filesystem Image'?");
    return false;
  }
  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) {
    Serial.println("ERROR: " CONFIG_PATH " not found on LittleFS");
    return false;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    Serial.printf("ERROR: could not parse " CONFIG_PATH ": %s\n", err.c_str());
    return false;
  }

  config.wifiSsid     = doc["wifi"]["ssid"]     | "";
  config.wifiPassword = doc["wifi"]["password"] | "";
  config.mqttBroker   = doc["mqtt"]["broker"]   | "";
  config.mqttPort     = doc["mqtt"]["port"]     | 1883;
  config.mqttUsername = doc["mqtt"]["username"] | "";
  config.mqttPassword = doc["mqtt"]["password"] | "";

  if (config.wifiSsid.empty() || config.mqttBroker.empty()) {
    Serial.println("ERROR: config.json needs at least wifi.ssid and mqtt.broker");
    return false;
  }
  Serial.printf("Config loaded: WiFi '%s', MQTT %s:%u, user '%s'\n",
    config.wifiSsid.c_str(), config.mqttBroker.c_str(), config.mqttPort,
    config.mqttUsername.c_str());
  return true;
}

// No usable config: blink the LED fast forever so it's obvious.
void haltWithError()
{
  while (true) {
    digitalWrite(ledPin, HIGH); delay(100);
    digitalWrite(ledPin, LOW);  delay(100);
  }
}

//////////////////////////////////////////////////////
// SETUP / LOOP
//////////////////////////////////////////////////////

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(resetPin, INPUT_PULLUP);
  // turn on to show we're still in setup (and are adding lights)
  digitalWrite(ledPin, HIGH);
  Serial.begin(115200);
  delay(200);

  // Hold the BOOT button while powering on to forget the key + all lights
  if (digitalRead(resetPin) == LOW) clearStorage();

  if (!loadConfig()) haltWithError();

  myLights.reserve(32);
  loadOrCreateKey();
  loadLights();

  // start sequence numbers somewhere random so the lights don't ignore our
  // first commands after a reboot as "already seen"
  SEND_COUNT = esp_random() & 0xFF;

  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));
  device.setName("BRMesh");
  device.setManufacturer("BRMesh");
  device.setModel("BRMesh");
  mqtt = new HAMqtt(client, device, 32);
  Serial.printf("ESP32 MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  BLEDevice::init("BRMesh bridge");
  pAdvertising = BLEDevice::getAdvertising();
  pBLEScan = BLEDevice::getScan();

  WiFi.setAutoReconnect(true);
  WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); // waiting for the connection
    Serial.println("Connecting to WiFi...");
  }

  // pair any lights that are on the default (factory) key
  addNewLights();

  // create HA entities for every known light (stored + newly paired)
  createMissingHALights();

  rescanButton = new HAButton("brmesh_rescan");
  rescanButton->setName("Rescan for Lights");
  rescanButton->setIcon("mdi:refresh-circle");
  rescanButton->onCommand(onRescanCommand);

  digitalWrite(ledPin, LOW);
  Serial.println("Starting MQTT");
  startMqtt();
  Serial.println("Ready");
}

// Debug: type "raw 72 01 FF 00 00 FF 00 00" in the serial monitor to send
// those 8 bytes as a light command (hex). Up to 8 bytes; missing ones are 0.
void handleSerialCommand()
{
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { line += c; continue; }
    line.trim();
    if (line.startsWith("raw ")) {
      uint8_t data[8] = {0};
      int n = 0;
      const char* p = line.c_str() + 4;
      char* end;
      while (n < 8) {
        long v = strtol(p, &end, 16);
        if (end == p) break;
        data[n++] = (uint8_t)v;
        p = end;
      }
      Serial.print("Sending raw: "); dump(data, 8); Serial.print("\n");
      single_control(my_key, data);
    } else if (line.length() > 0) {
      Serial.println("Unknown command. Usage: raw 72 <num> <bright> <B> <R> <G> <W1> <W2>");
    }
    line = "";
  }
}

void loop()
{
  mqtt->loop();
  handleSerialCommand();
  if (rescanRequested) {
    rescanRequested = false;
    rescanForLights();
  }
}