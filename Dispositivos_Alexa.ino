#ifdef ENABLE_DEBUG
#define DEBUG_ESP_PORT Serial
#define NODEBUG_WEBSOCKETS
#define NDEBUG
#endif

#include <Arduino.h>
#ifdef ESP8266
#include <ESP8266WiFi.h>
#endif
#ifdef ESP32
#include <WiFi.h>
#endif

#include "SinricPro.h"
#include "SinricProContactsensor.h"
#include "SinricPro.h"
#include "SinricProTemperaturesensor.h"
#include "DHT.h"

#define WIFI_SSID         "YOUR-WIFI-SSID"
#define WIFI_PASS         "YOUR-WIFI-PASSWORD"
#define APP_KEY           "YOUR-APP-KEY"
#define APP_SECRET        "YOUR-APP-SECRET"
#define CONTACT_ID        "YOUR_CONTACT_ID"
#define TEMP_SENSOR_ID    "YOUR_TEMP_ID"
#define EVENT_WAIT_TIME   60000
#define BAUD_RATE         9600                // Altere o Baud Rate se necessário
#define CONTACT_PIN       0                   // PINO onde o sensor touch está conectado
#define DHT_PIN    2                          // PINO onde o sensor de tmperatura está conectado

DHT dht;                                      // DHT sensor

bool deviceIsOn;                              // Estado do sensor de temperatura
float temperature;                            // temperatura atual
float humidity;                               // umidade atual
float lastTemperature;                        // temperatura anterior
float lastHumidity;                           // umidade anterior
unsigned long lastEvent = (-EVENT_WAIT_TIME); // última mudança
bool thingState = false;                      // estado do sensor touch
bool myPowerState = true;                     // estado do sensor touch (se ligado ou desligado)
bool lastContactState = false;                // último estado
unsigned long lastChange = 0;                 // última mudança do sensor

/**
   @brief Checks contactsensor connected to CONTACT_PIN

   If contactsensor state has changed, send event to SinricPro Server
   state from digitalRead():
        HIGH = contactsensor is closed
        LOW  = contactsensor is open
*/
void handleContactsensor() {
  if (!myPowerState) return;                            // se o touch foi desconectado

  unsigned long actualMillis = millis();
  if (actualMillis - lastChange < 250) return;          // debounce para evitar ruídos

  bool actualContactState = digitalRead(CONTACT_PIN);   // lê o estado atual do sensor

  if (actualContactState == true && lastContactState == false) {         // se houve mudança de estado
    Serial.printf("Contactsensor is %s now\r\n", actualContactState ? "open" : "closed");
    lastChange = actualMillis;                          // inicia o tempo de debounce
    thingState = !thingState;
    SinricProContactsensor &myContact = SinricPro[CONTACT_ID]; // captura o valor a partir do sinric pro
    myContact.sendContactEvent(thingState);      // envia o estado do sensor touch
  }
  lastContactState = actualContactState;              // atualiza o último estado
}

void handleTemperaturesensor() {
  if (deviceIsOn == false) return; // se o dht foi desconectado

  unsigned long actualMillis = millis();
  if (actualMillis - lastEvent < EVENT_WAIT_TIME) return; //tempo para nova leitura

  temperature = dht.getTemperature();          // lê temperatura atual em °C
  //  temperature = dht.getTemperature() * 1.8f + 32;  // lê temperatura atual em °F
  humidity = dht.getHumidity();                // lê umidade atual

  if (isnan(temperature)) { // se houve falha na leitura ou não houve mudança de temperatura
    Serial.printf("DHT reading failed!\r\n");  // print mensagem de erro
    return;                                    // faça nova leitura
  }

  if (temperature == lastTemperature || humidity == lastHumidity) return;

  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];  // leia a temperatura do sinric pro
  bool success = mySensor.sendTemperatureEvent(temperature, humidity); // envia a nova temperatura
  if (success) {  // se enviou com sucesso, print novos valores
    Serial.printf("Temperature: %2.1f Celsius\tHumidity: %2.1f%%\r\n", temperature, humidity);
  } else {  // se o onvio falhou, print mensagem de erro
    Serial.printf("Something went wrong...could not send Event to server!\r\n");
  }

  lastTemperature = temperature;  // salve a temperatura atual
  lastHumidity = humidity;        // salve a umidade atual atual
  lastEvent = actualMillis;       // salve o tempo atual
}


/**
   @brief Callback for setPowerState request

   @param deviceId      String containing deviceId (useful if this callback used by multiple devices)
   @param[in] state     bool true=turn on device / false=turn off device
   @param[out] state    bool true=device turned on / false=device turned off
   @return true         request handled properly
   @return false        request can't be handled because some kind of error happened
*/
bool onPowerState1(const String &deviceId, bool &state) {
  Serial.printf("Device %s turned %s (via SinricPro) \r\n", deviceId.c_str(), state ? "on" : "off");
  myPowerState = state;
  return true;
}
bool onPowerState2(const String &deviceId, bool &state) {
  Serial.printf("Temperaturesensor turned %s (via SinricPro) \r\n", state ? "on" : "off");
  deviceIsOn = state;
  return true;
}


// setup function for WiFi connection
void setupWiFi() {
  Serial.printf("\r\n[Wifi]: Connecting");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf(".");
    delay(250);
  }
  IPAddress localIP = WiFi.localIP();
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %d.%d.%d.%d\r\n", localIP[0], localIP[1], localIP[2], localIP[3]);
}

// setup function for SinricPro
void setupSinricPro() {
  // add device to SinricPro
  SinricProContactsensor& myContact = SinricPro[CONTACT_ID];
  myContact.onPowerState(onPowerState1);
  SinricProTemperaturesensor &mySensor = SinricPro[TEMP_SENSOR_ID];
  mySensor.onPowerState(onPowerState2);

  // setup SinricPro
  SinricPro.onConnected([]() {
    Serial.printf("Connected to SinricPro\r\n");
  });
  SinricPro.onDisconnected([]() {
    Serial.printf("Disconnected from SinricPro\r\n");
  });
  SinricPro.begin(APP_KEY, APP_SECRET);
  SinricPro.restoreDeviceStates(true);
}

// main setup function
void setup() {
  Serial.begin(BAUD_RATE); Serial.printf("\r\n\r\n");
  dht.setup(DHT_PIN);
  pinMode(CONTACT_PIN, INPUT);

  setupWiFi();
  setupSinricPro();
}

void loop() {
  handleContactsensor();
  handleTemperaturesensor();
  SinricPro.handle();
}