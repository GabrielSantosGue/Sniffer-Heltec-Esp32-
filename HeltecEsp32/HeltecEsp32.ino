
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string>
#include <cstddef>
#include <Wire.h>
#include <Preferences.h>
#include "heltec.h"
#include "FS.h"
#include "SD.h"
#include "Buffer.h"


using namespace std;

/* ===== configuración de compilación ===== */
#define MAX_CH 14       // 1 - 14 canales 
#define SNAP_LEN 2324   // longitud máxima de cada paquete recibido
#define BUTTON_PIN 5    // botón para cambiar de canal
#define USE_DISPLAY     
#define FLIP_DISPLAY    
#define SDA_PIN 26
#define SCL_PIN 27
#define MAX_X 128
#define MAX_Y 51
#if CONFIG_FREERTOS_UNICORE
#define RUNNING_CORE 0
#else
#define RUNNING_CORE 1
#endif





esp_err_t event_handler(void* ctx, system_event_t* event) {
  return ESP_OK;
}

/* ===== run-time variables ===== */
Buffer sdBuffer;

Preferences preferences;

bool useSD = false;
bool buttonPressed = false;
bool buttonEnabled = true;
uint32_t lastDrawTime;
uint32_t lastButtonTime;
uint32_t tmpPacketCounter;
uint32_t pkts[MAX_X];       // aquí se guardarán los paquetes por segundo
uint32_t deauths = 0;       // cuadros de muerte por segundo
unsigned int ch = 1;        // canal 802.11 actual
int rssiSum;

/* ===== functions ===== */
double getMultiplicator() {
  uint32_t maxVal = 1;
  for (int i = 0; i < MAX_X; i++) {
    if (pkts[i] > maxVal) maxVal = pkts[i];
  }
  if (maxVal > MAX_Y) return (double)MAX_Y / (double)maxVal;
  else return 1;
}

void setChannel(int newChannel) {
  ch = newChannel;
  if (ch > MAX_CH || ch < 1) ch = 1;

  preferences.begin("packetmonitor32", false);
  preferences.putUInt("channel", ch);
  preferences.end();

  esp_wifi_set_promiscuous(false);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous);
  esp_wifi_set_promiscuous(true);
}

bool setupSD() {
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return false;
  }

  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD_MMC card attached");
    return false;
  }

  Serial.print("SD_MMC Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);

  return true;
}

void wifi_promiscuous(void* buf, wifi_promiscuous_pkt_type_t type) {
  wifi_promiscuous_pkt_t* pkt = (wifi_promiscuous_pkt_t*)buf;
  wifi_pkt_rx_ctrl_t ctrl = (wifi_pkt_rx_ctrl_t)pkt->rx_ctrl;
  if (type == WIFI_PKT_MGMT && (pkt->payload[0] == 0xA0 || pkt->payload[0] == 0xC0 )) deauths++;
  if (type == WIFI_PKT_MISC) return;             // tipo de paquete incorrecto
  if (ctrl.sig_len > SNAP_LEN) return;           // Paquete demasiado largo(masculino)
  uint32_t packetLength = ctrl.sig_len;
  if (type == WIFI_PKT_MGMT) packetLength -= 4;  // Corrección de un error conocido en las FDI.
  tmpPacketCounter++;
  rssiSum += ctrl.rssi;
  if (useSD) sdBuffer.addPacket(pkt->payload, packetLength);
}

void draw() {
#ifdef USE_DISPLAY
  double multiplicator = getMultiplicator();
  int len;
  int rssi;
  if (pkts[MAX_X - 1] > 0) rssi = rssiSum / (int)pkts[MAX_X - 1];
  else rssi = rssiSum;
  Heltec.display -> clear(); 
  Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
  Heltec.display->drawString( 10, 0, (String)ch);
  Heltec.display->drawString( 14, 0, ("|"));
  Heltec.display->drawString( 30, 0, (String)rssi);
  Heltec.display->drawString( 34, 0, ("|"));
  Heltec.display->drawString( 82, 0, (String)tmpPacketCounter);
  Heltec.display->drawString( 87, 0, ("["));
  Heltec.display->drawString(106, 0, (String)deauths);
  Heltec.display->drawString(110, 0, ("]"));
  Heltec.display->drawString(114, 0, ("|"));
  Heltec.display->drawString(128, 0, (useSD ? "SD" : ""));
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->drawString( 36,  0, ("Pkts:"));
  Heltec.display->drawLine(0, 63 - MAX_Y, MAX_X, 63 - MAX_Y);
  for (int i = 0; i < MAX_X; i++) {
    len = pkts[i] * multiplicator;
    Heltec.display->drawLine(i, 63, i, 63 - (len > MAX_Y ? MAX_Y : len));
    if (i < MAX_X - 1) pkts[i] = pkts[i + 1];
  }
  Heltec.display->display();
#endif
}

/* ===== main program ===== */
void setup() {

  // Serial
  Serial.begin(115200);
  pinMode(LED,OUTPUT);
  digitalWrite(LED,HIGH);
  Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Enable*/, true /*Serial Enable*/);

  // Configuraciones
  preferences.begin("packetmonitor32", false);
  ch = preferences.getUInt("channel", 1);
  preferences.end();

  // Sistema y Wi-Fi
  nvs_flash_init();
  tcpip_adapter_init();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
  ESP_ERROR_CHECK(esp_wifi_start());

  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);

  // SD card
  sdBuffer = Buffer();

  if (setupSD())
    sdBuffer.open(&SD);

  // I/O
  pinMode(BUTTON_PIN, INPUT_PULLUP);


  /* mostrar pantalla de inicio */
  Heltec.display -> clear();   
  Heltec.display->drawString(40, 6, "Esp32-Sniffer");
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(6, 24, "@Autores");
  Heltec.display->drawString(6, 38, "Sebastian Guallasamin");
  Heltec.display->drawString(6, 48, "Gabriel Santos");
  Heltec.display->display();

  delay(10000);
//#endif

  // segundo núcleo
  xTaskCreatePinnedToCore(
    coreTask,               /* Función para implementar la tarea. */
    "coreTask",             /* Nombre de la tarea */
    2500,                   /* Tamaño de pila en palabras */
    NULL,                   /* Parámetro de entrada de tarea */
    0,                      /* prioridad de la tarea */
    NULL,                   /* Identificador de tareas. */
    RUNNING_CORE);          /*Identificador de tareas */

  // iniciar rastreador wifi
  esp_wifi_set_promiscuous_rx_cb(&wifi_promiscuous);
  esp_wifi_set_promiscuous(true);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

void coreTask( void * p ) {

  uint32_t currentTime;

  while (true) {

    currentTime = millis();


    // botón de verificación
    if (digitalRead(BUTTON_PIN) == LOW) {
      if (buttonEnabled) {
        if (!buttonPressed) {
          buttonPressed = true;
          lastButtonTime = currentTime;
        } else if (currentTime - lastButtonTime >= 2000) {
          if (useSD) {
            useSD = false;
            sdBuffer.close(&SD);
            draw();
          } else {
            if (setupSD())
              sdBuffer.open(&SD);
            draw();
          }
          buttonPressed = false;
          buttonEnabled = false;
        }
      }
    } else {
      if (buttonPressed) {
        setChannel(ch + 1);
        draw();
      }
      buttonPressed = false;
      buttonEnabled = true;
    }

    // guardar búfer en SD
    if (useSD)
      sdBuffer.save(&SD);

    // Dibujar pantalla
    if ( currentTime - lastDrawTime > 1000 ) {
      lastDrawTime = currentTime;

      pkts[MAX_X - 1] = tmpPacketCounter;

      draw();

      Serial.println((String)pkts[MAX_X - 1]);

      tmpPacketCounter = 0;
      deauths = 0;
      rssiSum = 0;
    }

    // entrada serie
    if (Serial.available()) {
      ch = Serial.readString().toInt();
      if (ch < 1 || ch > 14) ch = 1;
      setChannel(ch);
    }

  }

}
