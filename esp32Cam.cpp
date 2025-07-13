/* ======================================== Including the libraries. */
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "quirc.h"
#include <HTTPClient.h>
#include <WiFi.h>
/* ======================================== */

/* ======================================== Select camera model */
#define CAMERA_MODEL_AI_THINKER
/* ======================================== */

/* ======================================== GPIO de cámara para AI_THINKER */
#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27

  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif
/* ======================================== */

/* ======================================== Pines LED */
#define FLASH_LED_PIN 4    // Flash LED (se enciende solo al leer QR)
#define GREEN_LED_PIN 12   // Acceso concedido
#define RED_LED_PIN   13   // Acceso denegado / Token inválido
/* ======================================== */

/* ======================================== WiFi credentials */
const char* ssid     = "$88$";
const char* password = "metenespodridoconlaspapitas";
/* ======================================== */

/* ======================================== Tarea QRCodeReader handle */
TaskHandle_t QRCodeReader_Task;
/* ======================================== */

/* ======================================== Variables y estructuras QR */
struct quirc *q        = nullptr;
uint8_t    *image      = nullptr;
camera_fb_t *fb        = nullptr;
struct quirc_code code;
struct quirc_data data;
quirc_decode_error_t err;
String QRCodeResult = "";
/* ======================================== */

/* ======================================== Constantes de configuración */
const int MAX_PAYLOAD_SIZE    = 1024;
const int TIME_QR_COUNTDOWN_S = 15;     // Segundos de cuenta regresiva al 200
const int TIME_RESET_QR_MS    = 15000;  // LED rojo acceso denegado
const int TIME_TOKEN_ERROR_MS = 5000;   // LED rojo token inválido
/* ======================================== */

/* ======================================== Prototipos de funciones */
int   sendQRCodeToAPI(String qrContent);
void  cuentaRegresivaQR(int segundos);
void  prenderLedPorTiempo(int pin, int tiempo_ms);
void  mensajeReinicio();
void  dumpData(const struct quirc_data *data);
void  QRCodeReader(void * pvParameters);
/* ======================================== */

/* ---------------------------------------------------------------------------------
   Función sendQRCodeToAPI: envía POST JSON y retorna código HTTP.
--------------------------------------------------------------------------------- */
int sendQRCodeToAPI(String qrContent) {
  HTTPClient http;
  const char* url = "https://ymucpmxkp3.us-east-1.awsapprunner.com/api/access/check";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");

  String body = "{\"token\":\"" + qrContent + "\"}";
  int httpCode = http.POST(body);

  if (httpCode > 0) {
    Serial.printf("Código de respuesta: %d\n", httpCode);
    String response = http.getString();
    Serial.println("Respuesta del servidor: " + response);
  } else {
    Serial.printf("Error en la solicitud: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
  return httpCode;
}
/* ---------------------------------------------------------------------------------
   Funciones auxiliares para modularizar LEDs y cuenta regresiva
--------------------------------------------------------------------------------- */
void cuentaRegresivaQR(int segundos) {
  for (int i = segundos; i >= 1; i--) {
    Serial.print("Reseteando QR en ");
    Serial.print(i);
    Serial.println(" segundos...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void prenderLedPorTiempo(int pin, int tiempo_ms) {
  digitalWrite(pin, HIGH);
  vTaskDelay(tiempo_ms / portTICK_PERIOD_MS);
  digitalWrite(pin, LOW);
}

void mensajeReinicio() {
  Serial.println("reiniciando...en:");
  Serial.println("3...");
  Serial.println("2");
  Serial.println("1");
}

/* ---------------------------------------------------------------------------------
   Función dumpData: procesa datos del QR, enciende flash al inicio, apaga al final.
--------------------------------------------------------------------------------- */
void dumpData(const struct quirc_data *data) {
  // 0) Encender flash LED solo al procesar QR válido
  digitalWrite(FLASH_LED_PIN, HIGH);

  // 1) Copiar payload en buffer seguro
  char payloadBuffer[MAX_PAYLOAD_SIZE + 1];
  int len = data->payload_len;
  if (len > MAX_PAYLOAD_SIZE) len = MAX_PAYLOAD_SIZE;
  memcpy(payloadBuffer, data->payload, len);
  payloadBuffer[len] = '\0';

  // 2) Mostrar info de decodificación
  Serial.printf("Version: %d\n",   data->version);
  Serial.printf("ECC level: %c\n", "MLHQ"[data->ecc_level]);
  Serial.printf("Mask: %d\n",      data->mask);
  Serial.printf("Length: %d\n",    data->payload_len);
  Serial.printf("Payload: %s\n",   payloadBuffer);

  // 3) Enviar a API
  QRCodeResult = String(payloadBuffer);
  Serial.println("qr leído con éxito, aguarde...");
  int responseCode = sendQRCodeToAPI(QRCodeResult);

  // 4) Manejo de respuestas HTTP
  if (responseCode == 200) {
    Serial.println("acceso concedido");
    digitalWrite(GREEN_LED_PIN, HIGH);
    cuentaRegresivaQR(TIME_QR_COUNTDOWN_S);
    digitalWrite(GREEN_LED_PIN, LOW);
    Serial.println("QR eliminado");

  } else if (responseCode == 403 || responseCode == -1) {
    Serial.println("acceso denegado");
    prenderLedPorTiempo(RED_LED_PIN, TIME_RESET_QR_MS);
    mensajeReinicio();

  } else if (responseCode == 401 || responseCode == -11|| responseCode == 404) {
    Serial.println("Token inválido o vencido");
    prenderLedPorTiempo(RED_LED_PIN, TIME_TOKEN_ERROR_MS);

  } else {
    Serial.print("Código HTTP recibido: ");
    Serial.println(responseCode);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }

  // 5) Apagar flash LED
  digitalWrite(FLASH_LED_PIN, LOW);

  // 6) Reset para próxima lectura
  QRCodeResult = "";
}

/* ---------------------------------------------------------------------------------
   setup(): inicializa Serial, WiFi, cámara y crea tarea QRCodeReader.
--------------------------------------------------------------------------------- */
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Deshabilita brownout detector

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  Serial.print("Conectando a WiFi ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi conectado, IP: ");
  Serial.println(WiFi.localIP());

  // Configurar LEDs
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);

  // Configuración cámara
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_VGA;
  config.jpeg_quality = 60;
  config.fb_count     = 2;

  esp_err_t errCam = esp_camera_init(&config);
  if (errCam != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", errCam);
    ESP.restart();
  }
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, config.frame_size);
  Serial.println("Cámara iniciada correctamente.\n");

  // Crear tarea QRCodeReader en core 0
  xTaskCreatePinnedToCore(
    QRCodeReader,
    "QRCodeReader_Task",
    10000,
    NULL,
    1,
    &QRCodeReader_Task,
    0
  );
}

/* ---------------------------------------------------------------------------------
   loop vacío (todo se maneja en QRCodeReader).
--------------------------------------------------------------------------------- */
void loop() {
  vTaskDelay(1 / portTICK_PERIOD_MS);
}

/* ---------------------------------------------------------------------------------
   Tarea QRCodeReader: captura, decodifica y llama a dumpData().
--------------------------------------------------------------------------------- */
void QRCodeReader(void * pvParameters) {
  Serial.println("QRCodeReader listo en core:");
  Serial.println(xPortGetCoreID());
  Serial.println("Intentando leer QR, aguarde...\n");

  for (;;) {
    Serial.println("escaneando...");
    q = quirc_new();
    if (!q) {
      Serial.println("No se pudo crear objeto quirc");
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Fallo captura cámara");
      quirc_destroy(q);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      continue;
    }
    quirc_resize(q, fb->width, fb->height);
    image = quirc_begin(q, NULL, NULL);
    memcpy(image, fb->buf, fb->len);
    quirc_end(q);

    int count = quirc_count(q);
    if (count > 0) {
      quirc_extract(q, 0, &code);
      err = quirc_decode(&code, &data);
      if (err) {
        Serial.println("Decoding FAILED");
        QRCodeResult = "Decoding FAILED";
        vTaskDelay(2500 / portTICK_PERIOD_MS);
      } else {
        dumpData(&data);
      }
      Serial.println();
    } else {
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    esp_camera_fb_return(fb);
    fb    = nullptr;
    image = nullptr;
    quirc_destroy(q);
  }
}
