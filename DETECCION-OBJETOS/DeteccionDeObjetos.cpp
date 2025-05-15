/* Includes ---------------------------------------------------------------- */
#include <Componentes_Detect_inferencing.h> // Asegúrate que este nombre de archivo sea correcto
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include "esp_heap_caps.h" // Para heap_caps_check_integrity_all y ps_malloc

#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// --- Configuración WiFi ---
const char* ssid = "SSIDWIFI";
const char* password = "PASSWORDWIFI";

// --- Servidor Web y WebSockets ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- Modelo de Cámara ---
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
// #define CAMERA_MODEL_ESP_EYE

#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 4
#define SIOD_GPIO_NUM 18
#define SIOC_GPIO_NUM 23
#define Y9_GPIO_NUM 36
#define Y8_GPIO_NUM 37
#define Y7_GPIO_NUM 38
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 35
#define Y4_GPIO_NUM 14
#define Y3_GPIO_NUM 13
#define Y2_GPIO_NUM 34
#define VSYNC_GPIO_NUM 5
#define HREF_GPIO_NUM 27
#define PCLK_GPIO_NUM 25
#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#else
#error "Camera model not selected"
#endif

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320 // Ancho de captura de la cámara (QVGA)
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240 // Alto de captura de la cámara (QVGA)
#define EI_CAMERA_FRAME_BYTE_SIZE 3         // Para RGB888

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;
static bool is_camera_initialised = false;

// Buffers asignados globalmente en setup()
uint8_t *snapshot_buf = NULL;           // Buffer para la entrada del modelo EI (ej. 96x96x3)
uint8_t *rgb_intermediate_buffer = NULL; // Buffer para la imagen QVGA decodificada (320x240x3)

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM, .pin_reset = RESET_GPIO_NUM, .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM, .pin_sscb_scl = SIOC_GPIO_NUM, .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM, .pin_d5 = Y7_GPIO_NUM, .pin_d4 = Y6_GPIO_NUM, .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM, .pin_d1 = Y3_GPIO_NUM, .pin_d0 = Y2_GPIO_NUM, .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM, .pin_pclk = PCLK_GPIO_NUM, .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG, .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12, .fb_count = 1, // fb_count = 1 para inferencia, puede ser 2 para web stream si hay memoria
    .fb_location = CAMERA_FB_IN_PSRAM, .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function prototypes */
bool ei_camera_init(void);
bool ei_camera_capture_for_inference(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
String html_processor(const String& var);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <title>ESP32 Deteccion de Objetos - CODELAB</title>
    <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
    <style>
        body {
            font-family: Arial, Helvetica, sans-serif;
            text-align: center;
            margin: 0px auto;
            padding-top: 20px;
            background-color: #f7f7f7;
        }
        h1 {
            color: #333;
        }
        #stream-container {
            position: relative;
            width: 320px;
            height: 240px;
            margin: 20px auto;
            border: 1px solid #ccc;
            background-color: #000;
        }
        #video-stream {
            display: block;
            width: 100%;
            height: 100%;
        }
        #overlay {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
        }
        .info {
            margin: 15px auto;
            max-width: 320px;
            padding: 10px 15px;
            background-color: #fff;
            border-radius: 8px;
            box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
            font-size: 0.95em;
            color: #444;
            line-height: 1.5;
        }
        .info p, .info ul {
            margin: 5px 0;
            padding: 0;
        }
        .info p:first-child {
            font-weight: bold;
            color: red;
            margin-bottom: 10px;
        }
        .info ul {
            list-style-type: disc;
            padding-left: 20px;
            text-align: left;
        }
        .info li {
            margin-bottom: 5px;
        }
        .codelab-title {
            color: red;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <h1 class="codelab-title">CODELAB</h1>
    <h1>ESP32 Deteccion de Objetos</h1>
    <div id="stream-container">
        <img id="video-stream" src="/shot">
        <canvas id="overlay"></canvas>
    </div>
    <div class="info">
        La imagen se refresca. Las detecciones se actualizan en tiempo real.
    </div>
    <div class="info">
        Te invitamos a seguirnos y apoyarnos para poder seguir desarrollando estos proyectos!
    </div>
    <div class="info">
        <p>Visitanos o contactanos:</p>
        <ul>
            <li>roboticacodelab.com - Sitio Web Principal</li>
            <li>roboticacodelab.online - Sitio Web Educativo</li>
            <li>www.instagram.com/robotica_codelab - Instagram</li>
            <li>wa.me/573153338069 - WhatsApp</li>
            <li>RoboticaCodelab@gmail.com - Correo Electronico</li>
        </ul>
    </div>
    <script>
        const video = document.getElementById('video-stream'),
              canvas = document.getElementById('overlay'),
              ctx = canvas.getContext('2d');
        let websocket;
        const eiInputWidth = %EI_WIDTH%, eiInputHeight = %EI_HEIGHT%;

        function initWebSocket() {
            const wsURL = 'ws://' + window.location.hostname + '/ws';
            console.log('Conectando a WebSocket: ' + wsURL);
            websocket = new WebSocket(wsURL);
            websocket.onopen = e => console.log('WebSocket conectado.');
            websocket.onclose = e => {
                console.log('WebSocket desconectado. Intentando reconectar...');
                setTimeout(initWebSocket, 2000);
            };
            websocket.onerror = e => console.error('Error WebSocket:', e);
            websocket.onmessage = e => {
                try {
                    const data = JSON.parse(event.data);
                    data.boxes && drawBoundingBoxes(data.boxes);
                } catch (t) {}
            }
        }

        function drawBoundingBoxes(boxes) {
            const displayWidth = video.parentElement.offsetWidth,
                  displayHeight = video.parentElement.offsetHeight;
            canvas.width = displayWidth;
            canvas.height = displayHeight;
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            if (!boxes || boxes.length === 0) return;

            const scaleX = canvas.width / eiInputWidth,
                  scaleY = canvas.height / eiInputHeight;

            boxes.forEach(box => {
                if (box.value === 0) return;
                const x = box.x * scaleX,
                      y = box.y * scaleY,
                      w = box.width * scaleX,
                      h = box.height * scaleY;

                ctx.strokeStyle = 'lime';
                ctx.lineWidth = 2;
                ctx.strokeRect(x, y, w, h);

                ctx.fillStyle = 'lime';
                ctx.font = '14px Arial';
                const labelText = `${box.label} (${(100 * box.value).toFixed(1)}%)`,
                      textMetrics = ctx.measureText(labelText),
                      textX = x,
                      textY = y > 18 ? y - 5 : y + h + 15;

                ctx.fillStyle = 'rgba(0,0,0,0.5)';
                ctx.fillRect(textX, textY - 14, textMetrics.width + 4, 18);

                ctx.fillStyle = 'lime';
                ctx.fillText(labelText, textX + 2, textY);
            });
        }

        function refreshImage() {
            video.src = "/shot?t=" + new Date().getTime();
        }

        window.onload = () => {
            initWebSocket();
            setInterval(refreshImage, 100);
        };
    </script>
</body>
</html>



)rawliteral";

String html_processor(const String& var){
  if(var == "EI_WIDTH") return String(EI_CLASSIFIER_INPUT_WIDTH);
  if(var == "EI_HEIGHT") return String(EI_CLASSIFIER_INPUT_HEIGHT);
  return String();
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) Serial.printf("Cliente WebSocket #%u conectado desde %s\n", client->id(), client->remoteIP().toString().c_str());
  else if (type == WS_EVT_DISCONNECT) Serial.printf("Cliente WebSocket #%u desconectado\n", client->id());
}

void setup() {
    Serial.begin(115200);
    ei_printf("Inicio de la configuracion...\n");

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Conectando a WiFi...");
    uint8_t wifi_retries = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
        if (++wifi_retries > 20) { Serial.println("\nFallo al conectar WiFi. Reiniciando..."); ESP.restart(); }
    }
    Serial.println("\nWiFi conectado!");
    Serial.print("Dirección IP: "); Serial.println(WiFi.localIP());
    ei_printf("Memoria libre inicial: %d, PSRAM libre: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());

    if (!ei_camera_init()) {
        ei_printf("¡Fallo al inicializar la Cámara! Reiniciando...\r\n");
        delay(1000); ESP.restart();
    }
    ei_printf("Cámara inicializada\r\n");

    // --- Reservar buffers para imágenes ---
    size_t inference_buffer_size = (size_t)EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * EI_CAMERA_FRAME_BYTE_SIZE;
    snapshot_buf = (uint8_t*)malloc(inference_buffer_size);
    if (!snapshot_buf) {
        ei_printf("ERR: Fallo al asignar memoria para snapshot_buf! (Requerido: %u). Reiniciando...\n", inference_buffer_size);
        delay(1000); ESP.restart();
    }
    ei_printf("snapshot_buf asignado: %u bytes en RAM interna (o PSRAM via malloc)\n", inference_buffer_size);

    size_t intermediate_buffer_size = (size_t)EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE;
    #if CONFIG_SPIRAM_SUPPORT || CONFIG_ESP32_SPIRAM_SUPPORT // Definido en sdkconfig si PSRAM está habilitada
        rgb_intermediate_buffer = (uint8_t*)ps_malloc(intermediate_buffer_size);
        ei_printf("Intentando ps_malloc para rgb_intermediate_buffer: %u bytes en PSRAM\n", intermediate_buffer_size);
    #else
        rgb_intermediate_buffer = (uint8_t*)malloc(intermediate_buffer_size); // Caerá a PSRAM si malloc está configurado para ello y es grande
        ei_printf("Intentando malloc para rgb_intermediate_buffer: %u bytes (puede usar PSRAM)\n", intermediate_buffer_size);
    #endif

    if (!rgb_intermediate_buffer) {
        ei_printf("ERR: Fallo al asignar memoria para rgb_intermediate_buffer! (Requerido: %u). Reiniciando...\n", intermediate_buffer_size);
        if (snapshot_buf) free(snapshot_buf);
        delay(1000); ESP.restart();
    }
    ei_printf("rgb_intermediate_buffer asignado.\n");
    ei_printf("Memoria libre tras asignaciones: %d, PSRAM libre: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());
    
    if (!heap_caps_check_integrity_all(true)) {
        ei_printf("CORRUPCION DE HEAP detectada DESPUES de las asignaciones en setup!\n");
    } else {
        ei_printf("Integridad del HEAP OK DESPUES de las asignaciones en setup.\n");
    }

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->send_P(200, "text/html", index_html, html_processor); });
    server.on("/shot", HTTP_GET, [](AsyncWebServerRequest *request){
        camera_fb_t * fb = esp_camera_fb_get();
        if (!fb) { request->send(500, "text/plain", "Error al capturar imagen"); return; }
        request->send_P(200, "image/jpeg", fb->buf, fb->len);
        esp_camera_fb_return(fb);
    });
    server.begin();
    ei_printf("Servidor Web iniciado. Navega a http://%s\n", WiFi.localIP().toString().c_str());
}

void loop() {
    ws.cleanupClients(); 

    // ei_sleep(10); // Pequeña pausa, útil para estabilidad y WDT. Edge Impulse puede tener su propio delay.

    if (!snapshot_buf || !rgb_intermediate_buffer) {
        ei_printf("ERR: Buffers no asignados en loop(). Reiniciando...\n");
        delay(1000); ESP.restart(); // No debería ocurrir si setup tuvo éxito
        return;
    }
    
    // ei_printf("Inicio del loop. Memoria libre: %d, PSRAM libre: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());
    // if (!heap_caps_check_integrity_all(true)) {
    //     ei_printf("CORRUPCION DE HEAP detectada al INICIO del loop!\n");
    //     // Podrías querer detener o reiniciar aquí
    // }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (!ei_camera_capture_for_inference(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
        ei_printf("Fallo al capturar imagen para inferencia. Reintentando...\r\n");
        delay(100); // Pequeña pausa antes de reintentar
        return;
    }

    // if (!heap_caps_check_integrity_all(true)) {
    //    ei_printf("CORRUPCION DE HEAP detectada DESPUES de ei_camera_capture_for_inference!\n");
    // }

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);

    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Fallo al ejecutar el clasificador (%d). Reintentando...\n", err);
        return;
    }
    
    // if (!heap_caps_check_integrity_all(true)) {
    //    ei_printf("CORRUPCION DE HEAP detectada DESPUES de run_classifier!\n");
    // }


    if (ws.count() > 0) {
        String json_payload = "{\"boxes\":[";
        bool first_box = true;
        #if EI_CLASSIFIER_OBJECT_DETECTION == 1
            for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
                ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
                if (bb.value == 0) continue;
                if (!first_box) json_payload += ",";
                json_payload += "{\"label\":\"" + String(bb.label) + "\",\"value\":" + String(bb.value, 5) + 
                                ",\"x\":" + String(bb.x) + ",\"y\":" + String(bb.y) + 
                                ",\"width\":" + String(bb.width) + ",\"height\":" + String(bb.height) + "}";
                first_box = false;
            }
        #endif
        json_payload += "]}";
        ws.textAll(json_payload);
    }
}

bool ei_camera_init(void) {
    if (is_camera_initialised) return true;
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Fallo en la inicialización de la cámara con error 0x%x\n", err);
        return false;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) { // Ajustes específicos del sensor
        s->set_vflip(s, 1); s->set_brightness(s, 1); s->set_saturation(s, 0);
    }
    is_camera_initialised = true;
    return true;
}

bool ei_camera_capture_for_inference(uint32_t img_width, uint32_t img_height, uint8_t *out_buf /*snapshot_buf*/) {
    if (!is_camera_initialised) {
        ei_printf("ERR: Cámara no inicializada al intentar capturar\r\n");
        return false;
    }
    if (!rgb_intermediate_buffer || !out_buf) {
         ei_printf("ERR: Buffers no asignados en ei_camera_capture_for_inference\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Fallo en la captura de la cámara (esp_camera_fb_get)\n");
        return false;
    }

    bool conversion_success = false;
    if (fb->format == PIXFORMAT_JPEG) {
        conversion_success = fmt2rgb888(fb->buf, fb->len, fb->format, ::rgb_intermediate_buffer); // Usar global
    } else {
        ei_printf("ERR: Formato de pixel de cámara no es JPEG (%d). Se requiere JPEG.\n", fb->format);
        esp_camera_fb_return(fb);
        return false;
    }
    
    esp_camera_fb_return(fb); // Liberar buffer de la cámara lo antes posible

    if (!conversion_success) {
        ei_printf("ERR: Fallo en la conversión JPEG a RGB888\n");
        return false;
    }

    // Si la integridad del heap falla aquí, fmt2rgb888 es el culpable
    // if (!heap_caps_check_integrity_all(true)) {
    //    ei_printf("CORRUPCION DE HEAP detectada DESPUES de fmt2rgb888!\n");
    //    return false; // Detenerse para evitar más daño
    // }

    // Reescalar/recortar la imagen RGB (320x240) al tamaño de entrada del modelo (ej. 96x96)
    // y guardarla en 'out_buf' (que es 'snapshot_buf').
    // ¡¡¡ESTA ES LA FUNCIÓN SOSPECHOSA SEGÚN TU BACKTRACE ANTERIOR!!!
    ei::image::processing::crop_and_interpolate_rgb888(
        ::rgb_intermediate_buffer, // Origen: QVGA RGB
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,                   // Destino: buffer para el modelo EI
        img_width,                 // Ancho de la imagen del modelo EI
        img_height                 // Alto de la imagen del modelo EI
    );
    
    // Si la integridad del heap falla aquí, crop_and_interpolate_rgb888 es el culpable
    // if (!heap_caps_check_integrity_all(true)) {
    //    ei_printf("CORRUPCION DE HEAP detectada DESPUES de crop_and_interpolate_rgb888!\n");
    //    return false; // Detenerse para evitar más daño
    // }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    if (!snapshot_buf) return -1; // Seguridad
    size_t pixel_buffer_ix = offset * EI_CAMERA_FRAME_BYTE_SIZE;
    for (size_t i = 0; i < length; i++) {
        uint8_t r = snapshot_buf[pixel_buffer_ix++];
        uint8_t g = snapshot_buf[pixel_buffer_ix++];
        uint8_t b = snapshot_buf[pixel_buffer_ix++];
        out_ptr[i] = (float)((r << 16) | (g << 8) | b);
    }
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "El modelo de inferencia no está configurado para un sensor de CÁMARA."
#endif
