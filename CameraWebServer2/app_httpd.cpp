// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
// Licensed under the Apache License, Version 2.0

#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "esp32-hal-ledc.h"
#include "sdkconfig.h"
#include "camera_index.h"
#include "board_config.h"
#include "Arduino.h"
#include "HardwareSerial.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

// ── LED config ───────────────────────────────────────────────────────────────
#define LED_PIN        LED_GPIO_NUM
#define LED_FREQ       5000
#define LED_RESOLUTION 8

// ── Nano Serial config ───────────────────────────────────────────────────────
// AI Thinker ESP32-CAM: NO Serial2. Use Serial1 remapped to safe pins.
// GPIO14 = camera HREF pin — NEVER use for UART.
// GPIO2 = safe TX after boot (strapping pin, must be LOW at power-on).
// GPIO13 = unused RX side.
//
// Wiring (only 2 wires needed):
//   ESP32-CAM GPIO2  ──────►  Arduino Nano Pin 0 (RX)   [Yellow]
//   ESP32-CAM GND     ──────►  Arduino Nano GND           [Black]
//
// !! Disconnect the GPIO2→Nano wire before uploading code to the Nano !!
// ─────────────────────────────────────────────────────────────────────────────
#define NANO_SERIAL   Serial1
#define NANO_TX_PIN   2    // GPIO12 → Nano RX (pin 0)
#define NANO_RX_PIN   13    // GPIO13 → unused
#define NANO_BAUD     115200

// ─────────────────────────────────────────────────────────────────────────────

#if defined(LED_GPIO_NUM)
#define CONFIG_LED_MAX_INTENSITY 255
int  led_duty   = 0;
bool isStreaming = false;
#endif

typedef struct {
  httpd_req_t* req;
  size_t       len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

typedef struct {
  size_t size, index, count;
  int    sum;
  int*   values;
} ra_filter_t;

static ra_filter_t ra_filter;

static ra_filter_t* ra_filter_init(ra_filter_t* filter, size_t sample_size) {
  memset(filter, 0, sizeof(ra_filter_t));
  filter->values = (int*)malloc(sample_size * sizeof(int));
  if (!filter->values) return NULL;
  memset(filter->values, 0, sample_size * sizeof(int));
  filter->size = sample_size;
  return filter;
}

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
static int ra_filter_run(ra_filter_t* filter, int value) {
  if (!filter->values) return value;
  filter->sum -= filter->values[filter->index];
  filter->values[filter->index] = value;
  filter->sum += filter->values[filter->index];
  filter->index = (filter->index + 1) % filter->size;
  if (filter->count < filter->size) filter->count++;
  return filter->sum / filter->count;
}
#endif

#if defined(LED_GPIO_NUM)
void enable_led(bool en) {
  int duty = en ? led_duty : 0;
  if (en && isStreaming && led_duty > CONFIG_LED_MAX_INTENSITY)
    duty = CONFIG_LED_MAX_INTENSITY;
  ledcWrite(LED_PIN, duty);
  log_i("LED intensity: %d", duty);
}
#endif

// ═══════════════════════════════════════════════════════════════════════════
// Dashboard /cmd handler
//
// The Nano sketch uses: Serial.readStringUntil('\n') then string compare.
// So we MUST send newline-terminated strings. println() adds '\n'.
//
// Full command table:
//   Dashboard sends  │  ESP32 sends to Nano  │  Nano does
//   ─────────────────┼───────────────────────┼──────────────────────
//   FORWARD          │  "W\n"                │  step_forward(1)
//   BACKWARD         │  "S\n"                │  step_back(1)
//   LEFT             │  "A\n"                │  turn_left(1)
//   RIGHT            │  "D\n"                │  turn_right(1)
//   LEVEL1           │  "!\n"                │  set_height_level(1)
//   LEVEL2           │  "@\n"                │  set_height_level(2)
//   LEVEL3           │  "#\n"                │  set_height_level(3)
//   GESTURE_I        │  "I\n"                │  hand_shake(n_step)
//   GESTURE_O        │  "O\n"                │  hand_wave(n_step)
//   0–100  (X axis)  │  "50\n"  (example)    │  motorX.write(angle)
//   101–200 (Y axis) │  "150\n" (example)    │  motorY.write(angle)
// ═══════════════════════════════════════════════════════════════════════════
static esp_err_t dashboard_cmd_handler(httpd_req_t* req) {
  char  val_str[32] = {0};

  // Parse query string manually (no helper needed for single param)
  size_t qlen = httpd_req_get_url_query_len(req) + 1;
  if (qlen <= 1) { httpd_resp_send_404(req); return ESP_FAIL; }
  char* buf = (char*)malloc(qlen);
  if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }
  if (httpd_req_get_url_query_str(req, buf, qlen) != ESP_OK ||
      httpd_query_key_value(buf, "val", val_str, sizeof(val_str)) != ESP_OK) {
    free(buf); httpd_resp_send_404(req); return ESP_FAIL;
  }
  free(buf);

  log_i("Dashboard cmd: [%s]", val_str);

  // ── Direction ────────────────────────────────────────────────────────────
  if      (strcmp(val_str, "FORWARD")   == 0) { NANO_SERIAL.println("W"); log_i("→ W"); }
  else if (strcmp(val_str, "BACKWARD")  == 0) { NANO_SERIAL.println("S"); log_i("→ S"); }
  else if (strcmp(val_str, "LEFT")      == 0) { NANO_SERIAL.println("A"); log_i("→ A"); }
  else if (strcmp(val_str, "RIGHT")     == 0) { NANO_SERIAL.println("D"); log_i("→ D"); }
  // ── Height levels ────────────────────────────────────────────────────────
  else if (strcmp(val_str, "LEVEL1")    == 0) { NANO_SERIAL.println("1"); log_i("→ 1"); }
  else if (strcmp(val_str, "LEVEL2")    == 0) { NANO_SERIAL.println("2"); log_i("→ 2"); }
  else if (strcmp(val_str, "LEVEL3")    == 0) { NANO_SERIAL.println("3"); log_i("→ 3"); }
  // ── Gestures ─────────────────────────────────────────────────────────────
  else if (strcmp(val_str, "GESTURE_I") == 0) { NANO_SERIAL.println("I"); log_i("→ I"); }
  else if (strcmp(val_str, "GESTURE_O") == 0) { NANO_SERIAL.println("O"); log_i("→ O"); }
  // ── Slider values (numeric string, e.g. "50" or "150") ──────────────────
  else {
    int v = atoi(val_str);
    if ((v >= 0 && v <= 100) || (v >= 101 && v <= 200)) {
      NANO_SERIAL.println(v);   // sends "50\n" etc — Nano reads with readStringUntil('\n')
      log_i("→ slider %d", v);
    } else {
      log_i("Ignored unknown: [%s]", val_str);
    }
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

// ─────────────────────────────────────────────────────────────────────────────

static esp_err_t bmp_handler(httpd_req_t* req) {
  camera_fb_t* fb  = NULL;
  esp_err_t    res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  uint64_t fr_start = esp_timer_get_time();
#endif
  fb = esp_camera_fb_get();
  if (!fb) { log_e("Capture failed"); httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/x-windows-bmp");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.bmp");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char ts[32];
  snprintf(ts, 32, "%lld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
  httpd_resp_set_hdr(req, "X-Timestamp", ts);
  uint8_t* bmp_buf = NULL; size_t bmp_len = 0;
  bool ok = frame2bmp(fb, &bmp_buf, &bmp_len);
  esp_camera_fb_return(fb);
  if (!ok) { log_e("BMP conversion failed"); httpd_resp_send_500(req); return ESP_FAIL; }
  res = httpd_resp_send(req, (const char*)bmp_buf, bmp_len);
  free(bmp_buf);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  uint64_t fr_end = esp_timer_get_time();
  log_i("BMP: %llums %uB", (uint64_t)((fr_end - fr_start) / 1000), bmp_len);
#endif
  return res;
}

static size_t jpg_encode_stream(void* arg, size_t index, const void* data, size_t len) {
  jpg_chunking_t* j = (jpg_chunking_t*)arg;
  if (!index) j->len = 0;
  if (httpd_resp_send_chunk(j->req, (const char*)data, len) != ESP_OK) return 0;
  j->len += len;
  return len;
}

static esp_err_t capture_handler(httpd_req_t* req) {
  camera_fb_t* fb  = NULL;
  esp_err_t    res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_start = esp_timer_get_time();
#endif
#if defined(LED_GPIO_NUM)
  enable_led(true);
  vTaskDelay(150 / portTICK_PERIOD_MS);
  fb = esp_camera_fb_get();
  enable_led(false);
#else
  fb = esp_camera_fb_get();
#endif
  if (!fb) { log_e("Capture failed"); httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char ts[32];
  snprintf(ts, 32, "%lld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
  httpd_resp_set_hdr(req, "X-Timestamp", ts);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  size_t fb_len = 0;
#endif
  if (fb->format == PIXFORMAT_JPEG) {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    fb_len = fb->len;
#endif
    res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  } else {
    jpg_chunking_t jchunk = {req, 0};
    res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
    httpd_resp_send_chunk(req, NULL, 0);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    fb_len = jchunk.len;
#endif
  }
  esp_camera_fb_return(fb);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_end = esp_timer_get_time();
  log_i("JPG: %uB %ums", (uint32_t)fb_len, (uint32_t)((fr_end - fr_start) / 1000));
#endif
  return res;
}

static esp_err_t stream_handler(httpd_req_t* req) {
  camera_fb_t*    fb          = NULL;
  struct timeval  _timestamp;
  esp_err_t       res         = ESP_OK;
  size_t          _jpg_buf_len = 0;
  uint8_t*        _jpg_buf    = NULL;
  char*           part_buf[128];
  static int64_t  last_frame  = 0;
  if (!last_frame) last_frame = esp_timer_get_time();

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "60");
#if defined(LED_GPIO_NUM)
  isStreaming = true;
  enable_led(true);
#endif
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      log_e("Capture failed"); res = ESP_FAIL;
    } else {
      _timestamp.tv_sec  = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;
      if (fb->format != PIXFORMAT_JPEG) {
        bool ok = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb); fb = NULL;
        if (!ok) { log_e("JPEG compression failed"); res = ESP_FAIL; }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf     = fb->buf;
      }
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf((char*)part_buf, 128, _STREAM_PART,
                             _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, (const char*)part_buf, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)_jpg_buf, _jpg_buf_len);
    if (fb)       { esp_camera_fb_return(fb); fb = NULL; _jpg_buf = NULL; }
    else if (_jpg_buf) { free(_jpg_buf); _jpg_buf = NULL; }
    if (res != ESP_OK) { log_e("Send frame failed"); break; }
    int64_t fr_end    = esp_timer_get_time();
    int64_t frame_time = (fr_end - last_frame) / 1000;
    last_frame = fr_end;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    uint32_t avg = ra_filter_run(&ra_filter, frame_time);
    log_i("MJPG: %uB %ums (%.1ffps) AVG: %ums (%.1ffps)",
          (uint32_t)_jpg_buf_len, (uint32_t)frame_time, 1000.0f / frame_time,
          avg, 1000.0f / avg);
#endif
  }
#if defined(LED_GPIO_NUM)
  isStreaming = false;
  enable_led(false);
#endif
  return res;
}

static esp_err_t parse_get(httpd_req_t* req, char** obuf) {
  size_t buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    char* buf = (char*)malloc(buf_len);
    if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) { *obuf = buf; return ESP_OK; }
    free(buf);
  }
  httpd_resp_send_404(req);
  return ESP_FAIL;
}

static esp_err_t cmd_handler(httpd_req_t* req) {
  char* buf = NULL; char variable[32]; char value[32];
  if (parse_get(req, &buf) != ESP_OK) return ESP_FAIL;
  if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) != ESP_OK ||
      httpd_query_key_value(buf, "val", value,    sizeof(value))    != ESP_OK) {
    free(buf); httpd_resp_send_404(req); return ESP_FAIL;
  }
  free(buf);
  int val = atoi(value);
  sensor_t* s = esp_camera_sensor_get();
  int res = 0;
  if      (!strcmp(variable,"framesize"))     { if(s->pixformat==PIXFORMAT_JPEG) res=s->set_framesize(s,(framesize_t)val); }
  else if (!strcmp(variable,"quality"))        res=s->set_quality(s,val);
  else if (!strcmp(variable,"contrast"))       res=s->set_contrast(s,val);
  else if (!strcmp(variable,"brightness"))     res=s->set_brightness(s,val);
  else if (!strcmp(variable,"saturation"))     res=s->set_saturation(s,val);
  else if (!strcmp(variable,"gainceiling"))    res=s->set_gainceiling(s,(gainceiling_t)val);
  else if (!strcmp(variable,"colorbar"))       res=s->set_colorbar(s,val);
  else if (!strcmp(variable,"awb"))            res=s->set_whitebal(s,val);
  else if (!strcmp(variable,"agc"))            res=s->set_gain_ctrl(s,val);
  else if (!strcmp(variable,"aec"))            res=s->set_exposure_ctrl(s,val);
  else if (!strcmp(variable,"hmirror"))        res=s->set_hmirror(s,val);
  else if (!strcmp(variable,"vflip"))          res=s->set_vflip(s,val);
  else if (!strcmp(variable,"awb_gain"))       res=s->set_awb_gain(s,val);
  else if (!strcmp(variable,"agc_gain"))       res=s->set_agc_gain(s,val);
  else if (!strcmp(variable,"aec_value"))      res=s->set_aec_value(s,val);
  else if (!strcmp(variable,"aec2"))           res=s->set_aec2(s,val);
  else if (!strcmp(variable,"dcw"))            res=s->set_dcw(s,val);
  else if (!strcmp(variable,"bpc"))            res=s->set_bpc(s,val);
  else if (!strcmp(variable,"wpc"))            res=s->set_wpc(s,val);
  else if (!strcmp(variable,"raw_gma"))        res=s->set_raw_gma(s,val);
  else if (!strcmp(variable,"lenc"))           res=s->set_lenc(s,val);
  else if (!strcmp(variable,"special_effect")) res=s->set_special_effect(s,val);
  else if (!strcmp(variable,"wb_mode"))        res=s->set_wb_mode(s,val);
  else if (!strcmp(variable,"ae_level"))       res=s->set_ae_level(s,val);
#if defined(LED_GPIO_NUM)
  else if (!strcmp(variable,"led_intensity"))  { led_duty=val; if(isStreaming) enable_led(true); }
#endif
  else { log_i("Unknown: %s", variable); res=-1; }
  if (res < 0) return httpd_resp_send_500(req);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static int print_reg(char* p, sensor_t* s, uint16_t reg, uint32_t mask) {
  return sprintf(p, "\"0x%x\":%u,", reg, s->get_reg(s, reg, mask));
}

static esp_err_t status_handler(httpd_req_t* req) {
  static char json_response[1024];
  sensor_t* s = esp_camera_sensor_get();
  char* p = json_response;
  *p++ = '{';
  if (s->id.PID == OV5640_PID || s->id.PID == OV3660_PID) {
    for (int reg=0x3400;reg<0x3406;reg+=2) p+=print_reg(p,s,reg,0xFFF);
    p+=print_reg(p,s,0x3406,0xFF); p+=print_reg(p,s,0x3500,0xFFFF0);
    p+=print_reg(p,s,0x3503,0xFF); p+=print_reg(p,s,0x350a,0x3FF);
    p+=print_reg(p,s,0x350c,0xFFFF);
    for (int reg=0x5480;reg<=0x5490;reg++) p+=print_reg(p,s,reg,0xFF);
    for (int reg=0x5380;reg<=0x538b;reg++) p+=print_reg(p,s,reg,0xFF);
    for (int reg=0x5580;reg<0x558a;reg++)  p+=print_reg(p,s,reg,0xFF);
    p+=print_reg(p,s,0x558a,0x1FF);
  } else if (s->id.PID == OV2640_PID) {
    p+=print_reg(p,s,0xd3,0xFF); p+=print_reg(p,s,0x111,0xFF); p+=print_reg(p,s,0x132,0xFF);
  }
  p+=sprintf(p,"\"xclk\":%u,",          s->xclk_freq_hz/1000000);
  p+=sprintf(p,"\"pixformat\":%u,",     s->pixformat);
  p+=sprintf(p,"\"framesize\":%u,",     s->status.framesize);
  p+=sprintf(p,"\"quality\":%u,",       s->status.quality);
  p+=sprintf(p,"\"brightness\":%d,",    s->status.brightness);
  p+=sprintf(p,"\"contrast\":%d,",      s->status.contrast);
  p+=sprintf(p,"\"saturation\":%d,",    s->status.saturation);
  p+=sprintf(p,"\"sharpness\":%d,",     s->status.sharpness);
  p+=sprintf(p,"\"special_effect\":%u,",s->status.special_effect);
  p+=sprintf(p,"\"wb_mode\":%u,",       s->status.wb_mode);
  p+=sprintf(p,"\"awb\":%u,",           s->status.awb);
  p+=sprintf(p,"\"awb_gain\":%u,",      s->status.awb_gain);
  p+=sprintf(p,"\"aec\":%u,",           s->status.aec);
  p+=sprintf(p,"\"aec2\":%u,",          s->status.aec2);
  p+=sprintf(p,"\"ae_level\":%d,",      s->status.ae_level);
  p+=sprintf(p,"\"aec_value\":%u,",     s->status.aec_value);
  p+=sprintf(p,"\"agc\":%u,",           s->status.agc);
  p+=sprintf(p,"\"agc_gain\":%u,",      s->status.agc_gain);
  p+=sprintf(p,"\"gainceiling\":%u,",   s->status.gainceiling);
  p+=sprintf(p,"\"bpc\":%u,",           s->status.bpc);
  p+=sprintf(p,"\"wpc\":%u,",           s->status.wpc);
  p+=sprintf(p,"\"raw_gma\":%u,",       s->status.raw_gma);
  p+=sprintf(p,"\"lenc\":%u,",          s->status.lenc);
  p+=sprintf(p,"\"hmirror\":%u,",       s->status.hmirror);
  p+=sprintf(p,"\"vflip\":%u,",         s->status.vflip);
  p+=sprintf(p,"\"dcw\":%u,",           s->status.dcw);
  p+=sprintf(p,"\"colorbar\":%u",       s->status.colorbar);
#if defined(LED_GPIO_NUM)
  p+=sprintf(p,",\"led_intensity\":%u", led_duty);
#else
  p+=sprintf(p,",\"led_intensity\":%d", -1);
#endif
  *p++='}'; *p++=0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t xclk_handler(httpd_req_t* req) {
  char* buf=NULL; char _xclk[32];
  if (parse_get(req,&buf)!=ESP_OK) return ESP_FAIL;
  if (httpd_query_key_value(buf,"xclk",_xclk,sizeof(_xclk))!=ESP_OK) { free(buf); httpd_resp_send_404(req); return ESP_FAIL; }
  free(buf);
  sensor_t* s=esp_camera_sensor_get();
  int res=s->set_xclk(s,LEDC_TIMER_0,atoi(_xclk));
  if (res) return httpd_resp_send_500(req);
  httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
  return httpd_resp_send(req,NULL,0);
}

static esp_err_t reg_handler(httpd_req_t* req) {
  char* buf=NULL; char _reg[32],_mask[32],_val[32];
  if (parse_get(req,&buf)!=ESP_OK) return ESP_FAIL;
  if (httpd_query_key_value(buf,"reg",_reg,sizeof(_reg))!=ESP_OK ||
      httpd_query_key_value(buf,"mask",_mask,sizeof(_mask))!=ESP_OK ||
      httpd_query_key_value(buf,"val",_val,sizeof(_val))!=ESP_OK) { free(buf); httpd_resp_send_404(req); return ESP_FAIL; }
  free(buf);
  sensor_t* s=esp_camera_sensor_get();
  int res=s->set_reg(s,atoi(_reg),atoi(_mask),atoi(_val));
  if (res) return httpd_resp_send_500(req);
  httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
  return httpd_resp_send(req,NULL,0);
}

static esp_err_t greg_handler(httpd_req_t* req) {
  char* buf=NULL; char _reg[32],_mask[32];
  if (parse_get(req,&buf)!=ESP_OK) return ESP_FAIL;
  if (httpd_query_key_value(buf,"reg",_reg,sizeof(_reg))!=ESP_OK ||
      httpd_query_key_value(buf,"mask",_mask,sizeof(_mask))!=ESP_OK) { free(buf); httpd_resp_send_404(req); return ESP_FAIL; }
  free(buf);
  sensor_t* s=esp_camera_sensor_get();
  int res=s->get_reg(s,atoi(_reg),atoi(_mask));
  if (res<0) return httpd_resp_send_500(req);
  char buf2[20]; const char* val=itoa(res,buf2,10);
  httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
  return httpd_resp_send(req,val,strlen(val));
}

static int parse_get_var(char* buf, const char* key, int def) {
  char _int[16];
  if (httpd_query_key_value(buf,key,_int,sizeof(_int))!=ESP_OK) return def;
  return atoi(_int);
}

static esp_err_t pll_handler(httpd_req_t* req) {
  char* buf=NULL;
  if (parse_get(req,&buf)!=ESP_OK) return ESP_FAIL;
  int bypass=parse_get_var(buf,"bypass",0),mul=parse_get_var(buf,"mul",0),sys=parse_get_var(buf,"sys",0);
  int root=parse_get_var(buf,"root",0),pre=parse_get_var(buf,"pre",0),seld5=parse_get_var(buf,"seld5",0);
  int pclken=parse_get_var(buf,"pclken",0),pclk=parse_get_var(buf,"pclk",0);
  free(buf);
  sensor_t* s=esp_camera_sensor_get();
  int res=s->set_pll(s,bypass,mul,sys,root,pre,seld5,pclken,pclk);
  if (res) return httpd_resp_send_500(req);
  httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
  return httpd_resp_send(req,NULL,0);
}

static esp_err_t win_handler(httpd_req_t* req) {
  char* buf=NULL;
  if (parse_get(req,&buf)!=ESP_OK) return ESP_FAIL;
  int sx=parse_get_var(buf,"sx",0),sy=parse_get_var(buf,"sy",0),ex=parse_get_var(buf,"ex",0),ey=parse_get_var(buf,"ey",0);
  int offx=parse_get_var(buf,"offx",0),offy=parse_get_var(buf,"offy",0),tx=parse_get_var(buf,"tx",0),ty=parse_get_var(buf,"ty",0);
  int ox=parse_get_var(buf,"ox",0),oy=parse_get_var(buf,"oy",0);
  bool scale=parse_get_var(buf,"scale",0)==1, binning=parse_get_var(buf,"binning",0)==1;
  free(buf);
  sensor_t* s=esp_camera_sensor_get();
  int res=s->set_res_raw(s,sx,sy,ex,ey,offx,offy,tx,ty,ox,oy,scale,binning);
  if (res) return httpd_resp_send_500(req);
  httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
  return httpd_resp_send(req,NULL,0);
}

static esp_err_t index_handler(httpd_req_t* req) {
  httpd_resp_set_type(req,"text/html");
  httpd_resp_set_hdr(req,"Content-Encoding","gzip");
  sensor_t* s=esp_camera_sensor_get();
  if (s) {
    if      (s->id.PID==OV3660_PID) return httpd_resp_send(req,(const char*)index_ov3660_html_gz,index_ov3660_html_gz_len);
    else if (s->id.PID==OV5640_PID) return httpd_resp_send(req,(const char*)index_ov5640_html_gz,index_ov5640_html_gz_len);
    else                             return httpd_resp_send(req,(const char*)index_ov2640_html_gz,index_ov2640_html_gz_len);
  }
  log_e("Camera sensor not found");
  return httpd_resp_send_500(req);
}

void startCameraServer() {
  httpd_config_t config   = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 16;

  httpd_uri_t index_uri         = {"/",          HTTP_GET, index_handler,         NULL};
  httpd_uri_t status_uri        = {"/status",    HTTP_GET, status_handler,        NULL};
  httpd_uri_t cmd_uri           = {"/control",   HTTP_GET, cmd_handler,           NULL};
  httpd_uri_t dashboard_cmd_uri = {"/cmd",       HTTP_GET, dashboard_cmd_handler, NULL};
  httpd_uri_t capture_uri       = {"/capture",   HTTP_GET, capture_handler,       NULL};
  httpd_uri_t stream_uri        = {"/stream",    HTTP_GET, stream_handler,        NULL};
  httpd_uri_t bmp_uri           = {"/bmp",       HTTP_GET, bmp_handler,           NULL};
  httpd_uri_t xclk_uri          = {"/xclk",      HTTP_GET, xclk_handler,          NULL};
  httpd_uri_t reg_uri           = {"/reg",       HTTP_GET, reg_handler,           NULL};
  httpd_uri_t greg_uri          = {"/greg",      HTTP_GET, greg_handler,          NULL};
  httpd_uri_t pll_uri           = {"/pll",       HTTP_GET, pll_handler,           NULL};
  httpd_uri_t win_uri           = {"/resolution",HTTP_GET, win_handler,           NULL};

  ra_filter_init(&ra_filter, 20);
  log_i("Starting web server on port: '%d'", config.server_port);
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &dashboard_cmd_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &bmp_uri);
    httpd_register_uri_handler(camera_httpd, &xclk_uri);
    httpd_register_uri_handler(camera_httpd, &reg_uri);
    httpd_register_uri_handler(camera_httpd, &greg_uri);
    httpd_register_uri_handler(camera_httpd, &pll_uri);
    httpd_register_uri_handler(camera_httpd, &win_uri);
  }
  config.server_port += 1;
  config.ctrl_port   += 1;
  log_i("Starting stream server on port: '%d'", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setupLedFlash() {
#if defined(LED_GPIO_NUM)
  ledcAttach(LED_GPIO_NUM, LED_FREQ, LED_RESOLUTION);
  ledcWrite(LED_GPIO_NUM, 0);
  log_i("LED flash on GPIO %d", LED_GPIO_NUM);
#else
  log_i("LED flash disabled");
#endif
}
