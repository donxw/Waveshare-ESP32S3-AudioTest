/*
  Play /sdcard/test.wav from SD card on button press (GPIO2)

  - Uses board/APIs in this repo to mount the SD card if available:
    attempts mount_sdcard(), then sdcard_init() (from the 04_SD_Card sample),
    and finally falls back to SD_MMC.begin() if necessary.
  - Parses simple 16-bit PCM WAV files and streams to audio_playback_write.
  - Supports 16-bit PCM mono or stereo (mono is duplicated to stereo on the fly).
  - Keeps original audio init, UART restore, and button ISR/task pattern.
  - Place a 16-bit PCM WAV at /sdcard/test.wav (the repo's SD examples use /sdcard path).
*/

#include <Arduino.h>
#include <math.h>

#include "audio_bsp.h"
#include "user_config.h"
#include "src/power/board_power_bsp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/uart.h"  // uart_set_pin
#include "driver/gpio.h"  // GPIO_NUM_1 / GPIO_NUM_3 (if used by other code)
#include "esp_heap_caps.h" // heap_caps_malloc

// SD APIs / sample SD files in repo
#include <SD_MMC.h>           // primary SD_MMC access
#include "src/codec_board/codec_init.h" // mount_sdcard() declaration (if available)
#include "sdcard_bsp.h"      // sdcard_init() from the sample sketch (if available)

// --- Playback / button configuration ---
const int SAMPLE_RATE_HZ    = 16000; // codec audio sample rate used by board
const int VOLUME            = 80;
const int BUTTON_PIN = 2; // active-low button to GND (Waveshare connector)

// Streaming chunk: bytes of output PCM (16-bit stereo), must be multiple of 4
const size_t OUT_CHUNK_BYTES = 16 * 1024;

// FreeRTOS objects
static SemaphoreHandle_t beep_semaphore = NULL;
TaskHandle_t play_task_handle = NULL;

// Buffers
static uint8_t *stream_buf = NULL;

/* Helper: little-endian reads */
static uint32_t le32(const uint8_t *p) {
  return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t le16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/* Minimal ISR: give semaphore to wake the playback task */
void IRAM_ATTR button_isr()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (beep_semaphore) {
    xSemaphoreGiveFromISR(beep_semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
  }
}

/* Try to ensure SD card is mounted. We prefer mount_sdcard() (codec_board API),
   then sdcard_init() from the 04_SD_Card sample, then SD_MMC.begin() fallback. */
static bool ensure_sd_mounted()
{
  // Try mount_sdcard() if available
  #if defined(__has_include)
    #if __has_include("src/codec_board/codec_init.h")
      // mount_sdcard() declared in codec_init.h; call it
      if (mount_sdcard() == 0) {
        Serial.println("Mounted SD via mount_sdcard()");
        return true;
      } else {
        Serial.println("mount_sdcard() failed or not configured for this board");
      }
    #endif
  #endif

  // Try sdcard_init() from sample (04_SD_Card)
  #if defined(__has_include)
    #if __has_include("04_SD_Card/sdcard_bsp.h")
      // sdcard_init() used by sample 04_SD_Card.ino
      sdcard_init();
      // sdcard_init() in sample typically mounts under /sdcard using SD_MMC; we check SD_MMC
      if (SD_MMC.begin()) {
        Serial.println("SD_MMC began after sdcard_init()");
        return true;
      } else {
        Serial.println("sdcard_init() called but SD_MMC.begin() still failed");
      }
    #endif
  #endif

  // Fallback: try SD_MMC directly
  if (SD_MMC.begin()) {
    Serial.println("SD_MMC.begin() succeeded (fallback)");
    return true;
  }
  Serial.println("SD mount failed: ensure SD pins / power are configured and card present");
  return false;
}

/* Play 16-bit PCM WAV at /sdcard/test.wav
   Requirements: PCM (audio_format==1), bits_per_sample==16, channels=1 or 2.
   No resampling is performed (sample_rate mismatch will only warn).
*/
static void play_wav_from_sd(const char *path)
{
  if (!stream_buf) {
    Serial.println("stream_buf not allocated");
    return;
  }

  if (!ensure_sd_mounted()) {
    Serial.println("Cannot access SD card");
    return;
  }

  Serial.printf("Opening WAV: %s\n", path);
  File f = SD_MMC.open(path);
  if (!f || !f.available()) {
    Serial.println("Failed to open WAV file on SD_MMC. Make sure the file exists at /sdcard/test.wav");
    if (f) f.close();
    return;
  }

  // Read RIFF header
  uint8_t hdr12[12];
  if (f.read(hdr12, 12) != 12) {
    Serial.println("Failed to read WAV header");
    f.close();
    return;
  }
  if (memcmp(hdr12, "RIFF", 4) != 0 || memcmp(hdr12 + 8, "WAVE", 4) != 0) {
    Serial.println("Not a RIFF/WAVE file");
    f.close();
    return;
  }

  // Find fmt and data chunks
  uint16_t audio_format = 0;
  uint16_t num_channels = 0;
  uint32_t sample_rate = 0;
  uint16_t bits_per_sample = 0;
  uint32_t data_pos = 0;
  uint32_t data_size = 0;

  while (f.position() + 8 <= f.size()) {
    uint8_t chunk_hdr[8];
    if (f.read(chunk_hdr, 8) != 8) break;
    uint32_t chunk_size = le32(chunk_hdr + 4);
    if (memcmp(chunk_hdr, "fmt ", 4) == 0) {
      // read fmt chunk
      uint8_t *fmt = (uint8_t *)malloc(chunk_size);
      if (!fmt) {
        Serial.println("Out of memory reading fmt chunk");
        f.close();
        return;
      }
      if (f.read(fmt, chunk_size) != (int)chunk_size) {
        Serial.println("Failed to read fmt chunk");
        free(fmt);
        f.close();
        return;
      }
      audio_format = le16(fmt + 0);
      num_channels = le16(fmt + 2);
      sample_rate = le32(fmt + 4);
      bits_per_sample = le16(fmt + 14);
      free(fmt);
    } else if (memcmp(chunk_hdr, "data", 4) == 0) {
      data_pos = f.position();
      data_size = chunk_size;
      // seek past data chunk for now
      f.seek(f.position() + chunk_size);
    } else {
      // skip other chunk
      f.seek(f.position() + chunk_size);
    }
    // continue loop
  }

  if (data_pos == 0 || data_size == 0) {
    Serial.println("No data chunk found");
    f.close();
    return;
  }

  Serial.printf("WAV fmt: audio_format=%u channels=%u sample_rate=%u bits=%u data_bytes=%u\n",
                audio_format, (unsigned)num_channels, (unsigned)sample_rate, (unsigned)bits_per_sample, (unsigned)data_size);

  if (audio_format != 1) {
    Serial.println("WAV not PCM (unsupported)");
    f.close();
    return;
  }
  if (bits_per_sample != 16) {
    Serial.println("WAV not 16-bit (unsupported)");
    f.close();
    return;
  }
  if (num_channels != 1 && num_channels != 2) {
    Serial.println("WAV must be mono or stereo");
    f.close();
    return;
  }
  if (sample_rate != (uint32_t)SAMPLE_RATE_HZ) {
    Serial.println("Warning: WAV sample rate differs from codec SAMPLE_RATE_HZ (no resampling)");
  }

  // Seek to data start and stream
  f.seek(data_pos);
  audio_playback_set_vol(VOLUME);

  // For mono expansion we'll allocate a mono_outbuf once
  static uint8_t *mono_outbuf = NULL;
  if (!mono_outbuf) {
    mono_outbuf = (uint8_t *)heap_caps_malloc(OUT_CHUNK_BYTES, MALLOC_CAP_8BIT);
    if (!mono_outbuf) mono_outbuf = (uint8_t *)malloc(OUT_CHUNK_BYTES);
    if (!mono_outbuf) {
      Serial.println("Failed to allocate mono_outbuf");
      f.close();
      return;
    }
  }

  uint32_t bytes_remaining = data_size;
  while (bytes_remaining > 0) {
    size_t want_out = OUT_CHUNK_BYTES;
    size_t in_bytes_to_read = 0;
    if (num_channels == 2) {
      // stereo passthrough: read up to want_out
      in_bytes_to_read = (bytes_remaining < want_out) ? bytes_remaining : want_out;
    } else {
      // mono -> stereo duplication: for N input bytes output will be 2*N
      size_t max_in_by_out = want_out / 2;
      in_bytes_to_read = (bytes_remaining < max_in_by_out) ? bytes_remaining : max_in_by_out;
    }
    if (in_bytes_to_read == 0) break;

    // read input into stream_buf (reuse)
    size_t read_len = f.read(stream_buf, in_bytes_to_read);
    if (read_len == 0) break;
    bytes_remaining -= read_len;

    if (num_channels == 2) {
      // direct write
      audio_playback_write(stream_buf, read_len);
    } else {
      // expand mono to stereo into mono_outbuf
      size_t samples = read_len / 2;
      uint8_t *src = stream_buf;
      uint8_t *dst = mono_outbuf;
      for (size_t i = 0; i < samples; ++i) {
        // copy sample to left
        *dst++ = *src;
        *dst++ = *(src + 1);
        // copy same sample to right
        *dst++ = *src;
        *dst++ = *(src + 1);
        src += 2;
      }
      audio_playback_write(mono_outbuf, samples * 4);
    }
  }

  Serial.println("Finished playing WAV");
  f.close();
}

/* Playback task: wait for semaphore, debounce, then play WAV */
void play_task(void *pvParameters)
{
  (void)pvParameters;
  printf("play_task started, waiting for button presses...\n");

  for (;;) {
    if (xSemaphoreTake(beep_semaphore, portMAX_DELAY) == pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(30));
      int lvl = digitalRead(BUTTON_PIN);
      printf("play_task: button level read = %d\n", lvl);
      if (lvl == LOW) {
        Serial.println("Button pressed — playing /sdcard/test.wav");
        play_wav_from_sd("/sdcard/test.wav");
        Serial.println("Playback done.");
      } else {
        printf("play_task: transient (ignored)\n");
      }
      vTaskDelay(pdMS_TO_TICKS(150));
    }
  }
  vTaskDelete(NULL);
}

void setup()
{
  Serial.begin(115200);
  delay(50);
  printf("\n=== 08_Play_WAV_Button_GPIO2 setup ===\n");
  Serial.println("Serial started (attempt)");

  // Allocate streaming buffer
  stream_buf = (uint8_t *)heap_caps_malloc(OUT_CHUNK_BYTES, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if (!stream_buf) stream_buf = (uint8_t *)malloc(OUT_CHUNK_BYTES);
  if (!stream_buf) {
    printf("ERROR: failed to allocate stream_buf\n");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // Enable audio power BEFORE codec init
  board_power_bsp_t board_div(EPD_PWR_PIN, Audio_PWR_PIN, VBAT_PWR_PIN);
  board_div.POWEER_Audio_ON();
  vTaskDelay(pdMS_TO_TICKS(50));

  // Init audio
  audio_bsp_init();
  audio_play_init();

  // Restore UART0 pins and reinit Serial if necessary
  uart_set_pin(UART_NUM_0, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  Serial.end();
  delay(50);
  Serial.begin(115200);
  delay(50);
  Serial.println("Serial reinitialized after audio init");

  // (optional) try to mount SD early so file presence can be tested in setup
  ensure_sd_mounted();

  // Create semaphore
  beep_semaphore = xSemaphoreCreateBinary();
  if (beep_semaphore == NULL) {
    Serial.println("ERROR: failed to create semaphore");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // Button config and ISR: button to GND (active LOW)
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_isr, FALLING);

  // Create playback task
  xTaskCreatePinnedToCore(play_task, "PlayWAV", 8 * 1024, NULL, 4, &play_task_handle, 1);

  Serial.println("Setup complete — press button on GPIO2 to play /sdcard/test.wav");
}

void loop()
{
  static uint32_t last = 0;
  if (millis() - last > 3000) {
    last = millis();
    printf("Heartbeat: millis=%u\n", (unsigned)last);
  }
  delay(100);
}