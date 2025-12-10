/*
  08_Beep_Button_GPIO2_final.ino -> play WAV from SD instead of generating beep

  Changes:
  - On button press, open /test.wav from SD_MMC (SD card) and stream PCM to audio_playback_write.
  - Parses WAV header and locates "data" chunk, supports 16-bit PCM mono/stereo.
  - Streams in chunks to avoid large memory use.
  - Keep board power / audio init and UART restore logic from original.
  - Requires /test.wav on SD card; file should be 16-bit PCM, ideally SAMPLE_RATE_HZ (16k).
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

// SD card
#include "SD_MMC.h"
#include "FS.h"
// If your board needs the SPI SD driver instead, you can include <SD.h> and call SD.begin(CS_PIN).
// #include <SD.h>

// --- Playback configuration ---
const int SAMPLE_RATE_HZ    = 16000; // target sample rate for audio subsystem
const int VOLUME            = 80;
const int BUTTON_PIN = 2; // your chosen GPIO (exposed on Waveshare connector)

// Streaming chunk: bytes of output PCM (16-bit stereo), adjust if needed.
// Keep this reasonably small to limit RAM usage. Must be a multiple of 4 (2 bytes per sample * 2 channels).
const size_t OUT_CHUNK_BYTES = 16384;

// FreeRTOS objects
static SemaphoreHandle_t beep_semaphore = NULL;
TaskHandle_t play_beep_task_handle = NULL;

// Temporary buffer for streaming (allocated at runtime)
static uint8_t *stream_buf = NULL;

/**
 * Minimal ISR: give semaphore to wake the playback task.
 * Keep in IRAM and use ISR-safe API.
 */
void IRAM_ATTR button_isr()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (beep_semaphore) {
    xSemaphoreGiveFromISR(beep_semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
  }
}

/**
 * Helper: read little-endian 16/32 values from buffer
 */
static uint32_t le32(const uint8_t *p) {
  return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t le16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/**
 * Play WAV file from SD (SD_MMC preferred). Path example: "/test.wav"
 * Requirements: 16-bit PCM WAV. If sample rate != SAMPLE_RATE_HZ the code will warn but still stream (no resampling).
 * Supports mono (duplicates samples to stereo) and stereo 16-bit.
 */
static void play_wav_from_sd(const char *path)
{
  if (!stream_buf) {
    Serial.println("play_wav_from_sd: stream_buf not allocated");
    return;
  }

  Serial.printf("Opening WAV: %s\n", path);
  File f;
  if (SD_MMC.begin()) {
    f = SD_MMC.open(path);
  } else {
    Serial.println("Warning: SD_MMC.begin() failed. Please ensure SD is initialized or modify code for SD.begin(CS_PIN).");
  }

  if (!f || !f.available()) {
    Serial.println("Failed to open WAV file on SD_MMC. Trying SD (if enabled)...");
    // If you want to support classical SPI SD, add fallback here by uncommenting and setting CS pin.
    // if (SD.begin(CS_PIN)) {
    //   f = SD.open(path);
    // }
  }

  if (!f || !f.available()) {
    Serial.println("ERROR: could not open WAV file. Make sure /test.wav exists on SD card.");
    return;
  }

  // Parse WAV chunks: find "fmt " and "data"
  // Read RIFF header first 12 bytes
  uint8_t hdr12[12];
  if (f.read(hdr12, 12) != 12) {
    Serial.println("ERROR: failed to read WAV header.");
    f.close();
    return;
  }

  if (memcmp(hdr12, "RIFF", 4) != 0 || memcmp(hdr12 + 8, "WAVE", 4) != 0) {
    Serial.println("ERROR: not a RIFF/WAVE file.");
    f.close();
    return;
  }

  // Walk chunks to find "fmt " and "data"
  uint16_t audio_format = 0;
  uint16_t num_channels = 0;
  uint32_t sample_rate = 0;
  uint16_t bits_per_sample = 0;
  uint32_t data_chunk_pos = 0;
  uint32_t data_chunk_size = 0;

  while (f.position() < f.size()) {
    uint8_t chunk_hdr[8];
    if (f.read(chunk_hdr, 8) != 8) break;
    uint32_t chunk_id = le32(chunk_hdr);
    uint32_t chunk_size = le32(chunk_hdr + 4);
    // "fmt " chunk
    if (memcmp(chunk_hdr, "fmt ", 4) == 0) {
      // Read fmt data (ensure it fits)
      uint8_t *fmt = (uint8_t *)malloc(chunk_size);
      if (!fmt) {
        Serial.println("ERROR: out of memory parsing fmt chunk");
        f.close();
        return;
      }
      if (f.read(fmt, chunk_size) != (int)chunk_size) {
        Serial.println("ERROR: failed reading fmt chunk");
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
      data_chunk_pos = f.position();
      data_chunk_size = chunk_size;
      // Seek past data chunk for now; we'll reposition later to data_chunk_pos
      f.seek(f.position() + chunk_size);
    } else {
      // skip unknown chunk (plus pad if needed)
      f.seek(f.position() + chunk_size);
    }
    // handle odd chunk sizes: chunks are word-aligned, but Arduino File read/seek handles it sufficiently.
  }

  if (data_chunk_pos == 0 || data_chunk_size == 0) {
    Serial.println("ERROR: 'data' chunk not found in WAV.");
    f.close();
    return;
  }

  Serial.printf("WAV fmt: audio_format=%u channels=%u sample_rate=%u bits=%u data_bytes=%u\n",
                audio_format, (unsigned)num_channels, (unsigned)sample_rate, (unsigned)bits_per_sample, (unsigned)data_chunk_size);

  if (audio_format != 1) {
    Serial.println("ERROR: WAV is not PCM (unsupported compressed format).");
    f.close();
    return;
  }
  if (bits_per_sample != 16) {
    Serial.println("ERROR: WAV is not 16-bit (unsupported).");
    f.close();
    return;
  }
  if (sample_rate != (uint32_t)SAMPLE_RATE_HZ) {
    Serial.println("Warning: WAV sample rate does not match SAMPLE_RATE_HZ. No resampling performed.");
  }
  if (num_channels != 1 && num_channels != 2) {
    Serial.println("ERROR: WAV must be mono or stereo.");
    f.close();
    return;
  }

  // Reposition to data chunk
  f.seek(data_chunk_pos);

  // Prepare output buffer pointers
  const size_t in_frame_bytes = (size_t)num_channels * 2; // bytes per input frame
  // We'll use stream_buf as output; ensure it's large enough
  // For mono -> stereo doubling, max output bytes = (OUT_CHUNK_BYTES) ; we will read half as many input bytes
  uint8_t *outbuf = stream_buf;
  size_t outbuf_size = OUT_CHUNK_BYTES;

  audio_playback_set_vol(VOLUME);

  uint32_t bytes_remaining = data_chunk_size;
  while (bytes_remaining > 0) {
    // How many output bytes can we send this iteration
    size_t want_out = outbuf_size;
    // Convert to how many input bytes to read based on channels
    size_t in_bytes_to_read;
    if (num_channels == 2) {
      // stereo in -> pass through: in_bytes == out_bytes
      in_bytes_to_read = (want_out > bytes_remaining) ? bytes_remaining : want_out;
    } else { // mono
      // Mono input will become stereo output; for N input bytes, output bytes = N * 2
      size_t max_in_by_remain = bytes_remaining;
      size_t max_in_by_out = want_out / 2;
      in_bytes_to_read = (max_in_by_remain < max_in_by_out) ? max_in_by_remain : max_in_by_out;
    }
    if (in_bytes_to_read == 0) break;

    // Read input
    // We'll read into a temporary input buffer if mono; but to avoid extra allocs, reuse a portion of stream_buf:
    uint8_t *inbuf = stream_buf; // reuse same buffer region
    size_t read_len = f.read(inbuf, in_bytes_to_read);
    if (read_len == 0) break;
    bytes_remaining -= read_len;

    size_t out_len = 0;
    if (num_channels == 2) {
      // direct passthrough; inbuf already stereo 16-bit
      out_len = read_len;
      // write directly from inbuf
      audio_playback_write(inbuf, out_len);
    } else {
      // mono: need to expand each 16-bit sample to stereo (L=R)
      // We'll expand in-place into a second buffer region if possible. Since we used stream_buf size OUT_CHUNK_BYTES and read_len <= OUT_CHUNK_BYTES/2,
      // we can expand into stream_buf (we'll write into a temp out region).
      // Create a simple expansion into a heap-allocated buffer if necessary.
      size_t samples = read_len / 2; // 2 bytes per sample
      // Expand into a temporary buffer allocated once (on heap) sized samples*4 bytes
      // To avoid malloc/free each loop, allocate a static buffer on heap sized OUT_CHUNK_BYTES.
      static uint8_t *mono_outbuf = NULL;
      if (mono_outbuf == NULL) {
        mono_outbuf = (uint8_t *)heap_caps_malloc(OUT_CHUNK_BYTES, MALLOC_CAP_8BIT);
        if (!mono_outbuf) {
          Serial.println("ERROR: failed to allocate mono_outbuf.");
          break;
        }
      }
      uint8_t *dst = mono_outbuf;
      uint8_t *src = inbuf;
      for (size_t s = 0; s < samples; ++s) {
        // copy 2 bytes to left
        *dst++ = *src;
        *dst++ = *(src + 1);
        // copy same 2 bytes to right
        *dst++ = *src;
        *dst++ = *(src + 1);
        src += 2;
      }
      out_len = samples * 4;
      audio_playback_write(mono_outbuf, out_len);
    }

    // optional small delay to allow other tasks; audio_playback_write might block as needed
    // vTaskDelay(pdMS_TO_TICKS(0));
  }

  Serial.println("Finished playing WAV.");
  f.close();
}

/**
 * Task: wait for semaphore, debounce in task context, verify press, play WAV file.
 */
void play_beep_task(void *pvParameters)
{
  (void)pvParameters;
  printf("play_beep_task started, waiting for button presses...\n");

  for (;;) {
    if (xSemaphoreTake(beep_semaphore, portMAX_DELAY) == pdTRUE) {
      // debounce by delaying briefly in task context
      vTaskDelay(pdMS_TO_TICKS(30));

      // confirm active-low press (button -> GND)
      int lvl = digitalRead(BUTTON_PIN);
      printf("play_beep_task: button level read = %d\n", lvl);
      if (lvl == LOW) {
        Serial.println("Button pressed — playing /test.wav from SD card");
        play_wav_from_sd("/test.wav");
        Serial.println("Playback finished.");
      } else {
        printf("play_beep_task: press transient / bounce (ignored)\n");
      }

      // avoid repeated triggers while button is held
      vTaskDelay(pdMS_TO_TICKS(150));
    }
  }
  vTaskDelete(NULL);
}

void setup()
{
  // Start Serial early (may be remapped by audio init below)
  Serial.begin(115200);
  delay(50);
  printf("\n=== 08_Play_WAV_Button_GPIO2 setup ===\n");
  Serial.println("Serial started (attempt)");

  // Allocate streaming buffer
  stream_buf = (uint8_t *)heap_caps_malloc(OUT_CHUNK_BYTES, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if (!stream_buf) {
    stream_buf = (uint8_t *)malloc(OUT_CHUNK_BYTES);
  }
  if (!stream_buf) {
    printf("ERROR: failed to allocate stream_buf (%u bytes)\n", (unsigned)OUT_CHUNK_BYTES);
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // Enable audio power on the board BEFORE initializing codec/audio
  board_power_bsp_t board_div(EPD_PWR_PIN, Audio_PWR_PIN, VBAT_PWR_PIN);
  board_div.POWEER_Audio_ON();
  vTaskDelay(pdMS_TO_TICKS(50)); // let rails settle

  // Initialize audio subsystem (this may remap GPIOs)
  audio_bsp_init();
  audio_play_init();

  // Best-effort: restore UART0 to default TX/RX pins and reinit Serial
  // This helps Serial.println to continue working after codec init.
  uart_set_pin(UART_NUM_0, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  Serial.end();
  delay(50);
  Serial.begin(115200);
  delay(50);
  Serial.println("Serial reinitialized after audio init");

  // Initialize SD_MMC
  if (!SD_MMC.begin()) {
    Serial.println("Warning: SD_MMC.begin() failed. If your board uses SPI SD, uncomment and configure SD.begin(CS_PIN) fallback in the code.");
  } else {
    Serial.println("SD_MMC initialized. Looking for /test.wav on SD card root.");
  }

  // Create binary semaphore
  beep_semaphore = xSemaphoreCreateBinary();
  if (beep_semaphore == NULL) {
    Serial.println("ERROR: failed to create beep_semaphore");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // Configure button pin and attach ISR: button -> GND so use INPUT_PULLUP + FALLING
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_isr, FALLING);

  // Create the playback task (task will call audio_playback_write)
  xTaskCreatePinnedToCore(play_beep_task, "PlayBeep", 8 * 1024, NULL, 4, &play_beep_task_handle, 1);

  Serial.println("Setup complete — press button on GPIO2 to play /test.wav");
}

void loop()
{
  // Keep loop minimal; add a heartbeat so you know stdout is working
  static uint32_t last = 0;
  if (millis() - last > 3000) {
    last = millis();
    printf("Heartbeat: millis=%u\n", (unsigned)last);
  }
  delay(100);
}