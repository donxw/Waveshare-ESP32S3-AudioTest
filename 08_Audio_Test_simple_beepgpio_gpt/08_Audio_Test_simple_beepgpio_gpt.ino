/*
  08_Beep_Button_GPIO2_final.ino

  Cleaned final sketch:
  - Uses ISR + binary semaphore to trigger a 1kHz, 200ms beep on a button press (GPIO2).
  - Ensures board audio power is enabled before initializing codec/audio.
  - Restores UART0 pin mapping and reinitializes Arduino Serial after audio init
    so Serial.println() continues to work.
  - Allocates PCM buffer at runtime (heap) to avoid C file-scope VLA issues.
  - Uses short debounce in task context; ISR is kept minimal.

  Wiring:
  - Momentary button between GPIO2 and GND (active LOW). Do NOT hold button
    at reset/power-on (can affect boot mode).
  - Speaker/amplifier should be connected as on the Waveshare board.

  Notes:
  - If Serial still doesn't appear reliably, printf() remains a fallback.
  - If you prefer a different button pin, change BUTTON_PIN accordingly.
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

// --- Beep / button configuration ---
const int BEEP_FREQUENCY_HZ = 1000;
const int SAMPLE_RATE_HZ    = 16000;
const int DURATION_MS       = 200;
const int VOLUME            = 80;

const int BUTTON_PIN = 2; // your chosen GPIO (exposed on Waveshare connector)

// Derived sizes
const size_t NUM_SAMPLES = (SAMPLE_RATE_HZ / 1000) * DURATION_MS;
const size_t PCM_BUFFER_SIZE = NUM_SAMPLES * 4; // 16-bit stereo (L,R)

// PCM buffer pointer (allocated at runtime)
static uint8_t *pcm_data = NULL;

// FreeRTOS objects
static SemaphoreHandle_t beep_semaphore = NULL;
TaskHandle_t play_beep_task_handle = NULL;

/**
 * Generate a 1kHz sine wave, 16-bit stereo interleaved.
 */
static void generate_beep_pcm()
{
  if (pcm_data == NULL) return;
  int16_t *samples = (int16_t *)pcm_data;
  double amplitude = 30000.0 * (VOLUME / 100.0); // keep under 32767
  double step = (2.0 * M_PI * BEEP_FREQUENCY_HZ) / (double)SAMPLE_RATE_HZ;

  for (size_t i = 0; i < NUM_SAMPLES; ++i) {
    int16_t value = (int16_t)(amplitude * sin((double)i * step));
    *samples++ = value; // left
    *samples++ = value; // right
  }
}

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
 * Task: wait for semaphore, debounce in task context, verify press, play beep.
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
        Serial.println("Button pressed — playing beep");
        audio_playback_set_vol(VOLUME);
        audio_playback_write(pcm_data, PCM_BUFFER_SIZE);
        Serial.println("Beep finished.");
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
  printf("\n=== 08_Beep_Button_GPIO2_final setup ===\n");
  Serial.println("Serial started (attempt)");

  // Allocate PCM buffer: prefer PSRAM if available, fallback to malloc
  pcm_data = (uint8_t *)heap_caps_malloc(PCM_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!pcm_data) pcm_data = (uint8_t *)malloc(PCM_BUFFER_SIZE);
  if (!pcm_data) {
    printf("ERROR: failed to allocate pcm_data (%u bytes)\n", (unsigned)PCM_BUFFER_SIZE);
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

  // Generate beep samples
  generate_beep_pcm();

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
  xTaskCreatePinnedToCore(play_beep_task, "PlayBeep", 4 * 1024, NULL, 4, &play_beep_task_handle, 1);

  Serial.println("Setup complete — press button on GPIO2 to play beep");
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