#include "HX711.h"
#include <esp_camera.h>
#include <smart_Scall_inferencing.h>   // Edge Impulse library

// ===== AI THINKER ESP32-CAM PINS =====
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

// ================= HX711 =================
#define DT  2       // GPIO2
#define SCK 15      // GPIO15
HX711 scale;

// ---- CALIBRATION ----
long zeroRaw = -31000;           // empty scale raw value
float scaleFactor = 353379.56;   // raw → kg
float weightTriggerKg = 0.02;    // 20g trigger

bool inferenceDone = false;

// ================= PRICE TABLE =================
const char* items[] = {"onion", "potato"};
float prices[] = {80.0, 50.0};   // price per kg

// ================= CAMERA =================
uint8_t *snapshot_buf = NULL;
static bool camera_ready = false;

camera_config_t config;

// ================= SIMPLIFIED DATA CALLBACK =================
// This is the FIXED version that works with Edge Impulse
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  if (!snapshot_buf) {
    return -1;
  }
  
  // For grayscale image (96x96), just copy the bytes
  // Edge Impulse expects normalized values (0-255 range)
  for (size_t i = 0; i < length; i++) {
    size_t ix = offset + i;
    
    // Check bounds
    if (ix < EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT) {
      // Get pixel value (0-255) and convert to float
      out_ptr[i] = (float)snapshot_buf[ix];
    } else {
      out_ptr[i] = 0.0f;
    }
  }
  
  return 0;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n=========================================");
  Serial.println("       SMART SCALE SYSTEM - FIXED");
  Serial.println("=========================================");
  
  // DISABLE GPIO2 LED
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  
  // Initialize HX711
  Serial.println("\nInitializing HX711 scale...");
  scale.begin(DT, SCK);
  scale.set_scale(scaleFactor);
  
  // Tare the scale
  Serial.println("Taring scale (keep empty)...");
  scale.tare();
  delay(1000);
  
  // Update zero with actual reading
  zeroRaw = scale.read_average(10);
  Serial.printf("Zero raw value: %ld\n", zeroRaw);
  Serial.println("✓ HX711 ready");
  
  // Initialize Camera
  Serial.println("\nInitializing camera...");
  
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;  // Grayscale for Edge Impulse
  config.frame_size = FRAMESIZE_96X96;        // 96x96 pixels
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while(1) delay(1000);
  }
  
  camera_ready = true;
  Serial.println("✓ Camera ready (96x96 grayscale)");
  
  // Print Edge Impulse info
  Serial.println("\n=== EDGE IMPULSE MODEL INFO ===");
  Serial.printf("Input size: %d x %d\n", 
                EI_CLASSIFIER_INPUT_WIDTH, 
                EI_CLASSIFIER_INPUT_HEIGHT);
  Serial.printf("Expected buffer size: %d bytes\n", 
                EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT);
  Serial.printf("Number of labels: %d\n", EI_CLASSIFIER_LABEL_COUNT);
  
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    Serial.printf("  %d: %s - TK%.2f/kg\n", i, items[i], prices[i]);
  }
  
  Serial.println("\n✅ System Ready! Place item on scale.");
  Serial.println("=========================================\n");
}

// ================= INFERENCE FUNCTION =================
bool runInference() {
  Serial.println("\n=== STARTING INFERENCE ===");
  
  // Calculate buffer size (96x96 grayscale = 9216 bytes)
  size_t buf_size = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  snapshot_buf = (uint8_t*)malloc(buf_size);
  
  if (!snapshot_buf) {
    Serial.println("ERROR: Failed to allocate buffer!");
    return false;
  }
  
  Serial.printf("Allocated %d bytes for image buffer\n", buf_size);
  
  // Capture image
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("ERROR: Camera capture failed!");
    free(snapshot_buf);
    return false;
  }
  
  // Check if frame is correct size
  if (fb->len < buf_size) {
    Serial.printf("ERROR: Frame too small! Got %d, need %d\n", fb->len, buf_size);
    esp_camera_fb_return(fb);
    free(snapshot_buf);
    return false;
  }
  
  // Copy image data to buffer
  memcpy(snapshot_buf, fb->buf, buf_size);
  esp_camera_fb_return(fb);
  
  Serial.println("✓ Image captured and copied to buffer");
  
  // Prepare signal for Edge Impulse
  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;
  
  // Run inference
  ei_impulse_result_t result;
  Serial.println("Running Edge Impulse inference...");
  
  EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
  
  // Clean up buffer
  free(snapshot_buf);
  snapshot_buf = NULL;
  
  if (res != EI_IMPULSE_OK) {
    Serial.printf("ERROR: Inference failed with code: %d\n", res);
    return false;
  }
  
  Serial.println("✓ Inference successful!");
  
  // Find best classification
  int bestIndex = 0;
  float bestScore = 0;
  
  Serial.println("\n--- CLASSIFICATION RESULTS ---");
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    float score = result.classification[i].value;
    Serial.printf("  %s: %.3f\n", items[i], score);
    
    if (score > bestScore) {
      bestScore = score;
      bestIndex = i;
    }
  }
  
  // Check confidence threshold
  if (bestScore < 0.70) {
    Serial.printf("Low confidence (%.3f < 0.50) - ignoring\n", bestScore);
    return false;
  }
  
  // Get final accurate weight
  long raw = scale.read_average(10);
  float weightKg = (raw - zeroRaw) / scaleFactor;
  if (weightKg < 0) weightKg = 0;
  
  // Display results
  Serial.println("\n═══════════════════════════════════");
  Serial.print("✅ ITEM: ");
  Serial.println(items[bestIndex]);
  Serial.print("Confidence: ");
  Serial.println(bestScore, 3);
  Serial.print("Weight: ");
  Serial.print(weightKg, 3);
  Serial.println(" kg");
  
  float bill = weightKg * prices[bestIndex];
  Serial.print("💰 BILL: TK");
  Serial.println(bill, 2);
  Serial.println("═══════════════════════════════════\n");
  
  return true;
}

// ================= LOOP =================
void loop() {
  static unsigned long lastWeightTime = 0;
  unsigned long now = millis();
  
  // Read weight every 500ms
  if (now - lastWeightTime >= 500) {
    long raw = scale.read_average(3);  // Quick reading
    float weightKg = (raw - zeroRaw) / scaleFactor;
    
    if (weightKg < 0) weightKg = 0;
    
    Serial.print("Weight: ");
    Serial.print(weightKg, 3);
    Serial.println(" kg");
    
    // Check if we should trigger inference
    if (weightKg >= weightTriggerKg && !inferenceDone) {
      inferenceDone = true;
      Serial.println("\n>>> OBJECT DETECTED! <<<");
      
      // Give weight a moment to stabilize
      delay(200);
      
      // Run inference
      bool success = runInference();
      
      if (!success) {
        Serial.println("Inference failed or low confidence");
      }
    }
    
    // Reset when item is removed
    if (weightKg < 0.01 && inferenceDone) {
      inferenceDone = false;
      Serial.println("\nItem removed - ready for next item");
      Serial.println("-----------------------------------------\n");
    }
    
    lastWeightTime = now;
  }
  
  delay(100);
}
