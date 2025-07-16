#include <driver/i2s.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// OLED  Display
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Visualization 
#define Y_AXIS_SPACE  20
#define GRAPH_WIDTH   108 // 128 - 18
#define GRAPH_HEIGHT  64
#define GRAPH_EMPTY   255 // to init the values

uint8_t graphBufferSPL[GRAPH_WIDTH];
uint8_t graphBufferdBA[GRAPH_WIDTH];

// SD Card
#define SD_ENABLE   9
#define VSPI_MISO   13
#define VSPI_MOSI   11
#define VSPI_SCLK   12
#define VSPI_SS     10

SPIClass sdspi = SPIClass();

// Enable the GPIO sockets
#define IO_ENABLE   8

// DFGravity SLM
// Connected to GPIO C
#define SLM_PIN     A7

// ICS43434 Mic
// Connected to GPIO A
#define I2S_SD      D3 // PIN_IO3 Serial data in from mic BLUE cable
#define I2S_SCK     D2 // PIN_IO2 Serial clock YELLOW cable
// Connected to GPIO B
#define I2S_WS      D5 // PIN_IO5 L/R clock (word select) LONELY BLUE cable

#define I2S_PORT    I2S_NUM_0
#define SAMPLE_RATE 16000
#define BUFFER_SIZE 1024

int32_t sampleBuffer[BUFFER_SIZE];

// Setup ICS43434 Microphone
void setupI2S() {
  Serial.println("Initializing I2S...");
  i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_set_clk(I2S_PORT, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
  Serial.println("I2S initialized!");
}

// Setup OLED Display
void setupOLED() {
  Serial.println("Initializing OLED...");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    while (true) {
      Serial.println("SSD1306 allocation failed");
      delay(60000);
    }
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();

  // Initialize line graph buffer
  memset(graphBufferSPL, GRAPH_EMPTY, sizeof(graphBufferSPL));
  memset(graphBufferdBA, GRAPH_EMPTY, sizeof(graphBufferdBA));

  Serial.println("OLED initialized!");
  delay(1000);
}


void setupSD() {
  pinMode(SD_ENABLE, OUTPUT);
  digitalWrite(SD_ENABLE, LOW);

  delay(2000);
  sdspi.begin(VSPI_SCLK,VSPI_MISO,VSPI_MOSI,VSPI_SS);
  if(!SD.begin(VSPI_SS,sdspi)){
      Serial.println("Card Mount Failed");
      return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
      Serial.println("No SD card attached");
      return;
  }

  Serial.println("SD card initialized!");

  // Create file with header
  File file = SD.open("/dB-log.csv", FILE_WRITE);
  if (file && file.size() == 0) {
    file.println("timestamp_ms,dB_SPL,dBA");
    file.close();
  }

  // Comment out if not needed
  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
      Serial.println("MMC");
  } else if(cardType == CARD_SD){
      Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
  } else {
      Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}


void setup() {
  // Enable GPIO pins for SLM
  pinMode(IO_ENABLE, OUTPUT);
  digitalWrite(IO_ENABLE, LOW);

  Serial.begin(115200);
  delay(1000);

  setupSD();
  setupOLED();
  setupI2S();

  pinMode(SLM_PIN, INPUT);
  analogReadResolution(12); // 12-bit
  analogSetAttenuation(ADC_11db); // for full 0-3.3V range
  Serial.println("SLM initialized!");
}


// Microphone: Calculate Root Mean Square
float calculateRMS(int32_t* samples, size_t len) {
  double sum = 0;
  for (size_t i = 0; i < len; i++) {
    // Right-shift 8 bits to align 24-bit left-justified data from ICS43434
    int32_t sample24 = samples[i] >> 8;

    // Normalize 24bit signed int to (-1 to 1)
    float sample = sample24 / 8388608.0f; // 2^23 = 8388608
    sum += sample * sample;
  }
  return sqrt(sum / len);
}


// Sound Level Meter: Calculate dBA from volatge
float readSLMdBA() {
  int raw = analogRead(SLM_PIN);
  float voltage = raw * (3.3 / 4095.0); // 12-bit ADC resolution
  float dBA = 50.0 * voltage + 30.0;
  return dBA;
}


// Visualization on Display
int low = 30;   // Lower bound of Y-axis
int high = 120;  // Upper bound of Y-axis

void drawGraph(float dB_SPL, float dBA) {
  // Shift graphs left
  memmove(&graphBufferSPL[0], &graphBufferSPL[1], GRAPH_WIDTH - 1);
  memmove(&graphBufferdBA[0], &graphBufferdBA[1], GRAPH_WIDTH - 1);

  // Map latest value to pixel Y
  uint8_t y_SPL = map(constrain(dB_SPL, low, high), low, high, GRAPH_HEIGHT - 1, 0);
  uint8_t y_dBA = map(constrain(dBA, low, high), low, high, GRAPH_HEIGHT - 1, 0);
  
  graphBufferSPL[GRAPH_WIDTH - 1] = y_SPL;
  graphBufferdBA[GRAPH_WIDTH - 1] = y_dBA;

  display.clearDisplay();
  display.setTextSize(1);

  // Draw axis
  int q2 = 60;
  int q3 = 80;
  display.drawFastVLine(Y_AXIS_SPACE - 1, 0, GRAPH_HEIGHT, SSD1306_WHITE);
  for (int db : {low, q2, q3, high}) {
    int y_tick = map(db, low, high, GRAPH_HEIGHT - 1, 0);
    display.drawFastHLine(Y_AXIS_SPACE - 4, y_tick, 4, SSD1306_WHITE); // tick mark

    int labelY = constrain(y_tick - 3, 1, GRAPH_HEIGHT - 9);
    display.setCursor(0, labelY);
    display.printf("%3d", db);
  }

  // Scrolling solid line for dB SPL
  for (int x = 1; x < GRAPH_WIDTH; x++) {
    // Draw line btwn points x and x-1, (if they arent 255)
    if (graphBufferSPL[x - 2] == 255 || graphBufferSPL[x - 1] == 255) continue;
    if (x % 4 < 2) // for a dashed line
      display.drawLine(Y_AXIS_SPACE + x - 1, graphBufferSPL[x - 1], Y_AXIS_SPACE + x, graphBufferSPL[x], SSD1306_WHITE);
  }

  // Dashed line for dBA
  for (int x = 1; x < GRAPH_WIDTH; x++) {
    if (graphBufferdBA[x - 1] == GRAPH_EMPTY || graphBufferdBA[x] == GRAPH_EMPTY) continue;
    display.drawLine(Y_AXIS_SPACE + x - 1, graphBufferdBA[x - 1], Y_AXIS_SPACE + x, graphBufferdBA[x], SSD1306_WHITE);
  }

  // Textual values
  display.setCursor(Y_AXIS_SPACE + 5, GRAPH_HEIGHT - 7);
  display.printf("SPL:%.1f", dB_SPL);
  display.setCursor(Y_AXIS_SPACE + GRAPH_WIDTH / 2 + 2, GRAPH_HEIGHT - 7);
  display.printf("dBA:%.1f", dBA);

  display.display();
}

void logToSD(fs::FS &fs, const char *path, unsigned long timestamp, float spl, float dba) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  file.printf("%lu,%.2f,%.2f\n", timestamp, spl, dba);
  file.close();
}


void loop() {
  size_t bytesRead = 0;
  i2s_read(I2S_PORT, (void*)sampleBuffer, sizeof(sampleBuffer), &bytesRead, portMAX_DELAY);

  size_t sampleCount = bytesRead / sizeof(int32_t);
  float rms = calculateRMS(sampleBuffer, sampleCount);

  // Mic
  float dBFS = 20.0f * log10(rms);  // dB relative to Full Scale always <= 0
  float dB_SPL = dBFS + 120.0f;     // 0 dBFS := 120 dB SPL (from mic datasheet) 

  // SLM
  float dBA = readSLMdBA();

  Serial.printf("SPL: %.2f dB, dBA: %.2f\n", dB_SPL, dBA);
  // OLED
  drawGraph(dB_SPL, dBA);

  // SD
  unsigned long timestamp = millis();
  logToSD(SD, "/dB-log.csv", timestamp, dB_SPL, dBA);

  delay(200);
}
