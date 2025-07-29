/*
 * Based on the program by Ivan Kostoski (https://github.com/ikostoski/esp32-i2s-slm)
 * 
 * Sketch samples audio data from I2S microphone, processes the data 
 * with digital IIR filters and calculates A or C weighted Equivalent 
 * Continuous Sound Level (Leq)
 * 
 * I2S is setup to sample data at Fs=48000KHz (fixed value due to 
 * design of digital IIR filters). Data is read from I2S queue 
 * in 'sample blocks' (default 125ms block, equal to 6000 samples) 
 * by 'i2s_reader_task', filtered trough two IIR filters (equalizer 
 * and weighting), summed up and pushed into 'samples_queue' as 
 * sum of squares of filtered samples. The main task then pulls data 
 * from the queue and calculates decibel value relative to microphone 
 * reference amplitude, derived from datasheet sensitivity dBFS 
 * value, number of bits in I2S data, and the reference value for 
 * which the sensitivity is specified (typically 94dB, pure sine
 * wave at 1KHz).
 * 
 * Displays line on the small OLED screen with 'short' LAeq(125ms)
 * response and numeric LAeq(1sec) dB value from the signal RMS.
 */

#include <driver/i2s.h>
#include "sos-iir-filter.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

//
// Configuration
//

#define LEQ_PERIOD        1           // second(s)
#define WEIGHTING         A_weighting // Also avaliable: 'A_weighting' or 'None' (Z_weighting)
#define LEQ_UNITS         "LeqA"      // customize based on above weighting used (for display)
#define DB_UNITS          "dBA"       // customize based on above weighting used (for display)
#define USE_DISPLAY       0

// NOTE: Some microphones require at least DC-Blocker filter
#define MIC_EQUALIZER     ICS43434    // ICS43434 or set to 'None' to disable. (Find more IIR filters in OG repo!)
#define MIC_OFFSET_DB     3.0103      // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration

// Customize these values from microphone datasheet
#define MIC_SENSITIVITY   -26         // dBFS value expected at MIC_REF_DB (Sensitivity value from datasheet)
#define MIC_REF_DB        94.0        // Value at which point sensitivity is specified in datasheet (dB)
#define MIC_OVERLOAD_DB   120.0       // dB - Acoustic overload point
#define MIC_NOISE_DB      29          // dB - Noise floor
#define MIC_BITS          24          // valid number of bits in I2S data
#define MIC_CONVERT(s)    (s >> (SAMPLE_BITS - MIC_BITS)) // Shift to remove unused LSBs
#define MIC_TIMING_SHIFT  0           // Set to one to fix MSB timing for some microphones, i.e. SPH0645LM4H-x

// Calculate reference amplitude value at compile time
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);

//
// I2S pins - Can be routed to almost any (unused) ESP32 pin.
//            SD can be any pin, inlcuding input only pins (36-39).
//            SCK (i.e. BCLK) and WS (i.e. L/R CLK) must be output capable pins
//
// sensebox:eye (ESP32S3)
#define I2S_WS          PIN_IO48
#define I2S_SCK         PIN_QWIIC_SCL 
#define I2S_SD          PIN_QWIIC_SDA

// I2S peripheral to use (0 or 1)
#define I2S_PORT        I2S_NUM_0


// SD pins
#define VSPI_MISO       40
#define VSPI_MOSI       38
#define VSPI_SCLK       39
#define VSPI_SS         41
#define SD_ENABLE       3

SPIClass sdspi = SPIClass();
File logFile;

//
// Setup your display library (and geometry) here
// 
#if (USE_DISPLAY > 0)
// TODO
#endif

// 
// Equalizer IIR filters to flatten microphone frequency response
// See respective .m file for filter design. Fs = 48Khz.
//
// Filters are represented as Second-Order Sections cascade with assumption
// that b0 and a0 are equal to 1.0 and 'gain' is applied at the last step 
// B and A coefficients were transformed with GNU Octave: 
// [sos, gain] = tf2sos(B, A)
// See: https://www.dsprelated.com/freebooks/filters/Series_Second_Order_Sections.html
// NOTE: SOS matrix 'a1' and 'a2' coefficients are negatives of tf2sos output
//

// TDK/InvenSense ICS-43434
// Datasheet: https://www.invensense.com/wp-content/uploads/2016/02/DS-000069-ICS-43434-v1.1.pdf
// B = [0.477326418836803, -0.486486982406126, -0.336455844522277, 0.234624646917202, 0.111023257388606];
// A = [1.0, -1.93073383849136326, 0.86519456089576796, 0.06442838283825100, 0.00111249298800616];
SOS_Coefficients ICS43434_sos[] = { // Second-Order Sections {b1, b2, -a1, -a2}
  {+0.96986791463971267f, 0.23515976355743193f, -0.06681948004769928f, -0.00111521990688128f},
  {-1.98905931743624453f, 0.98908924206960169f, +1.99755331853906037f, -0.99755481510122113f}
};

SOS_IIR_Filter ICS43434(0.477326418836803f, ICS43434_sos);

//
// Weighting filters
//

//
// A-weighting IIR Filter, Fs = 48KHz 
// (By Dr. Matt L., Source: https://dsp.stackexchange.com/a/36122)
// B = [0.169994948147430, 0.280415310498794, -1.120574766348363, 0.131562559965936, 0.974153561246036, -0.282740857326553, -0.152810756202003]
// A = [1.0, -2.12979364760736134, 0.42996125885751674, 1.62132698199721426, -0.96669962900852902, 0.00121015844426781, 0.04400300696788968]
SOS_Coefficients A_weighting_sos[] = { // Second-Order Sections {b1, b2, -a1, -a2}
  {-2.00026996133106f, +1.00027056142719f, -1.060868438509278f, -0.163987445885926f},
  {+4.35912384203144f, +3.09120265783884f, +1.208419926363593f, -0.273166998428332f},
  {-0.70930303489759f, -0.29071868393580f, +1.982242159753048f, -0.982298594928989f}
};

SOS_IIR_Filter A_weighting(0.169994948147430f, A_weighting_sos);


//
// Sampling
//
#define SAMPLE_RATE       48000 // Hz, fixed to design of IIR filters
#define SAMPLE_BITS       32
#define SAMPLE_T          int32_t 
#define SAMPLES_SHORT     (SAMPLE_RATE / 8) // ~125ms
#define SAMPLES_LEQ       (SAMPLE_RATE * LEQ_PERIOD)
#define DMA_BANK_SIZE     (SAMPLES_SHORT / 16)
#define DMA_BANKS         32

// Static buffer for block of samples
float samples[SAMPLES_SHORT] __attribute__((aligned(4)));

void mic_i2s_init() {
  // Setup I2S to sample mono channel for SAMPLE_RATE * SAMPLE_BITS
  // NOTE: Recent update to Arduino_esp32 (1.0.2 -> 1.0.3)
  //       seems to have swapped ONLY_LEFT and ONLY_RIGHT channels
  const i2s_config_t i2s_config = {
    mode: i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    sample_rate: SAMPLE_RATE,
    bits_per_sample: i2s_bits_per_sample_t(SAMPLE_BITS),
    channel_format: I2S_CHANNEL_FMT_ONLY_LEFT,
    communication_format: i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,
    dma_buf_count: DMA_BANKS,
    dma_buf_len: DMA_BANK_SIZE,
    use_apll: true,
    tx_desc_auto_clear: false,
    fixed_mclk: 0
  };
  // I2S pin mapping
  const i2s_pin_config_t pin_config = {
    bck_io_num:   I2S_SCK,
    ws_io_num:    I2S_WS,
    data_out_num: -1, // not used
    data_in_num:  I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  #if (MIC_TIMING_SHIFT > 0)
    // Undocumented (?!) manipulation of I2S peripheral registers
    // to fix MSB timing issues with some I2S microphones
    REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));
    REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);
  #endif

  i2s_set_pin(I2S_PORT, &pin_config);
}

// Data we push to 'samples_queue'
struct sum_queue_t {
  // Sum of squares of mic samples, after Equalizer filter
  float sum_sqr_SPL;
  // Sum of squares of weighted mic samples
  float sum_sqr_weighted;
};

sum_queue_t q;
uint32_t Leq_samples = 0;
double Leq_sum_sqr = 0;
double Leq_dB = 0;

void setup() {
  setCpuFrequencyMhz(80);
  Serial.begin(112500);
  delay(1000);

  // Initialize SD card
  pinMode(SD_ENABLE, OUTPUT);
  digitalWrite(SD_ENABLE, LOW);
  delay(2000);

  sdspi.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);

  if(!SD.begin(VSPI_SS, sdspi)){
      Serial.println("Card Mount Failed");
      return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
      Serial.println("No SD card attached");
      return;
  }

  Serial.println("SD card initialized.");

  // Create log file header
  logFile = SD.open("/slm_log.csv", FILE_APPEND);
  if (logFile) {
      logFile.println("Timestamp_ms,Leq_dB");
      logFile.close();
  } else {
      Serial.println("Failed to open log file for writing.");
  }

  // Initialize mic
  mic_i2s_init();

  // Discard first block, microphone may have startup time (i.e. INMP441 up to 83ms)
  size_t bytes_read;
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);
}

void loop() {
  size_t bytes_read;

  // Block and wait for microphone values from I2S
  //
  // Data is moved from DMA buffers to our 'samples' buffer by the driver ISR
  // and when there is requested ammount of data, task is unblocked
  //
  // Note: i2s_read does not care it is writing in float[] buffer, it will write
  //       integer values to the given address, as received from the hardware peripheral. 
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);

  // Convert (including shifting) integer microphone values to floats, 
  // using the same buffer (assumed sample size is same as size of float), 
  // to save a bit of memory
  SAMPLE_T* int_samples = (SAMPLE_T*)&samples;
  for (int i = 0; i < SAMPLES_SHORT; i++) {
    samples[i] = MIC_CONVERT(int_samples[i]);
  }

  // Apply equalization and calculate Z-weighted sum of squares, 
  // writes filtered samples back to the same buffer.
  q.sum_sqr_SPL = MIC_EQUALIZER.filter(samples, samples, SAMPLES_SHORT);

  // Apply weighting and calucate weigthed sum of squares
  q.sum_sqr_weighted = WEIGHTING.filter(samples, samples, SAMPLES_SHORT);

  // Calculate dB values relative to MIC_REF_AMPL and adjust for microphone reference
  double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
  double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

  // In case of acoustic overload or below noise floor measurement, report infinty Leq value
  if (short_SPL_dB > MIC_OVERLOAD_DB) {
    Leq_sum_sqr = INFINITY;
  } else if (isnan(short_SPL_dB) || short_SPL_dB < MIC_NOISE_DB) {
    Leq_sum_sqr = -INFINITY;
  }

  // Accumulate Leq sum
  Leq_sum_sqr += q.sum_sqr_weighted;
  Leq_samples += SAMPLES_SHORT;

  // When we gather enough samples, calculate new Leq value
  if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD) {
      double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
      Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
      Leq_sum_sqr = 0;
      Leq_samples = 0;

      Serial.printf("Peak:%.0f, ", MIC_OVERLOAD_DB);
      Serial.printf("NoiseFloor:%i, ", MIC_NOISE_DB);
      Serial.printf("%s:%.0f\n", DB_UNITS, Leq_dB);

      // Log to SD card
      logFile = SD.open("/slm_log.csv", FILE_APPEND);
      if (logFile) {
          unsigned long timestamp = millis();
          logFile.printf("%lu,%.2f\n", timestamp, Leq_dB);
          logFile.close();
      } else {
          Serial.println("Failed to open log file for writing.");
      }
  }
}
