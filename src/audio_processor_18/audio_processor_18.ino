// audio processor
// (C) Jef Collin
// 2025



// reminder: when lvgl library is updated, edit the config file!!!

// update firmware version number dsp_firmware_version for automatic check and download
// make sure WP jumer is ON
// note that it takes a long time to download the new firmware
// there is no soft or hard reset function on the dsp module so power cycle after software update

// notes on the dsp stuff
// 0dB = float value 1 = approx 0.95V RMS
// I2S data is 24 bit in 32 bit form, shift right 8 positions, then divide by 8388608 (2 ^ 23) to get the float data 0-1
// readback is 3 bytes, no shift just divide by 524288 (2 ^ 19)
// line out to external amp 0.1v rms
// vu meter top green = 0.1v rms 0.2828vpp
// speaker out ca 90mv rms (only green leds) enough to drive the external amplifier




// todo



// libraries
#include <SPI.h>
#include <Preferences.h>
#include <lvgl.h>
#include <Wire.h>
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/semphr.h>
#include <SigmaDSP.h>
#include <driver/i2s.h>
#include <vector>
#include <algorithm>
#include <nvs_flash.h>
#include <nvs.h>
#include <set>
#include "arduinoFFT.h"
#include <LovyanGFX.hpp>


// Include generated parameter file
#include "SigmaDSP_parameters.h"

// change when the dsp code changes, this will trigger an update of the eeprom
int8_t dsp_firmware_version = 27;

// note config shows 0X68 and 0XA0 while I2C scanner shows 0X34 and 0X50, this is explained as 7 bit and 8 bit values

// The first parameter is the Wire object we'll be using when communicating wth the DSP
// The second parameter is the DSP i2c address, which is defined in the parameter file
// The third parameter is the sample rate
// An optional fourth parameter is the pin to physically reset the DSP
SigmaDSP dsp(Wire, DSP_I2C_ADDRESS, 48000.00f);

// Write Protect pin for dsp eeprom
// if low on power on the dsp fails
#define EEPROM_WP_PIN 14

// Only needed if an external i2c EEPROM is present + the DSP is in selfboot mode
// The first parameter is the Wire object we'll be using when communicating wth the EEPROM
// The second parameter is the EEPROM i2c address, which is defined in the parameter file
// The third parameter is the EEPROM size in kilobits (kb)
// An optional fourth parameter is the pin to toggle while writing content to EEPROM (for led not wp pin)
DSPEEPROM ee(Wire, EEPROM_I2C_ADDRESS, 256, 0);

SemaphoreHandle_t i2cMutex; // handle for the mutex

// update after change in sequence or new tab

#define TAB_MAIN_REF        0
#define TAB_MENU_REF        1
#define TAB_NOTCH_REF       2
#define TAB_FILTER_REF      3
#define TAB_PEAK_REF        4
#define TAB_COMPRESSOR_REF  5
#define TAB_EQUALIZER_REF   6
#define TAB_SCOPE_REF       7
#define TAB_SPECTRUM_REF    8
#define TAB_MORSE_REF       9
#define TAB_PRESETS_REF     10
#define TAB_TEST_REF        11
#define TAB_SETTINGS_REF    12
#define TAB_EDIT_REF        13
#define TAB_DYNAMIC_REF     14
#define TAB_PRESETSMAIN_REF 15

// I2S Configuration
#define I2S_NUM               I2S_NUM_0
#define SAMPLE_RATE           48000
#define BLOCK_SIZE            256

// must be multiple of block size
#define AUDIO_BUFFER_SIZE     4096

#define PIN_BCK               38
#define PIN_WS                39
#define PIN_DATA_IN           47
#define PIN_DATA_OUT          21

// transmit button
#define PTT_IN 18

// lvgl
#define LV_USE_DEBUG = 0;

// 3.5" display
#define TFT_HOR_RES 480
#define TFT_VER_RES 320
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 5 * (LV_COLOR_DEPTH / 8))

// use alps or other decoder
#define Use_Alps_Encoder false

// rotary encode io pins
// swap S1 and S2 pins depending on type of encoder
#define Encoder_1_Pin1 6
#define Encoder_1_Pin2 7
#define Encoder_2_Pin1 16
#define Encoder_2_Pin2 17
#define Encoder_1_Key 5
#define Encoder_2_Key 15

// rotary encoder
// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

#if (Use_Alps_Encoder)
// Alps EC11 encoder requires half step tables, others need full step
#define R_START 0x0
#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5

const unsigned char ttable[6][4] = {
  // R_START (00)
  {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
  // R_CCW_BEGIN
  {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
  // R_CW_BEGIN
  {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
  // R_START_M (11)
  {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
  // R_CW_BEGIN_M
  {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
  // R_CCW_BEGIN_M
  {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};

#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6
#define R_START 0x0

const unsigned char ttable[7][4] = {
  // R_START
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif

unsigned char Encoder_1_State = R_START;
unsigned char Encoder_2_State = R_START;

// track rotary encoder changes
int EncoderCounter1 = 0;
int EncoderCounter2 = 0;

long unsigned timer_encoderbutton1;
long unsigned timer_encoderbutton2;

boolean Encoder_Key1_Long_Press = false;
boolean Encoder_Key2_Long_Press = false;

char printbuf[100];

// due to the lovyangxf lib the IDE enforces more rules hence we need the structure definitions in front of the gxf class

typedef struct LMS_Context_t {
  // Buffers and state
  int32_t *inBuf, *outBuf;
  float *x, *w, *dline;
  int NFIR, NDEL_NOTCH, NDEL_DENOISE;
  int d_len, d_head, x_head, decay_counter;

  // Control flags
  bool enabled, mode_notch, stop_requested;
  bool denoiser_enabled;
  bool notch_enabled;

  // Audio parameters
  float PCM24_SCALE, PCM24_INV;
  float BETA_NOTCH, BETA_DENOISE, DECAY_NOTCH, DECAY_DENOISE;
  float gain_notch, gain_denoise;

  // AGC parameters
  bool agc_enabled, agc_needs_reset, agc_first_run;
  float agc_target_notch, agc_target_denoise;
  float agc_output_gain_notch, agc_output_gain_denoise;
  float agc_output_gain;
  float agc_current_target;  // ADDED MISSING MEMBER
  float agc_min_gain, agc_max_gain;
  float agc_attack_ms, agc_release_ms;
  float agc_env, agc_rms, agc_gain;
  int agc_soft_start_counter;
};

// compressor definition
typedef struct audio_compressor_t {
  float threshold = -60.0f;  // dB threshold
  float ratio     = 4.0f;    // compression ratio
  float rms_tc    = 120.0f;
  float hold      = 10.0f;
  float decay     = 10.0f;
  float postgain  = 0.0f;
  float knee      = 10.0f;    // dB width of the knee
};

// create compressor instance
audio_compressor_t audio_compressor_parameters;

// noise gate definition
typedef struct audio_noisegate_t {
  float gateThreshold = -60.0f;  // dB threshold
  float gateKnee     = 4.0f;    // compression ratio
  float gateFloor    = 1.0f;
};

// create compressor instance
audio_noisegate_t audio_noisegate_parameters;

// morse decoding states
typedef enum {
  STATE_SPACE,
  STATE_MARK
} morse_state_t;

// morse symbol buffer
struct morse_decoder_t {
  morse_state_t m_state;
  uint32_t duration;
  uint32_t unit_samples;
  char symbol[8];
  uint8_t sym_len;
  uint16_t last_env;
  float signal_peak;
  float noise_floor;
  uint32_t char_gap_threshold;
  uint32_t word_gap_threshold;
  uint32_t last_calibration_time;
  float peak_enveloppe;
  float threshold;
};

morse_decoder_t morse_decoder;

// graphics lib configuration
class LGFX : public lgfx::LGFX_Device
{
    lgfx::Panel_ILI9488     _panel_instance;
    lgfx::Touch_XPT2046     _touch_instance;
    lgfx::Bus_SPI           _bus_instance;
  public:
    LGFX(void)
    {
      {
        auto cfg = _bus_instance.config();
        // SPI
        cfg.spi_host = SPI2_HOST;
        cfg.spi_mode = 0;
        cfg.freq_write = 60000000;
        cfg.freq_read  = 16000000;
        cfg.spi_3wire  = true;
        cfg.use_lock   = true;
        cfg.dma_channel = SPI_DMA_CH_AUTO;
        cfg.pin_sclk = 12;
        cfg.pin_mosi = 11;
        cfg.pin_miso = 13;
        cfg.pin_dc   = 2;

        _bus_instance.config(cfg);
        _panel_instance.setBus(&_bus_instance);
      }

      {
        auto cfg = _panel_instance.config();
        cfg.pin_cs           =    10;
        cfg.pin_rst          =    -1;
        cfg.pin_busy         =    -1;

        cfg.panel_width      =   320;
        cfg.panel_height     =   480;
        cfg.offset_x         =     0;
        cfg.offset_y         =     0;
        cfg.offset_rotation  =     0;
        cfg.dummy_read_pixel =     8;
        cfg.dummy_read_bits  =     1;
        cfg.readable         =  true;
        cfg.invert           = false;
        cfg.rgb_order        = false;
        cfg.dlen_16bit       = false;
        cfg.bus_shared       =  true;

        _panel_instance.config(cfg);
      }
      {
        // must be raw data point but not used if calibration is used
        auto cfg = _touch_instance.config();
        cfg.x_min      = 0;
        cfg.x_max      = 4000;
        cfg.y_min      = 0;
        cfg.y_max      = 4000;
        cfg.pin_int    = -1;
        cfg.bus_shared = true;
        cfg.offset_rotation = 0;

        // SPI
        cfg.spi_host = SPI2_HOST;
        cfg.freq = 1000000;
        cfg.pin_sclk = 12;
        cfg.pin_mosi = 11;
        cfg.pin_miso = 13;
        cfg.pin_cs   =  42;
        _touch_instance.config(cfg);
        _panel_instance.setTouch(&_touch_instance);
      }
      setPanel(&_panel_instance);
    }
};

LGFX TFT;

LGFX_Sprite morseSprite(&TFT);

// callback routine for display
void my_disp_flush(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);
  lv_draw_sw_rgb565_swap(px_map, w * h);
  TFT.startWrite();
  TFT.pushImageDMA(area->x1, area->y1, w, h, (uint16_t*)px_map);
  TFT.endWrite();
  lv_disp_flush_ready(disp);
}

// two buffer system
void *draw_buf_1;
void *draw_buf_2;

unsigned long lastTickMillis = 0;

static lv_display_t *disp;

Preferences preferences;

// touch screen calib
uint16_t audio_setting_calibration_data[8];

// timers
unsigned long LVGL_Timer;
unsigned long key1_timer;
unsigned long key2_timer;
unsigned long prior_tick_Millis = millis();
unsigned long audio_vu_update_timer;

boolean audio_screen_update_done = false;

// slider to encoder match
boolean audio_rotary_selected_gain = true;
boolean audio_rotary_selected_speaker = true;

boolean audio_rotary_selected_threshold = true;
boolean audio_rotary_selected_postgain = true;

uint8_t audio_input_selected = 0;
uint8_t audio_input_selected_previous = 0;
boolean audio_input_muted = false;

int8_t audio_vu_meter = 0;

float audio_vu_readback = 0;

int32_t audio_gain = 10;
int32_t audio_line_out = 10;
int32_t audio_speaker = 0;
int32_t audio_headphone = 0;

int32_t filter_notch_freq0 = 100;
int32_t filter_notch_freq1 = 100;
int32_t filter_notch_freq2 = 100;

int32_t filter_notch_bandwidth0 = 5;
int32_t filter_notch_bandwidth1 = 5;
int32_t filter_notch_bandwidth2 = 5;

boolean filter_notch_on0 = false;
boolean filter_notch_on1 = false;
boolean filter_notch_on2 = false;

uint8_t audio_selected_notch = 0;
boolean audio_notch_lock_updates = false;

int32_t filter_highpass_freq = 100;
int32_t filter_lowpass_freq = 4000;

int32_t filter_bandpass_freq = 2050;
int32_t filter_bandwidth = 3900;

// selected change mode, true for hi-lo limits, false for center f and bandwidth
boolean audio_filter_mode_hi_lo = true;

// filter index to match dropdown
uint8_t audio_filter_mode = 0;

// to prevent race conditions we lock updates
boolean audio_filter_lock_updates = false;

int32_t filter_peak_bandpass_freq = 2050;
int32_t filter_peak_bandwidth = 5;
boolean filter_peak_on = false;

boolean audio_dynamic_denoiser = false;
boolean audio_dynamic_notch = false;
boolean audio_dynamic_agc = false;

#define AGC_SAMPLE_RATE 48000.0f
#define EPSILON 1e-10f

// Command definitions
#define CMD_ENABLE_SET      (1 << 0)
#define CMD_ENABLE_CLEAR    (1 << 1)
#define CMD_MODE_NOTCH      (1 << 2)
#define CMD_MODE_DENOISE    (1 << 3)
#define CMD_AGC_ENABLE      (1 << 4)
#define CMD_AGC_DISABLE     (1 << 5)
#define CMD_AGC_RESET       (1 << 6)
#define CMD_STOP            (1 << 7)
#define CMD_DENOISER_ENABLE     (1 << 8)
#define CMD_DENOISER_DISABLE    (1 << 9)
#define CMD_NOTCH_ENABLE        (1 << 10)
#define CMD_NOTCH_DISABLE       (1 << 11)

TaskHandle_t lmsTaskHandle = NULL;

int32_t audio_compressor_threshold = 0;
int32_t audio_compressor_ratio = 1;
int32_t audio_compressor_postgain = 0;
int32_t audio_compressor_noisegate = 0;

// Create an instance for each EQ block
secondOrderEQ eqBand0;
secondOrderEQ eqBand1;
secondOrderEQ eqBand2;
secondOrderEQ eqBand3;
secondOrderEQ eqBand4;
secondOrderEQ eqBand5;
secondOrderEQ eqBand6;
secondOrderEQ eqBand7;
secondOrderEQ eqBand8;
secondOrderEQ eqBand9;
secondOrderEQ eqBand10;
secondOrderEQ eqBand11;

uint8_t audio_selected_eq = 0;

int32_t audio_eq0 = 0;
int32_t audio_eq1 = 0;
int32_t audio_eq2 = 0;
int32_t audio_eq3 = 0;
int32_t audio_eq4 = 0;
int32_t audio_eq5 = 0;
int32_t audio_eq6 = 0;
int32_t audio_eq7 = 0;
int32_t audio_eq8 = 0;
int32_t audio_eq9 = 0;
int32_t audio_eq10 = 0;
int32_t audio_eq11 = 0;

#define MORSE_BUF_SIZE 64
#define MORSE_SPRITE_W 400
#define MORSE_SPRITE_H 24
#define MORSE_NOISE_FLOOR         0.1

boolean morse_on = false;
boolean morse_buffer_ready = false;
int32_t morse_inputbuffer[BLOCK_SIZE];
int morse_dot_width = 5;
int morse_dot_height = 5;
int morse_dash_width = 12;
int morse_space_width = 8;
int morse_spacing = 6;
int morse_spacing2 = 4;
int morse_scroll_offset = 0;
int morse_ticker_tail = 0;
int morse_ticker_head = 0;
const int MORSE_TICKER_BUF_SIZE = 256;
char morse_ticker_buf[MORSE_TICKER_BUF_SIZE];
volatile bool morse_ticker_changed = false;
volatile char morse_buffer[MORSE_BUF_SIZE];
volatile uint8_t morse_head = 0;
volatile uint8_t morse_tail = 0;
String morse_full_buffer = "";

struct morse_code_tbl {
  const char* code;
  char symbol;
};

const morse_code_tbl morse_table[] = {
  {".-",    'A'}, {"-...",  'B'}, {"-.-.",  'C'}, {"-..",   'D'}, {".",     'E'},
  {"..-.",  'F'}, {"--.",   'G'}, {"....",  'H'}, {"..",    'I'}, {".---",  'J'},
  {"-.-",   'K'}, {".-..",  'L'}, {"--",    'M'}, {"-.",    'N'}, {"---",   'O'},
  {".--.",  'P'}, {"--.-",  'Q'}, {".-.",   'R'}, {"...",   'S'}, {"-",     'T'},
  {"..-",   'U'}, {"...-",  'V'}, {".--",   'W'}, {"-..-",  'X'}, {"-.--",  'Y'},
  {"--..",  'Z'},
  {"-----", '0'}, {".----", '1'}, {"..---", '2'}, {"...--", '3'}, {"....-", '4'},
  {".....", '5'}, {"-....", '6'}, {"--...", '7'}, {"---..", '8'}, {"----.", '9'},
  {".-.-.-", '.'}, {"--..--", ','}, {"..--..", '?'}, {".----.", '\''},
  {"-.-.--", '!'}, {"-..-.", '/'}, {"-.--.", '('}, {"-.--.-", ')'},
  {".-...", '&'}, {"---...", ':'}, {"-.-.-.", ';'}, {"-...-", '='},
  {".-.-.", '+'}, {"-....-", '-'}, {"..--.-", '_'}, {".-..-.", '"'},
  {".--.-.", '@'},
  {NULL,    '\0'}  // end marker
};

uint8_t audio_presets_sel = 0;

String audio_presets_msg[] = {
  "Preset 1",
  "Preset 2",
  "Preset 3",
  "Preset 4",
  "Preset 5",
  "Preset 6",
  "Preset 7",
  "Preset 8",
  "Preset 9",
  "Preset 10"
};

int32_t audio_sample_buffer[2048];

// Ringbuffer
volatile int32_t audio_buffer[AUDIO_BUFFER_SIZE];
volatile size_t audio_buffer_write_index = 0;
volatile size_t audio_buffer_read_index = 0;

long unsigned audio_scope_timer;
int8_t* audio_scope_previous;
float audio_scope_scale = 8000;
boolean audio_scope_autoscale = false;
volatile bool audio_scope_trigger = true;
volatile bool audio_scope_ready = false;
int32_t audio_scope_buffer[2048];
static uint8_t audio_scope_fill_block_count = 0;

// spectrum sampling and FFT

#define FFT_SIZE 2048
#define MAX_FREQ 4000
#define BIN_WIDTH (SAMPLE_RATE / FFT_SIZE)
#define NUM_BINS_TO_PROCESS (MAX_FREQ / BIN_WIDTH)
#define NUM_BANDS 119
#define BAND_WIDTH (NUM_BINS_TO_PROCESS / NUM_BANDS)

int audio_spectrum_band_values[NUM_BANDS] = {0};

static double audio_spectrum_vReal[FFT_SIZE];
static double audio_spectrum_vImag[FFT_SIZE];

ArduinoFFT<double> FFT = ArduinoFFT<double>(audio_spectrum_vReal, audio_spectrum_vImag, FFT_SIZE, SAMPLE_RATE);

uint8_t audio_spectrum_cursor = 0;
uint8_t audio_spectrum_cursor_last_pos = 255;
boolean audio_spectrum_peaktrigger = false;
boolean audio_spectrum_bandpasstrigger = false;
int32_t audio_spectrum_low_freq = 100;
int32_t audio_spectrum_high_freq = 4000;
int32_t audio_test_signal_gain = 0;

int8_t audio_test_mode = 0;
int32_t audio_test_frequency1 = 1000;
uint8_t audio_test_enc2_mode = 0;
boolean audio_test_cursor_override = false;

// optional ptt button input will mute sound during transmit
boolean audio_ptt_active = false;
boolean audio_ptt_active_last = false;
uint16_t audio_ptt_debounce = 100;
volatile bool ptt_event_flag = false;
volatile unsigned long audio_ptt_debounce_timer = 0;

// for keyboard routine
uint8_t kb_caller = 0;
uint8_t kb_returntab = 0;
String kb_text = "";

// lvgl screen variables
static lv_obj_t *tabview;

static lv_obj_t *tab_main;
static lv_obj_t *tab_menu;
static lv_obj_t *tab_notch;
static lv_obj_t *tab_filter;
static lv_obj_t *tab_peak;
static lv_obj_t *tab_compressor;
static lv_obj_t *tab_equalizer;
static lv_obj_t *tab_scope;
static lv_obj_t *tab_spectrum;
static lv_obj_t *tab_morse;
static lv_obj_t *tab_presets;
static lv_obj_t *tab_test;
static lv_obj_t *tab_settings;
static lv_obj_t *tab_edit;
static lv_obj_t *tab_dynamic;
static lv_obj_t *tab_presetsmain;

static lv_obj_t *slider_main_gain;
static lv_obj_t *slider_main_output;
static lv_obj_t *slider_main_speaker;
static lv_obj_t *slider_main_headphone;

static lv_obj_t *lbl_main_gain;
static lv_obj_t *lbl_main_output;
static lv_obj_t *lbl_main_speaker;
static lv_obj_t *lbl_main_headphone;

static lv_obj_t * btn_main_preset;
static lv_obj_t * btn_main_menu;

static lv_obj_t *led_main_tx;
static lv_obj_t *lbl_main_tx;

static lv_obj_t * lbl_notch0;
static lv_obj_t * lbl_notch1;
static lv_obj_t * lbl_notch2;

static lv_obj_t * cb_notch_on0;
static lv_obj_t * cb_notch_on1;
static lv_obj_t * cb_notch_on2;

static lv_obj_t * spinbox_notch_frequency_0;
static lv_obj_t * spinbox_notch_frequency_1;
static lv_obj_t * spinbox_notch_frequency_2;

static lv_obj_t * btn_notch_frequency_0_inc;
static lv_obj_t * btn_notch_frequency_0_dec;

static lv_obj_t * btn_notch_frequency_1_inc;
static lv_obj_t * btn_notch_frequency_1_dec;

static lv_obj_t * btn_notch_frequency_2_inc;
static lv_obj_t * btn_notch_frequency_2_dec;

static lv_obj_t * spinbox_notch_width_0;
static lv_obj_t * spinbox_notch_width_1;
static lv_obj_t * spinbox_notch_width_2;

static lv_obj_t * btn_notch_width_0_inc;
static lv_obj_t * btn_notch_width_0_dec;

static lv_obj_t * btn_notch_width_1_inc;
static lv_obj_t * btn_notch_width_1_dec;

static lv_obj_t * btn_notch_width_2_inc;
static lv_obj_t * btn_notch_width_2_dec;

static lv_obj_t * btn_notch_back;

static lv_obj_t * spinbox_filter_highpass;
static lv_obj_t * spinbox_filter_lowpass;
static lv_obj_t * spinbox_filter_bandpass;
static lv_obj_t * spinbox_filter_bandwidth;

static lv_obj_t * btn_filter_highpass_inc;
static lv_obj_t * btn_filter_highpass_dec;

static lv_obj_t * btn_filter_lowpass_inc;
static lv_obj_t * btn_filter_lowpass_dec;

static lv_obj_t * btn_filter_bandpass_inc;
static lv_obj_t * btn_filter_bandpass_dec;

static lv_obj_t * btn_filter_bandwidth_inc;
static lv_obj_t * btn_filter_bandwidth_dec;

static lv_obj_t * dropdown_filter_mode;

static lv_obj_t * btn_filter_reset;
static lv_obj_t * btn_filter_back;

static lv_obj_t * cb_peak_on;
static lv_obj_t * spinbox_peak_bandpass;
static lv_obj_t * btn_peak_bandpass_inc;
static lv_obj_t * btn_peak_bandpass_dec;
static lv_obj_t * spinbox_peak_bandwidth;
static lv_obj_t * btn_peak_bandwidth_inc;
static lv_obj_t * btn_peak_bandwidth_dec;

static lv_obj_t * btn_peak_back;

static lv_obj_t * cb_dynamic_on;
static lv_obj_t * cb_dynamic_autonotch;
static lv_obj_t * cb_dynamic_agc;
static lv_obj_t * btn_dynamic_back;

static lv_obj_t *slider_compressor_threshold;
static lv_obj_t *slider_compressor_ratio;
static lv_obj_t *slider_compressor_postgain;
static lv_obj_t *slider_compressor_noisegate;

static lv_obj_t *lbl_compressor_threshold;
static lv_obj_t *lbl_compressor_ratio;
static lv_obj_t *lbl_compressor_postgain;
static lv_obj_t *lbl_compressor_noisegate;

static lv_obj_t *btn_compressor_reset;
static lv_obj_t *btn_compressor_back;

static lv_obj_t *slider_eq0;
static lv_obj_t *slider_eq1;
static lv_obj_t *slider_eq2;
static lv_obj_t *slider_eq3;
static lv_obj_t *slider_eq4;
static lv_obj_t *slider_eq5;
static lv_obj_t *slider_eq6;
static lv_obj_t *slider_eq7;
static lv_obj_t *slider_eq8;
static lv_obj_t *slider_eq9;
static lv_obj_t *slider_eq10;
static lv_obj_t *slider_eq11;

static lv_obj_t *lbl_eq0;
static lv_obj_t *lbl_eq1;
static lv_obj_t *lbl_eq2;
static lv_obj_t *lbl_eq3;
static lv_obj_t *lbl_eq4;
static lv_obj_t *lbl_eq5;
static lv_obj_t *lbl_eq6;
static lv_obj_t *lbl_eq7;
static lv_obj_t *lbl_eq8;
static lv_obj_t *lbl_eq9;
static lv_obj_t *lbl_eq10;
static lv_obj_t *lbl_eq11;

static lv_obj_t *btn_eq_reset;
static lv_obj_t *btn_eq_back;

static lv_obj_t * cb_presets_selection0;
static lv_obj_t * cb_presets_selection1;
static lv_obj_t * cb_presets_selection2;
static lv_obj_t * cb_presets_selection3;
static lv_obj_t * cb_presets_selection4;
static lv_obj_t * cb_presets_selection5;
static lv_obj_t * cb_presets_selection6;
static lv_obj_t * cb_presets_selection7;
static lv_obj_t * cb_presets_selection8;
static lv_obj_t * cb_presets_selection9;

static lv_obj_t * preset_checkboxes[10];

static lv_obj_t * btn_presets_set;
static lv_obj_t * btn_presets_recall;
static lv_obj_t * btn_presets_back;

static lv_obj_t * btn_scope_back;

static lv_obj_t * btn_presetsmain_preset0;
static lv_obj_t * btn_presetsmain_preset1;
static lv_obj_t * btn_presetsmain_preset2;
static lv_obj_t * btn_presetsmain_preset3;
static lv_obj_t * btn_presetsmain_preset4;
static lv_obj_t * btn_presetsmain_preset5;
static lv_obj_t * btn_presetsmain_preset6;
static lv_obj_t * btn_presetsmain_preset7;
static lv_obj_t * btn_presetsmain_preset8;
static lv_obj_t * btn_presetsmain_preset9;

static lv_obj_t * lbl_presetsmain_preset0;
static lv_obj_t * lbl_presetsmain_preset1;
static lv_obj_t * lbl_presetsmain_preset2;
static lv_obj_t * lbl_presetsmain_preset3;
static lv_obj_t * lbl_presetsmain_preset4;
static lv_obj_t * lbl_presetsmain_preset5;
static lv_obj_t * lbl_presetsmain_preset6;
static lv_obj_t * lbl_presetsmain_preset7;
static lv_obj_t * lbl_presetsmain_preset8;
static lv_obj_t * lbl_presetsmain_preset9;

static lv_obj_t * preset_mainbuttons[10];

//static lv_obj_t * btn_presetsmain_back;

static lv_obj_t * lbl_spectrum_cursor;

static lv_obj_t * btn_spectrum_back;

static lv_obj_t *obj_morse_text_container;
static lv_obj_t *lbl_morse_text;
static lv_obj_t *lbl_morse_freq;
static lv_obj_t *cb_morse_on;
static lv_obj_t *btn_morse_clear;
static lv_obj_t *btn_morse_back;

static lv_obj_t *slider_test_signal_gain;

static lv_obj_t * cb_test_on;

static lv_obj_t * btn_test_back;

static lv_obj_t * spinbox_test_frequency_1;

static lv_obj_t * btn_test_frequency_1_inc;
static lv_obj_t * btn_test_frequency_1_dec;

static lv_obj_t * cb_settings_input0;
static lv_obj_t * cb_settings_input1;

static lv_obj_t * btn_settings_back;

static lv_obj_t *edit_txtentry;
static lv_obj_t *edit_keyboard;

// touch screen callback
void my_touchpad_read( lv_indev_t * indev, lv_indev_data_t * data )
{
  uint16_t touchX, touchY;
  data->state = LV_INDEV_STATE_REL;
  if ( TFT.getTouch( &touchX, &touchY ) )
  {
    data->state = LV_INDEV_STATE_PR;
    if (touchX > 479 || touchY > 319)
    {
    }
    else
    {
      data->point.x = touchX;
      data->point.y = touchY;
    }
  }
}


// encoder interrupts
void IRAM_ATTR isr1() {
  unsigned char pinstate = (digitalRead(Encoder_1_Pin2) << 1) | digitalRead(Encoder_1_Pin1);
  Encoder_1_State = ttable[Encoder_1_State & 0xf][pinstate];
  unsigned char result = Encoder_1_State & 0x30;
  if (result == DIR_CW) {
    if (EncoderCounter1 < 10) {
      EncoderCounter1++;
    }
  } else if (result == DIR_CCW) {
    if (EncoderCounter1 > -10) {
      EncoderCounter1--;
    }
  }
}

void IRAM_ATTR isr2() {
  unsigned char pinstate = (digitalRead(Encoder_2_Pin2) << 1) | digitalRead(Encoder_2_Pin1);
  Encoder_2_State = ttable[Encoder_2_State & 0xf][pinstate];
  unsigned char result = Encoder_2_State & 0x30;
  if (result == DIR_CW) {
    if (EncoderCounter2 < 10) {
      EncoderCounter2++;
    }
  } else if (result == DIR_CCW) {
    if (EncoderCounter2 > -10) {
      EncoderCounter2--;
    }
  }
}

void IRAM_ATTR ptt_interrupt() {
  unsigned long t_now = millis(); // not perfect in ISR, but usable just to detect edges quickly
  // only register event if debounce period passed
  if (t_now - audio_ptt_debounce_timer > audio_ptt_debounce) {
    audio_ptt_debounce_timer = t_now;
    audio_ptt_active = !digitalRead(PTT_IN);
    if (audio_ptt_active != audio_ptt_active_last) {
      audio_ptt_active_last = audio_ptt_active;
      ptt_event_flag = true; // signal event for main loop
    }
  }
}

// use Arduinos millis() as tick source
static uint32_t my_tick(void)
{
  return millis();
}

void setup() {
  Serial.begin(115200); /* prepare for possible serial debug */

  // transmit button
  pinMode(PTT_IN, INPUT);

  // write protect pin eeprom dsp
  pinMode(EEPROM_WP_PIN, OUTPUT);
  digitalWrite(EEPROM_WP_PIN, HIGH);

  pinMode(Encoder_1_Pin1, INPUT);
  pinMode(Encoder_1_Pin2, INPUT);
  pinMode(Encoder_2_Pin1, INPUT);
  pinMode(Encoder_2_Pin2, INPUT);
  pinMode(Encoder_1_Key, INPUT);
  pinMode(Encoder_2_Key, INPUT);

  audio_scope_previous = (int8_t*)ps_malloc(512);

  Wire.begin();  // Initialize I2C
  dsp.begin();
  ee.begin();

  //    Serial.println(F("Pinging i2c lines...\n0 -> device is present\n2 -> device is not present"));
  //    Serial.print(F("DSP response: "));
  //    Serial.println(dsp.ping());
  //    Serial.print(F("EEPROM ping: "));
  //    Serial.println(ee.ping());


  // get current state to start otherwise encoder might not react to first rotation click
  unsigned char temppinstate1 = (digitalRead(Encoder_1_Pin2) << 1) | digitalRead(Encoder_1_Pin1);
  Encoder_1_State = ttable[Encoder_1_State & 0xf][temppinstate1];

  unsigned char temppinstate2 = (digitalRead(Encoder_2_Pin2) << 1) | digitalRead(Encoder_2_Pin1);
  Encoder_2_State = ttable[Encoder_2_State & 0xf][temppinstate2];

  TFT.init();
  TFT.setRotation(3);
  TFT.fillScreen(TFT_BLACK);

  lv_init();

  // set a tick source so that LVGL will know how much time elapsed
  lv_tick_set_cb(my_tick);

  // setup the two buffers
  draw_buf_1 = heap_caps_malloc(DRAW_BUF_SIZE, MALLOC_CAP_SPIRAM);
  draw_buf_2 = heap_caps_malloc(DRAW_BUF_SIZE, MALLOC_CAP_SPIRAM);

  disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);

  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_buffers(disp, draw_buf_1, draw_buf_2, DRAW_BUF_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touchpad_read);

  morseSprite.createSprite(MORSE_SPRITE_W, MORSE_SPRITE_H);
  morseSprite.setTextColor(TFT_WHITE, TFT_BLACK);

  // use a mutex for all dsp related communication since we have multiple tasks competing for I2C access
  i2cMutex = xSemaphoreCreateMutex();

  // clear screen touch calibration
  clear_settings();

  // check if EEPROM already contains the current firmware version
  uint8_t dsp_check_version = ee.getFirmwareVersion();
  if (dsp_check_version != dsp_firmware_version) {
    TFT.fillScreen(TFT_WHITE);
    TFT.setTextColor(TFT_RED, TFT_WHITE);
    TFT.setTextDatum(MC_DATUM);
    TFT.setTextSize(3);
    TFT.drawString("DSP firmware updating...", TFT.width() / 2, TFT.height() / 2);

    // disable write protect
    digitalWrite(EEPROM_WP_PIN, LOW);

    // the last parameter in writeFirmware is the FW version, and prevents the MCU from overwriting on every reboot

    ee.writeFirmware(DSP_eeprom_firmware, sizeof(DSP_eeprom_firmware), dsp_firmware_version);
    dsp.reset();
    delay(2000); // wait for the FW to load from the EEPROM
    digitalWrite(EEPROM_WP_PIN, HIGH);

    TFT.fillScreen(TFT_WHITE);
    TFT.setTextColor(TFT_GREEN, TFT_WHITE);
    TFT.drawString("Completed... cycle power.", TFT.width() / 2, TFT.height() / 2);

    while (1) {}
  }

  attachInterrupt(Encoder_1_Pin1, isr1, CHANGE);
  attachInterrupt(Encoder_1_Pin2, isr1, CHANGE);
  attachInterrupt(Encoder_2_Pin1, isr2, CHANGE);
  attachInterrupt(Encoder_2_Pin2, isr2, CHANGE);

  boolean forcecalibrate = false;
  // check if one of the buttons is pressed during boot
  if ((digitalRead(Encoder_1_Key) + digitalRead(Encoder_2_Key)) == 1) {
    // make sure its not a power on glitch due to capacitor charging on the encoder module
    delay(500);
    if ((digitalRead(Encoder_1_Key) + digitalRead(Encoder_2_Key)) == 1) {
      // call the calibration screen if one and only one of the encoder switches is pressed during power on
      forcecalibrate = true;
    }
  }
  if (!load_settings() or forcecalibrate) {
    calibratescreen();
  } else {
    TFT.setTouchCalibrate(audio_setting_calibration_data);
  }

  //  // factory defaults on both encoders clicked
  //  if (digitalRead(Encoder_1_Key) == 0 and digitalRead(Encoder_2_Key) == 0) {
  //    // make sure its not a power on glitch due to capacitor caharging on the encoder module
  //    delay(500);
  //    if (digitalRead(Encoder_1_Key) == 0 and digitalRead(Encoder_2_Key) == 0) {
  //      reset_settings();
  //    }
  //  }

  Display_Splash_Screen();

  delay(2000);

  Setup_Screens();

  // this has to be done after the setup since declaring the variables does not create a pointer to them
  preset_checkboxes[0] = cb_presets_selection0;
  preset_checkboxes[1] = cb_presets_selection1;
  preset_checkboxes[2] = cb_presets_selection2;
  preset_checkboxes[3] = cb_presets_selection3;
  preset_checkboxes[4] = cb_presets_selection4;
  preset_checkboxes[5] = cb_presets_selection5;
  preset_checkboxes[6] = cb_presets_selection6;
  preset_checkboxes[7] = cb_presets_selection7;
  preset_checkboxes[8] = cb_presets_selection8;
  preset_checkboxes[9] = cb_presets_selection9;

  preset_mainbuttons[0] = lbl_presetsmain_preset0;
  preset_mainbuttons[1] = lbl_presetsmain_preset1;
  preset_mainbuttons[2] = lbl_presetsmain_preset2;
  preset_mainbuttons[3] = lbl_presetsmain_preset3;
  preset_mainbuttons[4] = lbl_presetsmain_preset4;
  preset_mainbuttons[5] = lbl_presetsmain_preset5;
  preset_mainbuttons[6] = lbl_presetsmain_preset6;
  preset_mainbuttons[7] = lbl_presetsmain_preset7;
  preset_mainbuttons[8] = lbl_presetsmain_preset8;
  preset_mainbuttons[9] = lbl_presetsmain_preset9;

  //// 2nd order equalizer typedef
  //typedef struct secondOrderEQ_t
  //{
  //  float Q            = 1.41; // Parametric, Peaking, range 0-16
  //  float boost        = 0.0;  // Range +/-15 [dB]
  //  float freq;                // Range 20-20000 [Hz]
  //  uint8_t filterType = parameters::filterType::peaking; // parameters::filterType::[type]
  //  uint8_t phase      = parameters::phase::deg_0;        // parameters::phase::deg_0/deg_180
  //  uint8_t state      = parameters::state::on;           // parameters::state::on/off
  //} secondOrderEQ;

  // initialise EQ bands
  eqBand0.freq = 100;
  eqBand0.Q = 0.8;
  eqBand0.boost = 0;

  eqBand1.freq = 150;
  eqBand1.Q = 1.2;
  eqBand1.boost = 0;

  eqBand2.freq = 220;
  eqBand2.Q = 1.5;
  eqBand2.boost = 0;

  eqBand3.freq = 300;
  eqBand3.Q = 1.7;
  eqBand3.boost = 0;

  eqBand4.freq = 430;
  eqBand4.Q = 1.8;
  eqBand4.boost = 0;

  eqBand5.freq = 600;
  eqBand5.Q = 2.0;
  eqBand5.boost = 0;

  eqBand6.freq = 850;
  eqBand6.Q = 2.2;
  eqBand6.boost = 0;

  eqBand7.freq = 1200;
  eqBand7.Q = 2.5;
  eqBand7.boost = 0;

  eqBand8.freq = 1700;
  eqBand8.Q = 2.7;
  eqBand8.boost = 0;

  eqBand9.freq = 2400;
  eqBand9.Q = 2.8;
  eqBand9.boost = 0;

  eqBand10.freq = 3300;
  eqBand10.Q = 3.0;
  eqBand10.boost = 0;

  eqBand11.freq = 3800;
  eqBand11.Q = 3.2;
  eqBand11.boost = 0;

  // get first preset, apply is done automatically
  load_presets(0);



  // set all defaults that are not set already
  audio_selected_notch = 0;
  set_notch_labels(audio_selected_notch);
  set_notch_cursors(audio_selected_notch);

  audio_filter_mode_hi_lo = true;
  set_filter_cursors(audio_filter_mode_hi_lo);

  set_presets_selection_buttons_state();
  load_presets_names();
  set_presets_labels();
  set_presetsmain_labels();

  // normal connection
  set_esp32_mode(0);

  // there seems to be a serious loss at the input and AD convertion, therefor we compensate a bit
  set_input_gain(1.3);

  set_line_out_gain(1);

  // max is 16 per module, clipping above 4 (ca 2.5v out on analog output)
  set_hp_out_gain(4);

  set_sine1_on(false);
  lv_spinbox_set_value(spinbox_test_frequency_1, audio_test_frequency1);
  update_test_signal_gain();
  set_test_signal_gain();

  // unmute input
  mute_input(false);

  show_tx_led(false);

  select_hi_lo_filters(audio_filter_mode);

  morse_setlabel();

  set_main_labels(0);
  set_main_labels(1);

  set_compressor_labels(0);
  set_compressor_labels(1);

  set_eq_labels(0);

  set_limiter();

  // I2S Configuration, some fixes required for S3 module
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX | I2S_MODE_TX), // RX and TX modes
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = BLOCK_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = true, // Clear TX descriptor on underflow
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = PIN_BCK,
    .ws_io_num = PIN_WS,
    .data_out_num = PIN_DATA_OUT,
    .data_in_num = PIN_DATA_IN
  };

  esp_err_t error_code;

  error_code = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);

  if (error_code != 0) {
    Serial.println("i2s install error");
    Serial.println(error_code);
  }

  error_code = i2s_set_pin(I2S_NUM, &pin_config);

  if (error_code != 0) {
    Serial.println("i2s set error");
    Serial.println(error_code);
  }

  // start all timers

  audio_vu_update_timer = millis() - 200;
  audio_scope_timer = millis() - 200;

  key1_timer = millis();
  key2_timer = millis();

  LVGL_Timer = millis() - 200;

  lastTickMillis = millis();

  // ptt button intercept
  attachInterrupt(PTT_IN, ptt_interrupt, CHANGE);

  // start FreeRTOS tasks, balance between the cores
  xTaskCreatePinnedToCore(i2s_reader_task, "I2S Reader", 4096, NULL, 1, NULL, 0);

  xTaskCreatePinnedToCore(morse_task, "Morse Processor", 8192, NULL, 1, NULL, 1);

  start_lms_task(false, false, false);

}

void loop() {

  // ptt logic
  // when tx active, mute input
  if (ptt_event_flag) {
    ptt_event_flag = false;
    mute_input(audio_ptt_active);
    if (audio_input_muted) {
      // clear vu meter
      if ((lv_tabview_get_tab_act(tabview) == TAB_MAIN_REF or lv_tabview_get_tab_act(tabview) == TAB_COMPRESSOR_REF or lv_tabview_get_tab_act(tabview) == TAB_MORSE_REF)) {
        audio_vu_meter = 0;
        VU_meter_update();
      }
      // clear scope
      if (lv_tabview_get_tab_act(tabview) == TAB_SCOPE_REF) {
        scope_clear();
      }
      // clear spectrum
      if (lv_tabview_get_tab_act(tabview) == TAB_SPECTRUM_REF) {
        spectrum_clear();
      }
      show_tx_led(true);
    }
    else {
      show_tx_led(false);
    }
  }

  // check encoder buttons
  if (digitalRead(Encoder_1_Key) == 0) {
    timer_encoderbutton1 = millis();
    Encoder_Key1_Long_Press = false;
    // wait until key is no longer pressed or time expired
    while (digitalRead(Encoder_1_Key) == 0) {
      if (millis() - timer_encoderbutton1 > 1000) {
        Encoder_Key1_Long_Press = true;
        break;
      }
    }
    if (Encoder_Key1_Long_Press) {
      // long press
      while (digitalRead(Encoder_1_Key) == 0) {}
      //little debounce
      delay(200);
      // actions

    }
    else {
      // short press
      switch (lv_tabview_get_tab_act(tabview)) {
        case TAB_MAIN_REF:
          // main screen
          audio_rotary_selected_gain = !audio_rotary_selected_gain;
          set_main_labels(0);
          break;

        case TAB_NOTCH_REF:
          // notch screen
          if (++audio_selected_notch > 2) {
            audio_selected_notch = 0;
          }
          set_notch_labels(audio_selected_notch);
          set_notch_cursors(audio_selected_notch);
          break;

        case TAB_FILTER_REF:
          // filter screen
          audio_filter_mode_hi_lo = !audio_filter_mode_hi_lo;
          set_filter_cursors(audio_filter_mode_hi_lo);
          break;

        case TAB_PEAK_REF:
          checkbox_toggle(cb_peak_on);
          // force check since LV_EVENT_CLICKED does not pick it up and LV_EVENT_VALUE_CHANGED is triggered multiple times
          lv_obj_send_event(cb_peak_on, LV_EVENT_CLICKED, NULL);
          break;

        case TAB_DYNAMIC_REF:
          // dynamic screen
          checkbox_toggle(cb_dynamic_agc);
          // force check since LV_EVENT_CLICKED does not pick it up and LV_EVENT_VALUE_CHANGED is triggered multiple times
          lv_obj_send_event(cb_dynamic_agc, LV_EVENT_CLICKED, NULL);
          break;

        case TAB_COMPRESSOR_REF:
          // compressor screen
          audio_rotary_selected_threshold = !audio_rotary_selected_threshold;
          set_compressor_labels(0);
          break;

        case TAB_SCOPE_REF:
          // scope screen
          audio_scope_autoscale = true;
          break;

        case TAB_SPECTRUM_REF:
          // spectrum screen
          audio_spectrum_peaktrigger = true;
          break;

        case TAB_MORSE_REF:
          // morse screen
          // reset filter to 1000Hz
          lv_spinbox_set_value(spinbox_filter_bandpass, 1000);
          morse_setlabel();
          break;

        case TAB_TEST_REF:
          // test screen
          checkbox_toggle(cb_test_on);
          // force check since LV_EVENT_CLICKED does not pick it up and LV_EVENT_VALUE_CHANGED is triggered multiple times
          test_switch_on_off();
          break;

      }
    }
  }

  if (digitalRead(Encoder_2_Key) == 0) {
    timer_encoderbutton2 = millis();
    Encoder_Key2_Long_Press = false;
    // wait until key is no longer pressed or time expired
    while (digitalRead(Encoder_2_Key) == 0) {
      if (millis() - timer_encoderbutton2 > 1000) {
        Encoder_Key2_Long_Press = true;
        break;
      }
    }
    if (Encoder_Key2_Long_Press) {
      // long press
      while (digitalRead(Encoder_2_Key) == 0) {}
      //little debounce
      delay(200);
    }
    else {
      // short press
      switch (lv_tabview_get_tab_act(tabview)) {
        case TAB_MAIN_REF:
          // main screen
          audio_rotary_selected_speaker = !audio_rotary_selected_speaker;
          set_main_labels(1);
          break;

        case TAB_NOTCH_REF:
          // notch screen
          switch (audio_selected_notch) {
            case 0:
              filter_notch_on0 = ! filter_notch_on0;
              if (filter_notch_on0) {
                lv_obj_add_state(cb_notch_on0, LV_STATE_CHECKED);
              }
              else {
                lv_obj_clear_state(cb_notch_on0, LV_STATE_CHECKED);
              }
              break;
            case 1:
              filter_notch_on1 = ! filter_notch_on1;
              if (filter_notch_on1) {
                lv_obj_add_state(cb_notch_on1, LV_STATE_CHECKED);
              }
              else {
                lv_obj_clear_state(cb_notch_on1, LV_STATE_CHECKED);
              }
              break;
            case 2:
              filter_notch_on2 = ! filter_notch_on2;
              if (filter_notch_on2) {
                lv_obj_add_state(cb_notch_on2, LV_STATE_CHECKED);
              }
              else {
                lv_obj_clear_state(cb_notch_on2, LV_STATE_CHECKED);
              }
              break;
          }
          break;

        case TAB_COMPRESSOR_REF:
          // compressor screen
          audio_rotary_selected_postgain = !audio_rotary_selected_postgain;
          set_compressor_labels(1);
          break;

        case TAB_MORSE_REF:
          // morse screen
          checkbox_toggle(cb_morse_on);
          // force check since LV_EVENT_CLICKED does not pick it up and LV_EVENT_VALUE_CHANGED is triggered multiple times
          lv_obj_send_event(cb_morse_on, LV_EVENT_CLICKED, NULL);
          break;

      }
    }
  }

  // check rotary encoders
  if (EncoderCounter1 != 0) {
    switch (lv_tabview_get_tab_act(tabview)) {
      case TAB_MAIN_REF:
        // main screen
        if (audio_rotary_selected_gain) {
          if (EncoderCounter1 > 0) {
            if (audio_gain < lv_slider_get_max_value(slider_main_gain)) {
              audio_gain++;
              update_gain();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (audio_gain > 0) {
                audio_gain--;
                update_gain();
              }
            }
          }
        }
        else {
          if (EncoderCounter1 > 0) {
            if (audio_line_out < lv_slider_get_max_value(slider_main_output)) {
              audio_line_out++;
              update_output();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (audio_line_out > 0) {
                audio_line_out--;
                update_output();
              }
            }
          }
        }
        EncoderCounter1 = 0;
        break;

      case TAB_NOTCH_REF:
        // notch screen
        if (EncoderCounter1 > 0) {
          inc_notch_frequency(audio_selected_notch);
        }
        else {
          if (EncoderCounter1 < 0) {
            dec_notch_frequency(audio_selected_notch);
          }
        }
        EncoderCounter1 = 0;
        break;

      case TAB_FILTER_REF:
        // filter screen
        if (EncoderCounter1 > 0) {
          if (audio_filter_mode_hi_lo) {
            lv_spinbox_increment(spinbox_filter_highpass);
          }
          else {
            lv_spinbox_increment(spinbox_filter_bandpass);
          }
        }
        else {
          if (EncoderCounter1 < 0) {
            if (audio_filter_mode_hi_lo) {
              lv_spinbox_decrement(spinbox_filter_highpass);
            }
            else {
              lv_spinbox_decrement(spinbox_filter_bandpass);
            }
          }
        }
        EncoderCounter1 = 0;
        break;

      case TAB_PEAK_REF:
        // peaking screen
        if (EncoderCounter1 > 0) {
          lv_spinbox_increment(spinbox_peak_bandpass);
        }
        else {
          if (EncoderCounter1 < 0) {
            lv_spinbox_decrement(spinbox_peak_bandpass);
          }
        }
        EncoderCounter1 = 0;
        break;

      case TAB_COMPRESSOR_REF:
        // compressor screen
        if (audio_rotary_selected_threshold) {
          if (EncoderCounter1 > 0) {
            if (audio_compressor_threshold < lv_slider_get_max_value(slider_compressor_threshold)) {
              audio_compressor_threshold++;
              update_compressor_threshold();
              audio_compressor_update();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (audio_compressor_threshold > 0) {
                audio_compressor_threshold--;
                update_compressor_threshold();
                audio_compressor_update();
              }
            }
          }
        }
        else {
          if (EncoderCounter1 > 0) {
            if (audio_compressor_ratio  < lv_slider_get_max_value(slider_compressor_ratio)) {
              audio_compressor_ratio ++;
              update_compressor_ratio();
              audio_compressor_update();
            }
          }
          else {
            if (EncoderCounter1 < 0) {
              if (audio_compressor_ratio  > 1) {
                audio_compressor_ratio --;
                update_compressor_ratio();
                audio_compressor_update();
              }
            }
          }
        }
        EncoderCounter1 = 0;
        break;

      case TAB_EQUALIZER_REF:
        // eq screen
        if (EncoderCounter1 > 0) {
          inc_eq_slider(audio_selected_eq);
        }
        else {
          if (EncoderCounter1 < 0) {
            dec_eq_slider(audio_selected_eq);
          }
        }
        EncoderCounter1 = 0;
        break;

      case TAB_SCOPE_REF:
        // scope screen
        if (EncoderCounter1 > 0) {
          if (audio_scope_scale < 70000 - (audio_scope_scale / 10)) {
            audio_scope_scale += audio_scope_scale / 10;
          }
        }
        else {
          if (EncoderCounter1 < 0) {
            if (audio_scope_scale > 120 + (audio_scope_scale / 10)) {
              audio_scope_scale -= (audio_scope_scale / 10);
            }
          }
        }
        EncoderCounter1 = 0;
        break;

      case TAB_SPECTRUM_REF:
        // spectrum screen
        if (EncoderCounter1 > 0) {
          if (audio_spectrum_cursor < 118) {
            audio_spectrum_cursor++;
            spectrum_update_cursor(audio_spectrum_cursor);
          }
        }
        else {
          if (EncoderCounter1 < 0) {
            if (audio_spectrum_cursor > 0) {
              audio_spectrum_cursor--;
              spectrum_update_cursor(audio_spectrum_cursor);
            }
          }
        }
        EncoderCounter1 = 0;
        break;

      case TAB_MORSE_REF:
        // morse screen
        // limit range
        // determine which spinbox to use
        lv_obj_t* active_spinbox;
        if (lv_obj_has_state(cb_peak_on, LV_STATE_CHECKED)) {
          active_spinbox = spinbox_peak_bandpass;
        } else {
          active_spinbox = spinbox_filter_bandpass;
        }
        filter_bandpass_freq = lv_spinbox_get_value(active_spinbox);
        if (EncoderCounter1 > 0) {
          if (filter_bandpass_freq < 1200) {
            lv_spinbox_increment(active_spinbox);
          }
        }
        else {
          if (EncoderCounter1 < 0) {
            if (filter_bandpass_freq > 400) {
              lv_spinbox_decrement(active_spinbox);
            }
          }
        }
        // since steps are selected in the filter screen, boundaries can be crossed so re-check
        filter_bandpass_freq = lv_spinbox_get_value(active_spinbox);
        if (filter_bandpass_freq > 1200) {
          lv_spinbox_set_value(active_spinbox, 1200);
        }
        if (filter_bandpass_freq < 400) {
          lv_spinbox_set_value(active_spinbox, 400);
        }
        morse_setlabel();
        EncoderCounter1 = 0;
        break;

      case TAB_TEST_REF:
        // test screen
        if (EncoderCounter1 > 0) {
          if (audio_test_frequency1 <= 15000 - lv_spinbox_get_step(spinbox_test_frequency_1)) {
            audio_test_frequency1 = audio_test_frequency1 + lv_spinbox_get_step(spinbox_test_frequency_1);
            lv_spinbox_set_value(spinbox_test_frequency_1, audio_test_frequency1);
            set_sine_frequency1(audio_test_frequency1);
          }
        }
        else {
          if (EncoderCounter1 < 0) {
            if (audio_test_frequency1 >= 20 + lv_spinbox_get_step(spinbox_test_frequency_1)) {
              audio_test_frequency1 = audio_test_frequency1 - lv_spinbox_get_step(spinbox_test_frequency_1);
              lv_spinbox_set_value(spinbox_test_frequency_1, audio_test_frequency1);
              set_sine_frequency1(audio_test_frequency1);
            }
          }
        }
        EncoderCounter1 = 0;
        break;

      default:
        // clear if other screens
        EncoderCounter1 = 0;
        break;
    }
  }

  if (EncoderCounter2 != 0) {
    switch (lv_tabview_get_tab_act(tabview)) {
      case TAB_MAIN_REF:
        // main screen
        if (audio_rotary_selected_speaker) {
          if (EncoderCounter2 > 0) {
            if (audio_speaker < lv_slider_get_max_value(slider_main_speaker)) {
              audio_speaker++;
              update_speaker();
            }
          }
          else {
            if (EncoderCounter2 < 0) {
              if (audio_speaker > 0) {
                audio_speaker--;
                update_speaker();
              }
            }
          }
        }
        else {
          if (EncoderCounter2 > 0) {
            if (audio_headphone < lv_slider_get_max_value(slider_main_headphone)) {
              audio_headphone++;
              update_headphone();
            }
          }
          else {
            if (EncoderCounter2 < 0) {
              if (audio_headphone > 0) {
                audio_headphone--;
                update_headphone();
              }
            }
          }
        }
        EncoderCounter2 = 0;
        break;

      case TAB_NOTCH_REF:
        // notch screen
        if (EncoderCounter2 > 0) {
          inc_notch_width(audio_selected_notch);
        }
        else {
          if (EncoderCounter2 < 0) {
            dec_notch_width(audio_selected_notch);
          }
        }
        EncoderCounter2 = 0;
        break;

      case TAB_FILTER_REF:
        // filter screen
        if (EncoderCounter2 > 0) {
          if (audio_filter_mode_hi_lo) {
            lv_spinbox_increment(spinbox_filter_lowpass);
          }
          else {
            lv_spinbox_increment(spinbox_filter_bandwidth);
          }
        }
        else {
          if (EncoderCounter2 < 0) {
            if (audio_filter_mode_hi_lo) {
              lv_spinbox_decrement(spinbox_filter_lowpass);
            }
            else {
              lv_spinbox_decrement(spinbox_filter_bandwidth);
            }
          }
        }
        EncoderCounter2 = 0;
        break;

      case TAB_PEAK_REF:
        // peaking screen
        if (EncoderCounter2 > 0) {
          lv_spinbox_increment(spinbox_peak_bandwidth);
        }
        else {
          if (EncoderCounter2 < 0) {
            lv_spinbox_decrement(spinbox_peak_bandwidth);
          }
        }
        EncoderCounter2 = 0;
        break;

      case TAB_COMPRESSOR_REF:
        // compressor screen
        if (audio_rotary_selected_postgain) {
          if (EncoderCounter2 > 0) {
            if (audio_compressor_postgain  < lv_slider_get_max_value(slider_compressor_postgain)) {
              audio_compressor_postgain ++;
              update_compressor_postgain();
              audio_compressor_update();
            }
          }
          else {
            if (EncoderCounter2 < 0) {
              if (audio_compressor_postgain  > 1) {
                audio_compressor_postgain --;
                update_compressor_postgain();
                audio_compressor_update();
              }
            }
          }
        }
        else {
          if (EncoderCounter2 > 0) {
            if (audio_compressor_noisegate  < lv_slider_get_max_value(slider_compressor_noisegate)) {
              audio_compressor_noisegate ++;
              update_compressor_noisegate();
              audio_noisegate_update();
            }
          }
          else {
            if (EncoderCounter2 < 0) {
              if (audio_compressor_noisegate  > 1) {
                audio_compressor_noisegate --;
                update_compressor_noisegate();
                audio_noisegate_update();
              }
            }
          }
        }
        EncoderCounter2 = 0;
        break;

      case TAB_EQUALIZER_REF:
        // eq screen
        if (EncoderCounter2 > 0) {
          if (audio_selected_eq < 11) {
            audio_selected_eq++;
            set_eq_labels(audio_selected_eq);
          }
        }
        else {
          if (EncoderCounter2 < 0) {
            if (audio_selected_eq > 0) {
              audio_selected_eq--;
              set_eq_labels(audio_selected_eq);
            }
          }
        }
        EncoderCounter2 = 0;
        break;

      case TAB_TEST_REF:
        // test screen
        if (EncoderCounter2 > 0) {
          if (audio_test_signal_gain < lv_slider_get_max_value(slider_test_signal_gain)) {
            audio_test_signal_gain++;
            update_test_signal_gain();
          }
        }
        else {
          if (EncoderCounter2 < 0) {
            if (audio_test_signal_gain > 0) {
              audio_test_signal_gain--;
              update_test_signal_gain();
            }
          }
        }
        EncoderCounter2 = 0;
        break;

      default:
        // clear if other screens
        EncoderCounter2 = 0;
        break;
    }
  }

  // other screen related activity
  switch (lv_tabview_get_tab_act(tabview)) {
    case TAB_SCOPE_REF:
      // scope screen timed update
      if (!audio_input_muted and audio_screen_update_done and millis() - audio_scope_timer > 20  and audio_scope_ready) {
        scope_sample_plot();
        audio_scope_timer = millis();
      }
      break;

    case TAB_SPECTRUM_REF:
      // spectrum screen timed update
      if (!audio_input_muted and audio_screen_update_done and millis() - audio_scope_timer > 20 and audio_scope_ready) {
        spectrum_sample();
        spectrum_show();
        audio_scope_timer = millis();
      }
      break;

    case TAB_MORSE_REF:
      // morse screen
      // update morse text area
      check_morse_buffer();
      // update morse ticker
      if (morse_ticker_changed) {
        morse_ticker_changed = false;
        draw_morse_ticker();
      }
      break;

  }

  // VU meter update
  if (lv_tabview_get_tab_act(tabview) == TAB_MAIN_REF or lv_tabview_get_tab_act(tabview) == TAB_COMPRESSOR_REF or lv_tabview_get_tab_act(tabview) == TAB_MORSE_REF) {
    if (!audio_input_muted) {
      if (millis() - audio_vu_update_timer > 50) {
        int8_t audio_vu_meter_previous = audio_vu_meter;
        vu_meter_read();
        if (audio_vu_meter != audio_vu_meter_previous) {
          VU_meter_update();
        }
        audio_vu_update_timer = millis();
      }
    }
  }

  // update screen, let lvgl do its thing
  if (millis() >= LVGL_Timer + 5) {
    lv_timer_handler();
    LVGL_Timer = millis();
    // direct draw activity to ensure screen is build before we draw
    // draw after screen is first drawn, scope
    if (!audio_screen_update_done and lv_tabview_get_tab_act(tabview) == TAB_SCOPE_REF) {
      scope_draw_screen(true);
    }
    // draw after screen is first drawn, spectrum
    if (!audio_screen_update_done and lv_tabview_get_tab_act(tabview) == TAB_SPECTRUM_REF) {
      spectrum_draw_screen(true);
      spectrum_update_cursor(audio_spectrum_cursor);
    }
    audio_screen_update_done = true;
  }
}

// splash screen
void Display_Splash_Screen(void) {
  lv_obj_set_style_bg_color (lv_scr_act(), lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );

  static lv_style_t label_style1;
  lv_style_set_text_color(&label_style1, lv_color_white());
  lv_style_set_text_font(&label_style1, &lv_font_montserrat_40);
  lv_style_set_text_font(&label_style1, &lv_font_montserrat_24);

  lv_obj_t *labelintro1 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro1, &label_style1, LV_PART_MAIN);
  lv_label_set_text(labelintro1, "DSP Audio Processor");
  lv_obj_align(labelintro1, LV_ALIGN_TOP_MID, 0, 10);

  static lv_style_t label_style2;
  lv_style_set_text_color(&label_style2, lv_palette_main(LV_PALETTE_BLUE));
  lv_style_set_text_font(&label_style2, &lv_font_montserrat_18);

  lv_obj_t *labelintro2 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro2, &label_style2, LV_PART_MAIN);
  lv_label_set_text(labelintro2, "At boot time press:");
  lv_obj_align(labelintro2, LV_ALIGN_BOTTOM_MID, 0, -130);

  lv_obj_t *labelintro3 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro3, &label_style2, LV_PART_MAIN);
  lv_label_set_text(labelintro3, "Any encoder button to calibrate LCD");
  lv_obj_align(labelintro3, LV_ALIGN_BOTTOM_MID, 0, -100);

  //  lv_obj_t *labelintro4 = lv_label_create(lv_scr_act());
  //  lv_obj_add_style(labelintro4, &label_style2, LV_PART_MAIN);
  //  lv_label_set_text(labelintro4, "Both encoder buttons to reset settings");
  //  lv_obj_align(labelintro4, LV_ALIGN_BOTTOM_MID, 0, -70);

  static lv_style_t label_style3;
  lv_style_set_text_color(&label_style3, lv_palette_main(LV_PALETTE_ORANGE));
  lv_style_set_text_font(&label_style3, &lv_font_montserrat_24);
  lv_obj_t *labelintro10 = lv_label_create(lv_scr_act());
  lv_obj_add_style(labelintro10, &label_style3, LV_PART_MAIN);
  lv_label_set_text(labelintro10, "(C) Jef Collin 2025");
  lv_obj_align(labelintro10, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_refr_now(NULL);
}

void Setup_Screens(void) {
  // general use non adressable objects
  lv_obj_t * label;
  lv_obj_t * btn;

  int32_t obj_height;

  // tabs
  tabview = lv_tabview_create(lv_scr_act());
  lv_tabview_set_tab_bar_size(tabview, 0);

  tab_main = lv_tabview_add_tab(tabview, "");
  tab_menu = lv_tabview_add_tab(tabview, "");
  tab_notch = lv_tabview_add_tab(tabview, "");
  tab_filter = lv_tabview_add_tab(tabview, "");
  tab_peak  = lv_tabview_add_tab(tabview, "");
  tab_compressor = lv_tabview_add_tab(tabview, "");
  tab_equalizer = lv_tabview_add_tab(tabview, "");
  tab_scope = lv_tabview_add_tab(tabview, "");
  tab_spectrum = lv_tabview_add_tab(tabview, "");
  tab_morse = lv_tabview_add_tab(tabview, "");
  tab_presets = lv_tabview_add_tab(tabview, "");
  tab_test = lv_tabview_add_tab(tabview, "");
  tab_settings = lv_tabview_add_tab(tabview, "");
  tab_edit = lv_tabview_add_tab(tabview, "");
  tab_dynamic = lv_tabview_add_tab(tabview, "");
  tab_presetsmain = lv_tabview_add_tab(tabview, "");

  lv_obj_clear_flag(tabview, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_main, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_menu, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_notch, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_filter, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_peak, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_compressor, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_equalizer, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_presets, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_scope, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_spectrum, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_morse, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_test, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_settings, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_edit, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_dynamic, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(tab_presetsmain, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_clear_flag(lv_tabview_get_content(tabview), LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_style_bg_color (tab_main, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_menu, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_notch, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_filter, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_peak, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_compressor, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_equalizer, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_presets, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_scope, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_test, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_settings, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_edit, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_spectrum, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_morse, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_dynamic, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );
  lv_obj_set_style_bg_color (tab_presetsmain, lv_color_black(), LV_PART_MAIN | LV_STATE_DEFAULT );

  lv_obj_set_style_bg_opa(tab_main, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_menu, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_notch, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_filter, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_peak, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_compressor, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_equalizer, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_presets, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_scope, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_test, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_settings, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_edit, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_spectrum, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_morse, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_dynamic, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(tab_presetsmain, LV_OPA_COVER , LV_PART_MAIN | LV_STATE_DEFAULT);

  // remove 16 pix padding
  lv_obj_set_style_pad_top(tab_main, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_main, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_main, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_main, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_menu, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_menu, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_menu, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_menu, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_notch, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_notch, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_notch, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_notch, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_filter, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_filter, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_filter, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_filter, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_peak, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_peak, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_peak, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_peak, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_compressor, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_compressor, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_compressor, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_compressor, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_equalizer, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_equalizer, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_equalizer, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_equalizer, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_presets, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_presets, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_presets, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_presets, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_scope, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_scope, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_scope, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_scope, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_test, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_test, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_test, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_test, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_settings, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_settings, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_settings, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_settings, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_edit, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_edit, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_edit, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_edit, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_spectrum, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_spectrum, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_spectrum, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_spectrum, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_morse, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_morse, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_morse, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_morse, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_dynamic, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_dynamic, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_dynamic, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_dynamic, 0, LV_PART_MAIN);

  lv_obj_set_style_pad_top(tab_presetsmain, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(tab_presetsmain, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_left(tab_presetsmain, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_right(tab_presetsmain, 0, LV_PART_MAIN);



  // object styles

  // label styles
  static lv_style_t label_style1;
  lv_style_set_text_color(&label_style1, lv_color_black());
  lv_style_set_text_font(&label_style1, &lv_font_montserrat_24);
  lv_style_set_text_align(&label_style1, LV_TEXT_ALIGN_LEFT);

  static lv_style_t label_style2;
  lv_style_set_text_color(&label_style2, lv_color_white());
  lv_style_set_text_font(&label_style2, &lv_font_montserrat_24);

  static lv_style_t label_style3;
  lv_style_set_text_color(&label_style3, lv_color_white());
  lv_style_set_text_font(&label_style3, &lv_font_montserrat_12);

  static lv_style_t label_style4;
  lv_style_set_text_color(&label_style4, lv_color_white());
  lv_style_set_text_font(&label_style4, &lv_font_montserrat_18);

  static lv_style_t label_style5;
  lv_style_set_text_color(&label_style5, lv_palette_main(LV_PALETTE_RED));
  lv_style_set_text_font(&label_style5, &lv_font_montserrat_24);

  static lv_style_t label_style6;
  lv_style_set_text_color(&label_style6, lv_color_white());
  lv_style_set_text_font(&label_style6, &lv_font_montserrat_24);
  lv_style_set_text_align(&label_style6, LV_TEXT_ALIGN_RIGHT);

  static lv_style_t label_style7;
  lv_style_set_text_color(&label_style7, lv_color_white());
  lv_style_set_text_font(&label_style7, &lv_font_montserrat_20);

  static lv_style_t label_style8;
  lv_style_set_text_color(&label_style8, lv_color_black());
  lv_style_set_text_font(&label_style8, &lv_font_montserrat_22);

  // button styles
  static lv_style_t btn_style1;
  lv_style_set_radius(&btn_style1, 3);
  lv_style_set_bg_color(&btn_style1, lv_color_white());
  lv_style_set_text_font(&btn_style1, &lv_font_montserrat_24);
  lv_style_set_text_color(&btn_style1, lv_color_black());
  lv_style_set_size(&btn_style1, 149, 50);

  static lv_style_t btn_style1_checked;
  lv_style_set_bg_color(&btn_style1_checked, lv_palette_main(LV_PALETTE_BLUE));
  lv_style_set_text_font(&btn_style1_checked, &lv_font_montserrat_24);
  lv_style_set_text_color(&btn_style1_checked, lv_color_black());

  static lv_style_t btn_style2;
  lv_style_init(&btn_style2);
  lv_style_set_radius(&btn_style2, 0);
  lv_style_set_text_font(&btn_style2, &lv_font_montserrat_20);
  lv_style_set_bg_color(&btn_style2, lv_color_white());
  lv_style_set_size(&btn_style2, 480, 31);

  static lv_style_t btn_label_style2;
  lv_style_init(&btn_label_style2);
  lv_style_set_text_font(&btn_label_style2, &lv_font_montserrat_20);
  lv_style_set_text_color(&btn_label_style2, lv_color_black());
  lv_style_set_align(&btn_label_style2, LV_ALIGN_LEFT_MID);
  lv_style_set_pad_left(&btn_label_style2, 10);


  // radio buttons style
  static lv_style_t radiobtn_style1;
  lv_style_init(&radiobtn_style1);
  lv_style_set_radius(&radiobtn_style1, LV_RADIUS_CIRCLE);
  lv_style_set_bg_color(&radiobtn_style1, lv_color_black());
  lv_style_set_text_font(&radiobtn_style1, &lv_font_montserrat_20);
  lv_style_set_text_color(&radiobtn_style1, lv_color_white());
  lv_style_set_text_opa(&radiobtn_style1, LV_OPA_COVER);
  lv_style_set_border_color(&radiobtn_style1, lv_color_white());

  static lv_style_t radiobtn_checked_style1;
  lv_style_init(&radiobtn_checked_style1);
  lv_style_set_bg_image_src(&radiobtn_checked_style1, NULL);
  lv_style_set_bg_color(&radiobtn_checked_style1, lv_color_white());

  // slider styles
  /*Create a transition*/
  static const lv_style_prop_t props[] = {LV_STYLE_BG_COLOR, 0};
  static lv_style_transition_dsc_t transition_dsc;
  lv_style_transition_dsc_init(&transition_dsc, props, lv_anim_path_linear, 300, 0, NULL);

  static lv_style_t slider_style1_main;
  static lv_style_t slider_style1_indicator;
  static lv_style_t slider_style1_knob;
  static lv_style_t slider_style1_pressed_color;
  lv_style_init(&slider_style1_main);
  lv_style_set_bg_opa(&slider_style1_main, LV_OPA_COVER);
  lv_style_set_bg_color(&slider_style1_main, lv_color_hex3(0xbbb));
  lv_style_set_radius(&slider_style1_main, LV_RADIUS_CIRCLE);
  lv_style_set_pad_ver(&slider_style1_main, -2); /*Makes the indicator larger*/

  lv_style_init(&slider_style1_indicator);
  lv_style_set_bg_opa(&slider_style1_indicator, LV_OPA_COVER);
  lv_style_set_bg_color(&slider_style1_indicator, lv_palette_main(LV_PALETTE_LIGHT_BLUE));
  lv_style_set_radius(&slider_style1_indicator, LV_RADIUS_CIRCLE);
  lv_style_set_transition(&slider_style1_indicator, &transition_dsc);

  lv_style_init(&slider_style1_knob);
  lv_style_set_bg_opa(&slider_style1_knob, LV_OPA_COVER);
  lv_style_set_bg_color(&slider_style1_knob, lv_palette_main(LV_PALETTE_LIGHT_BLUE));
  lv_style_set_border_color(&slider_style1_knob, lv_palette_main(LV_PALETTE_LIGHT_BLUE));
  lv_style_set_border_width(&slider_style1_knob, 2);
  lv_style_set_radius(&slider_style1_knob, LV_RADIUS_CIRCLE);
  lv_style_set_pad_all(&slider_style1_knob, 6); /*Makes the knob larger*/
  lv_style_set_transition(&slider_style1_knob, &transition_dsc);

  lv_style_init(&slider_style1_pressed_color);
  lv_style_set_bg_color(&slider_style1_pressed_color, lv_palette_main(LV_PALETTE_LIGHT_BLUE));

  // spinbox styles
  static lv_style_t spinbox_style1;

  lv_style_set_bg_color(&spinbox_style1, lv_color_white());
  lv_style_set_text_font(&spinbox_style1, &lv_font_montserrat_20);
  lv_style_set_text_color(&spinbox_style1, lv_color_black());

  static lv_style_t spinbox_style2;

  lv_style_set_bg_color(&spinbox_style2, lv_color_white());
  lv_style_set_text_font(&spinbox_style2, &lv_font_montserrat_20);
  lv_style_set_text_color(&spinbox_style2, lv_color_black());
  lv_style_set_text_align(&spinbox_style2, LV_TEXT_ALIGN_CENTER);

  // checkbox style
  static lv_style_t cb_style1;
  lv_style_init(&cb_style1);
  // lv_style_set_radius(&cb_style1, LV_RADIUS_CIRCLE);
  lv_style_set_bg_color(&cb_style1, lv_color_black());
  lv_style_set_text_font(&cb_style1, &lv_font_montserrat_24);
  lv_style_set_text_color(&cb_style1, lv_color_white());
  lv_style_set_text_opa(&cb_style1, LV_OPA_COVER);

  // line styles
  static lv_style_t style_line;
  lv_style_init(&style_line);
  lv_style_set_line_width(&style_line, 2);
  lv_style_set_line_color(&style_line, lv_palette_main(LV_PALETTE_GREEN));
  lv_style_set_line_rounded(&style_line, true);

  static lv_style_t style_line2;
  lv_style_init(&style_line2);
  lv_style_set_line_width(&style_line2, 6);
  lv_style_set_line_color(&style_line2, lv_palette_main(LV_PALETTE_GREEN));
  lv_style_set_line_rounded(&style_line2, true);

  // dropdown style
  static lv_style_t dropdown_style1;
  lv_style_set_text_color(&dropdown_style1, lv_color_black());
  lv_style_set_text_font(&dropdown_style1, &lv_font_montserrat_24);

  // main screen

  lv_obj_t * vuframe1 = lv_obj_create(tab_main);
  lv_obj_set_size(vuframe1, 29, 304);
  lv_obj_align(vuframe1, LV_ALIGN_TOP_LEFT, 0, 8);
  lv_obj_set_style_bg_opa(vuframe1, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_radius(vuframe1, 0, LV_PART_MAIN);
  lv_obj_clear_flag(vuframe1, LV_OBJ_FLAG_SCROLLABLE);

  slider_main_gain = lv_slider_create(tab_main);
  lv_obj_remove_style_all(slider_main_gain);
  lv_obj_add_style(slider_main_gain, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_main_gain, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_main_gain, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_main_gain, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_main_gain, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_main_gain, 15, 200);
  lv_slider_set_range(slider_main_gain, 0, 50);
  lv_obj_align(slider_main_gain, LV_ALIGN_TOP_LEFT, 79, 60);
  lv_obj_add_event_cb(slider_main_gain, event_gain_change, LV_EVENT_ALL, NULL);

  lbl_main_gain = lv_label_create(tab_main);
  lv_label_set_text(lbl_main_gain, "IN");
  lv_obj_add_style(lbl_main_gain, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_main_gain, slider_main_gain, LV_ALIGN_OUT_TOP_MID, 0, -25);

  slider_main_output = lv_slider_create(tab_main);
  lv_obj_remove_style_all(slider_main_output);
  lv_obj_add_style(slider_main_output, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_main_output, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_main_output, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_main_output, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_main_output, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_main_output, 15, 200);
  lv_slider_set_range(slider_main_output, 0, 50);
  lv_obj_align(slider_main_output, LV_ALIGN_TOP_LEFT, 144, 60);
  lv_obj_add_event_cb(slider_main_output, event_output_change, LV_EVENT_ALL, NULL);

  lbl_main_output = lv_label_create(tab_main);
  lv_label_set_text(lbl_main_output, "LO");
  lv_obj_add_style(lbl_main_output, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_main_output, slider_main_output, LV_ALIGN_OUT_TOP_MID, 0, -25);

  slider_main_speaker = lv_slider_create(tab_main);
  lv_obj_remove_style_all(slider_main_speaker);
  lv_obj_add_style(slider_main_speaker, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_main_speaker, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_main_speaker, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_main_speaker, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_main_speaker, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_main_speaker, 15, 200);
  lv_slider_set_range(slider_main_speaker, 0, 50);
  lv_obj_align(slider_main_speaker, LV_ALIGN_TOP_LEFT, 209, 60);
  lv_obj_add_event_cb(slider_main_speaker, event_speaker_change, LV_EVENT_ALL, NULL);

  lbl_main_speaker = lv_label_create(tab_main);
  lv_label_set_text(lbl_main_speaker, "SP");
  lv_obj_add_style(lbl_main_speaker, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_main_speaker, slider_main_speaker, LV_ALIGN_OUT_TOP_MID, 0, -25);


  slider_main_headphone = lv_slider_create(tab_main);
  lv_obj_remove_style_all(slider_main_headphone);
  lv_obj_add_style(slider_main_headphone, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_main_headphone, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_main_headphone, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_main_headphone, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_main_headphone, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_main_headphone, 15, 200);
  lv_slider_set_range(slider_main_headphone, 0, 50);
  lv_obj_align(slider_main_headphone, LV_ALIGN_TOP_LEFT, 274, 60);
  lv_obj_add_event_cb(slider_main_headphone, event_headphone_change, LV_EVENT_ALL, NULL);

  lbl_main_headphone = lv_label_create(tab_main);
  lv_label_set_text(lbl_main_headphone, "HP");
  lv_obj_add_style(lbl_main_headphone, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_main_headphone, slider_main_headphone, LV_ALIGN_OUT_TOP_MID, 0, -25);

  led_main_tx = lv_led_create(tab_main);
  lv_obj_align(led_main_tx, LV_ALIGN_TOP_LEFT, 331, 50);
  lv_led_set_brightness(led_main_tx, 150);
  lv_led_set_color(led_main_tx, lv_palette_main(LV_PALETTE_RED));
  lv_led_on(led_main_tx);

  lbl_main_tx = lv_label_create(tab_main);
  lv_label_set_text(lbl_main_tx, "TX");
  lv_obj_add_style(lbl_main_tx, &label_style5, LV_PART_MAIN);

  lv_obj_align_to(lbl_main_tx, led_main_tx, LV_ALIGN_OUT_RIGHT_MID, 5, 0);

  btn_main_preset = lv_btn_create(tab_main);
  lv_obj_add_style(btn_main_preset, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_main_preset, event_presetsmainscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_main_preset);
  lv_label_set_text(label, "Presets");
  lv_obj_center(label);
  lv_obj_align(btn_main_preset, LV_ALIGN_TOP_RIGHT, 0, 201);

  btn_main_menu = lv_btn_create(tab_main);
  lv_obj_add_style(btn_main_menu, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_main_menu, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_main_menu);
  lv_label_set_text(label, "Menu");
  lv_obj_center(label);
  lv_obj_align(btn_main_menu, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab menu

  lv_obj_t * btn50 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn50, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn50, event_notchscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn50);
  lv_label_set_text(label, "Notch");
  lv_obj_center(label);
  lv_obj_align(btn50, LV_ALIGN_TOP_LEFT, 0, 0);

  lv_obj_t * btn51 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn51, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn51, event_filterscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn51);
  lv_label_set_text(label, "Filter");
  lv_obj_center(label);
  lv_obj_align(btn51, LV_ALIGN_TOP_LEFT, 0, 67);

  lv_obj_t * btn52 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn52, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn52, event_peakscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn52);
  lv_label_set_text(label, "Peaking");
  lv_obj_center(label);
  lv_obj_align(btn52, LV_ALIGN_TOP_LEFT, 0, 134);

  lv_obj_t * btn53 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn53, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn53, event_dynamicscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn53);
  lv_label_set_text(label, "Dynamic");
  lv_obj_center(label);
  lv_obj_align(btn53, LV_ALIGN_TOP_LEFT, 0, 201);

  lv_obj_t * btn55 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn55, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn55, event_compressorscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn55);
  lv_label_set_text(label, "Compressor");
  lv_obj_center(label);
  lv_obj_align(btn55, LV_ALIGN_TOP_MID, 0, 0);

  lv_obj_t * btn56 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn56, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn56, event_eqscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn56);
  lv_label_set_text(label, "Equalizer");
  lv_obj_center(label);
  lv_obj_align(btn56, LV_ALIGN_TOP_MID, 0, 67);

  lv_obj_t * btn57 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn57, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn57, event_morsescr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn57);
  lv_label_set_text(label, "Morse");
  lv_obj_center(label);
  lv_obj_align(btn57, LV_ALIGN_TOP_MID, 0, 134);

  lv_obj_t * btn59 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn59, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn59, event_presetsscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn59);
  lv_label_set_text(label, "Presets");
  lv_obj_center(label);
  lv_obj_align(btn59, LV_ALIGN_TOP_MID, 0, 268);

  lv_obj_t * btn60 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn60, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn60, event_scopescr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn60);
  lv_label_set_text(label, "Scope");
  lv_obj_center(label);
  lv_obj_align(btn60, LV_ALIGN_TOP_RIGHT, 0, 0);

  lv_obj_t * btn61 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn61, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn61, event_spectrumscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn61);
  lv_label_set_text(label, "Spectrum");
  lv_obj_center(label);
  lv_obj_align(btn61, LV_ALIGN_TOP_RIGHT, 0, 67);

  lv_obj_t * btn62 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn62, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn62, event_testscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn62);
  lv_label_set_text(label, "Test");
  lv_obj_center(label);
  lv_obj_align(btn62, LV_ALIGN_TOP_RIGHT, 0, 134);

  lv_obj_t * btn63 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn63, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn63, event_settingsscreen, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn63);
  lv_label_set_text(label, "Settings");
  lv_obj_center(label);
  lv_obj_align(btn63, LV_ALIGN_TOP_RIGHT, 0, 201);

  lv_obj_t * btn64 = lv_btn_create(tab_menu);
  lv_obj_add_style(btn64, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn64, event_mainscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn64);
  lv_label_set_text(label, LV_SYMBOL_HOME);
  lv_obj_center(label);
  lv_obj_align(btn64, LV_ALIGN_TOP_RIGHT, 0, 268);


  // tab notch filters

  label = lv_label_create(tab_notch);
  lv_obj_add_style(label, &label_style7, LV_PART_MAIN);
  lv_label_set_text(label, "#");
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 5);

  label = lv_label_create(tab_notch);
  lv_obj_add_style(label, &label_style7, LV_PART_MAIN);
  lv_label_set_text(label, "Active");
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 19, 5);

  static lv_point_precise_t line_pointsct[] = { {0, 0}, {29, 0}, {44, 39}, {74, 39}, {89, 0}, {119, 0} };
  static lv_point_precise_t line_pointsctct[] = { {59, 39}, {59, 35} };
  static lv_point_precise_t line_pointsnbw[] = { {41, 19}, {77, 19} };
  static lv_point_precise_t line_pointsnbwl[] = { {37, 19}, {40, 19} };
  static lv_point_precise_t line_pointsnbwr[] = { {78, 19}, {81, 19} };

  lv_obj_t * notch_line1;
  notch_line1 = lv_line_create(tab_notch);
  lv_line_set_points(notch_line1, line_pointsct, 6);
  lv_obj_add_style(notch_line1, &style_line, 0);
  lv_obj_align(notch_line1, LV_ALIGN_TOP_LEFT, 127, 0);

  lv_obj_t * notch_line2;
  notch_line2 = lv_line_create(tab_notch);
  lv_line_set_points(notch_line2, line_pointsctct, 2);
  lv_obj_add_style(notch_line2, &style_line2, 0);
  lv_obj_align(notch_line2, LV_ALIGN_TOP_LEFT, 127, 0);

  lv_obj_t * notch_line3;
  notch_line3 = lv_line_create(tab_notch);
  lv_line_set_points(notch_line3, line_pointsct, 6);
  lv_obj_add_style(notch_line3, &style_line, 0);
  lv_obj_align(notch_line3, LV_ALIGN_TOP_LEFT, 332, 0);

  lv_obj_t * notch_line4;
  notch_line4 = lv_line_create(tab_notch);
  lv_line_set_points(notch_line4, line_pointsnbw, 2);
  lv_obj_add_style(notch_line4, &style_line, 0);
  lv_obj_align(notch_line4, LV_ALIGN_TOP_LEFT, 332, 0);

  lv_obj_t * notch_line5;
  notch_line5 = lv_line_create(tab_notch);
  lv_line_set_points(notch_line5, line_pointsnbwl, 2);
  lv_obj_add_style(notch_line5, &style_line2, 0);
  lv_obj_align(notch_line5, LV_ALIGN_TOP_LEFT, 332, 0);

  lv_obj_t * notch_line6;
  notch_line6 = lv_line_create(tab_notch);
  lv_line_set_points(notch_line6, line_pointsnbwr, 2);
  lv_obj_add_style(notch_line6, &style_line2, 0);
  lv_obj_align(notch_line6, LV_ALIGN_TOP_LEFT, 332, 0);

  lbl_notch0 = lv_label_create(tab_notch);
  lv_label_set_text(lbl_notch0, "1");
  lv_obj_add_style(lbl_notch0, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_notch0, LV_ALIGN_TOP_LEFT, 0, 60);

  lbl_notch1 = lv_label_create(tab_notch);
  lv_label_set_text(lbl_notch1, "2");
  lv_obj_add_style(lbl_notch1, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_notch1, LV_ALIGN_TOP_LEFT, 0, 130);

  lbl_notch2 = lv_label_create(tab_notch);
  lv_label_set_text(lbl_notch2, "3");
  lv_obj_add_style(lbl_notch2, &label_style2, LV_PART_MAIN);
  lv_obj_align(lbl_notch2, LV_ALIGN_TOP_LEFT, 0, 200);

  cb_notch_on0 = lv_checkbox_create(tab_notch);
  lv_checkbox_set_text(cb_notch_on0, "");
  lv_obj_align(cb_notch_on0, LV_ALIGN_TOP_LEFT, 34, 60);
  lv_obj_add_style(cb_notch_on0, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_notch_on0, event_notch_switch_0, LV_EVENT_ALL, NULL);

  cb_notch_on1 = lv_checkbox_create(tab_notch);
  lv_checkbox_set_text(cb_notch_on1, "");
  lv_obj_align(cb_notch_on1, LV_ALIGN_TOP_LEFT, 34, 130);
  lv_obj_add_style(cb_notch_on1, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_notch_on1, event_notch_switch_1, LV_EVENT_ALL, NULL);

  cb_notch_on2 = lv_checkbox_create(tab_notch);
  lv_checkbox_set_text(cb_notch_on2, "");
  lv_obj_align(cb_notch_on2, LV_ALIGN_TOP_LEFT, 34, 200);
  lv_obj_add_style(cb_notch_on2, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_notch_on2, event_notch_switch_2, LV_EVENT_ALL, NULL);

  spinbox_notch_frequency_0 = lv_spinbox_create(tab_notch);
  lv_spinbox_set_range(spinbox_notch_frequency_0, 100, 4000);
  lv_spinbox_set_digit_format(spinbox_notch_frequency_0, 4, 0);

  lv_spinbox_set_step(spinbox_notch_frequency_0, 1);
  lv_obj_set_width(spinbox_notch_frequency_0, 75);
  lv_obj_set_height(spinbox_notch_frequency_0, 46);

  lv_obj_align(spinbox_notch_frequency_0, LV_ALIGN_TOP_LEFT, 150, 55);
  lv_obj_add_style(spinbox_notch_frequency_0, &spinbox_style1, LV_PART_MAIN);

  lv_obj_add_event_cb(spinbox_notch_frequency_0, event_notch_frequency0_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_notch_frequency_0);

  btn_notch_frequency_0_inc = lv_button_create(tab_notch);
  lv_obj_set_size(btn_notch_frequency_0_inc, obj_height, obj_height);
  lv_obj_align_to(btn_notch_frequency_0_inc, spinbox_notch_frequency_0, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_notch_frequency_0_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_notch_frequency_0_inc, increment_notch_frequency0_event_cb, LV_EVENT_ALL,  NULL);

  btn_notch_frequency_0_dec = lv_button_create(tab_notch);
  lv_obj_set_size(btn_notch_frequency_0_dec, obj_height, obj_height);
  lv_obj_align_to(btn_notch_frequency_0_dec, spinbox_notch_frequency_0, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_notch_frequency_0_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_notch_frequency_0_dec, decrement_notch_frequency0_event_cb, LV_EVENT_ALL, NULL);

  spinbox_notch_frequency_1 = lv_spinbox_create(tab_notch);
  lv_spinbox_set_range(spinbox_notch_frequency_1, 100, 4000);
  lv_spinbox_set_digit_format(spinbox_notch_frequency_1, 4, 0);

  lv_spinbox_set_step(spinbox_notch_frequency_1, 1);
  lv_obj_set_width(spinbox_notch_frequency_1, 75);
  lv_obj_set_height(spinbox_notch_frequency_1, 46);

  lv_obj_align(spinbox_notch_frequency_1, LV_ALIGN_TOP_LEFT, 150, 125);
  lv_obj_add_style(spinbox_notch_frequency_1, &spinbox_style1, LV_PART_MAIN);

  lv_obj_add_event_cb(spinbox_notch_frequency_1, event_notch_frequency1_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_notch_frequency_1);

  btn_notch_frequency_1_inc = lv_button_create(tab_notch);
  lv_obj_set_size(btn_notch_frequency_1_inc, obj_height, obj_height);
  lv_obj_align_to(btn_notch_frequency_1_inc, spinbox_notch_frequency_1, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_notch_frequency_1_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_notch_frequency_1_inc, increment_notch_frequency1_event_cb, LV_EVENT_ALL,  NULL);

  btn_notch_frequency_1_dec = lv_button_create(tab_notch);
  lv_obj_set_size(btn_notch_frequency_1_dec, obj_height, obj_height);
  lv_obj_align_to(btn_notch_frequency_1_dec, spinbox_notch_frequency_1, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_notch_frequency_1_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_notch_frequency_1_dec, decrement_notch_frequency1_event_cb, LV_EVENT_ALL, NULL);

  spinbox_notch_frequency_2 = lv_spinbox_create(tab_notch);
  lv_spinbox_set_range(spinbox_notch_frequency_2, 100, 4000);
  lv_spinbox_set_digit_format(spinbox_notch_frequency_2, 4, 0);

  lv_spinbox_set_step(spinbox_notch_frequency_2, 1);
  lv_obj_set_width(spinbox_notch_frequency_2, 75);
  lv_obj_set_height(spinbox_notch_frequency_2, 46);

  lv_obj_align(spinbox_notch_frequency_2, LV_ALIGN_TOP_LEFT, 150, 195);
  lv_obj_add_style(spinbox_notch_frequency_2, &spinbox_style1, LV_PART_MAIN);

  lv_obj_add_event_cb(spinbox_notch_frequency_2, event_notch_frequency2_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_notch_frequency_2);

  btn_notch_frequency_2_inc = lv_button_create(tab_notch);
  lv_obj_set_size(btn_notch_frequency_2_inc, obj_height, obj_height);
  lv_obj_align_to(btn_notch_frequency_2_inc, spinbox_notch_frequency_2, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_notch_frequency_2_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_notch_frequency_2_inc, increment_notch_frequency2_event_cb, LV_EVENT_ALL,  NULL);

  btn_notch_frequency_2_dec = lv_button_create(tab_notch);
  lv_obj_set_size(btn_notch_frequency_2_dec, obj_height, obj_height);
  lv_obj_align_to(btn_notch_frequency_2_dec, spinbox_notch_frequency_2, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_notch_frequency_2_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_notch_frequency_2_dec, decrement_notch_frequency2_event_cb, LV_EVENT_ALL, NULL);

  spinbox_notch_width_0 = lv_spinbox_create(tab_notch);
  lv_spinbox_set_range(spinbox_notch_width_0, 5, 300);
  lv_spinbox_set_digit_format(spinbox_notch_width_0, 3, 0);

  lv_spinbox_set_step(spinbox_notch_width_0, 1);
  lv_obj_set_width(spinbox_notch_width_0, 65);
  lv_obj_set_height(spinbox_notch_width_0, 46);

  lv_obj_align(spinbox_notch_width_0, LV_ALIGN_TOP_LEFT, 360, 55);
  lv_obj_add_style(spinbox_notch_width_0, &spinbox_style1, LV_PART_MAIN);

  lv_obj_add_event_cb(spinbox_notch_width_0, event_notch_width0_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_notch_width_0);

  btn_notch_width_0_inc = lv_button_create(tab_notch);
  lv_obj_set_size(btn_notch_width_0_inc, obj_height, obj_height);
  lv_obj_align_to(btn_notch_width_0_inc, spinbox_notch_width_0, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_notch_width_0_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_notch_width_0_inc, increment_notch_width0_event_cb, LV_EVENT_ALL,  NULL);

  btn_notch_width_0_dec = lv_button_create(tab_notch);
  lv_obj_set_size(btn_notch_width_0_dec, obj_height, obj_height);
  lv_obj_align_to(btn_notch_width_0_dec, spinbox_notch_width_0, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_notch_width_0_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_notch_width_0_dec, decrement_notch_width0_event_cb, LV_EVENT_ALL, NULL);

  spinbox_notch_width_1 = lv_spinbox_create(tab_notch);
  lv_spinbox_set_range(spinbox_notch_width_1, 5, 300);
  lv_spinbox_set_digit_format(spinbox_notch_width_1, 3, 0);

  lv_spinbox_set_step(spinbox_notch_width_1, 1);
  lv_obj_set_width(spinbox_notch_width_1, 65);
  lv_obj_set_height(spinbox_notch_width_1, 46);

  lv_obj_align(spinbox_notch_width_1, LV_ALIGN_TOP_LEFT, 360, 125);
  lv_obj_add_style(spinbox_notch_width_1, &spinbox_style1, LV_PART_MAIN);

  lv_obj_add_event_cb(spinbox_notch_width_1, event_notch_width1_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_notch_width_1);

  btn_notch_width_1_inc = lv_button_create(tab_notch);
  lv_obj_set_size(btn_notch_width_1_inc, obj_height, obj_height);
  lv_obj_align_to(btn_notch_width_1_inc, spinbox_notch_width_1, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_notch_width_1_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_notch_width_1_inc, increment_notch_width1_event_cb, LV_EVENT_ALL,  NULL);

  btn_notch_width_1_dec = lv_button_create(tab_notch);
  lv_obj_set_size(btn_notch_width_1_dec, obj_height, obj_height);
  lv_obj_align_to(btn_notch_width_1_dec, spinbox_notch_width_1, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_notch_width_1_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_notch_width_1_dec, decrement_notch_width1_event_cb, LV_EVENT_ALL, NULL);

  spinbox_notch_width_2 = lv_spinbox_create(tab_notch);
  lv_spinbox_set_range(spinbox_notch_width_2, 5, 300);
  lv_spinbox_set_digit_format(spinbox_notch_width_2, 3, 0);

  lv_spinbox_set_step(spinbox_notch_width_2, 1);
  lv_obj_set_width(spinbox_notch_width_2, 65);
  lv_obj_set_height(spinbox_notch_width_2, 46);

  lv_obj_align(spinbox_notch_width_2, LV_ALIGN_TOP_LEFT, 360, 195);
  lv_obj_add_style(spinbox_notch_width_2, &spinbox_style1, LV_PART_MAIN);

  lv_obj_add_event_cb(spinbox_notch_width_2, event_notch_width2_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_notch_width_2);

  btn_notch_width_2_inc = lv_button_create(tab_notch);
  lv_obj_set_size(btn_notch_width_2_inc, obj_height, obj_height);
  lv_obj_align_to(btn_notch_width_2_inc, spinbox_notch_width_2, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_notch_width_2_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_notch_width_2_inc, increment_notch_width2_event_cb, LV_EVENT_ALL,  NULL);

  btn_notch_width_2_dec = lv_button_create(tab_notch);
  lv_obj_set_size(btn_notch_width_2_dec, obj_height, obj_height);
  lv_obj_align_to(btn_notch_width_2_dec, spinbox_notch_width_2, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_notch_width_2_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_notch_width_2_dec, decrement_notch_width2_event_cb, LV_EVENT_ALL, NULL);

  btn_notch_back = lv_btn_create(tab_notch);
  lv_obj_add_style(btn_notch_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_notch_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_notch_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_notch_back, LV_ALIGN_TOP_RIGHT, 0, 268);


  // tab_filter

  spinbox_filter_highpass = lv_spinbox_create(tab_filter);
  // since hi must be at least 50 lower than lo, set limit
  lv_spinbox_set_range(spinbox_filter_highpass, 100, 4000 - 50);
  lv_spinbox_set_digit_format(spinbox_filter_highpass, 4, 0);

  lv_spinbox_set_step(spinbox_filter_highpass, 1);
  lv_obj_set_width(spinbox_filter_highpass, 75);
  lv_obj_set_height(spinbox_filter_highpass, 46);

  lv_obj_align(spinbox_filter_highpass, LV_ALIGN_TOP_LEFT, 83, 73);
  lv_obj_add_style(spinbox_filter_highpass, &spinbox_style1, LV_PART_MAIN);

  lv_obj_add_event_cb(spinbox_filter_highpass, event_filter_highpass_frequency_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_filter_highpass);

  btn_filter_highpass_inc = lv_button_create(tab_filter);
  lv_obj_set_size(btn_filter_highpass_inc, obj_height, obj_height);
  lv_obj_align_to(btn_filter_highpass_inc, spinbox_filter_highpass, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_filter_highpass_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_filter_highpass_inc, increment_filter_high_event_cb, LV_EVENT_ALL,  NULL);

  btn_filter_highpass_dec = lv_button_create(tab_filter);
  lv_obj_set_size(btn_filter_highpass_dec, obj_height, obj_height);
  lv_obj_align_to(btn_filter_highpass_dec, spinbox_filter_highpass, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_filter_highpass_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_filter_highpass_dec, decrement_filter_high_event_cb, LV_EVENT_ALL, NULL);

  spinbox_filter_lowpass = lv_spinbox_create(tab_filter);
  // since lowpass must be at least 50 higher than hi set limit
  lv_spinbox_set_range(spinbox_filter_lowpass, 100 + 50, 4000);
  lv_spinbox_set_digit_format(spinbox_filter_lowpass, 4, 0);

  lv_spinbox_set_step(spinbox_filter_lowpass, 1);
  lv_obj_set_width(spinbox_filter_lowpass, 75);
  lv_obj_set_height(spinbox_filter_lowpass, 46);

  lv_obj_align(spinbox_filter_lowpass, LV_ALIGN_TOP_LEFT, 323, 73);
  lv_obj_add_style(spinbox_filter_lowpass, &spinbox_style1, LV_PART_MAIN);

  lv_obj_add_event_cb(spinbox_filter_lowpass, event_filter_lowpass_frequency_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_filter_lowpass);

  btn_filter_lowpass_inc = lv_button_create(tab_filter);
  lv_obj_set_size(btn_filter_lowpass_inc, obj_height, obj_height);
  lv_obj_align_to(btn_filter_lowpass_inc, spinbox_filter_lowpass, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_filter_lowpass_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_filter_lowpass_inc, increment_filter_low_event_cb, LV_EVENT_ALL,  NULL);

  btn_filter_lowpass_dec = lv_button_create(tab_filter);
  lv_obj_set_size(btn_filter_lowpass_dec, obj_height, obj_height);
  lv_obj_align_to(btn_filter_lowpass_dec, spinbox_filter_lowpass, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_filter_lowpass_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_filter_lowpass_dec, decrement_filter_low_event_cb, LV_EVENT_ALL, NULL);

  spinbox_filter_bandpass = lv_spinbox_create(tab_filter);
  // since bw is min 50, adjust limits
  lv_spinbox_set_range(spinbox_filter_bandpass, 100 + 25, 4000 - 75);
  lv_spinbox_set_digit_format(spinbox_filter_bandpass, 4, 0);

  lv_spinbox_set_step(spinbox_filter_bandpass, 1);
  lv_obj_set_width(spinbox_filter_bandpass, 75);
  lv_obj_set_height(spinbox_filter_bandpass, 46);

  lv_obj_align(spinbox_filter_bandpass, LV_ALIGN_TOP_LEFT, 83, 203);
  lv_obj_add_style(spinbox_filter_bandpass, &spinbox_style1, LV_PART_MAIN);

  lv_obj_add_event_cb(spinbox_filter_bandpass, event_filter_bandpass_frequency_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_filter_bandpass);

  btn_filter_bandpass_inc = lv_button_create(tab_filter);
  lv_obj_set_size(btn_filter_bandpass_inc, obj_height, obj_height);
  lv_obj_align_to(btn_filter_bandpass_inc, spinbox_filter_bandpass, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_filter_bandpass_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_filter_bandpass_inc, increment_filter_bp_event_cb, LV_EVENT_ALL,  NULL);

  btn_filter_bandpass_dec = lv_button_create(tab_filter);
  lv_obj_set_size(btn_filter_bandpass_dec, obj_height, obj_height);
  lv_obj_align_to(btn_filter_bandpass_dec, spinbox_filter_bandpass, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_filter_bandpass_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_filter_bandpass_dec, decrement_filter_bp_event_cb, LV_EVENT_ALL, NULL);

  spinbox_filter_bandwidth = lv_spinbox_create(tab_filter);
  lv_spinbox_set_range(spinbox_filter_bandwidth, 50, 7950);
  lv_spinbox_set_digit_format(spinbox_filter_bandwidth, 4, 0);

  lv_spinbox_set_step(spinbox_filter_bandwidth, 1);
  lv_obj_set_width(spinbox_filter_bandwidth, 75);
  lv_obj_set_height(spinbox_filter_bandwidth, 46);

  lv_obj_align(spinbox_filter_bandwidth, LV_ALIGN_TOP_LEFT, 323, 203);
  lv_obj_add_style(spinbox_filter_bandwidth, &spinbox_style1, LV_PART_MAIN);

  lv_obj_add_event_cb(spinbox_filter_bandwidth, event_filter_bandpass_width_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_filter_bandwidth);

  btn_filter_bandwidth_inc = lv_button_create(tab_filter);
  lv_obj_set_size(btn_filter_bandwidth_inc, obj_height, obj_height);
  lv_obj_align_to(btn_filter_bandwidth_inc, spinbox_filter_bandwidth, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_filter_bandwidth_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_filter_bandwidth_inc, increment_filter_bw_event_cb, LV_EVENT_ALL,  NULL);

  btn_filter_bandwidth_dec = lv_button_create(tab_filter);
  lv_obj_set_size(btn_filter_bandwidth_dec, obj_height, obj_height);
  lv_obj_align_to(btn_filter_bandwidth_dec, spinbox_filter_bandwidth, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_filter_bandwidth_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_filter_bandwidth_dec, decrement_filter_bw_event_cb, LV_EVENT_ALL, NULL);

  static lv_point_precise_t line_pointshp[] = { {0, 39}, {39, 39}, {79, 0}, {119, 0} };
  static lv_point_precise_t line_pointslp[] = { {0, 0}, {39, 0}, {79, 39}, {119, 39} };
  static lv_point_precise_t line_pointsbp[] = { {0, 39}, {29, 39}, {44, 0}, {74, 0}, {89, 39}, {119, 39} };
  static lv_point_precise_t line_pointsbw[] = { {41, 19}, {77, 19} };
  static lv_point_precise_t line_pointsbwl[] = { {37, 19}, {40, 19} };
  static lv_point_precise_t line_pointsbwr[] = { {78, 19}, {81, 19} };
  static lv_point_precise_t line_pointsbpct[] = { {59, 0}, {59, 4} };

  lv_obj_t * filter_line1;
  filter_line1 = lv_line_create(tab_filter);
  lv_line_set_points(filter_line1, line_pointshp, 4);
  lv_obj_add_style(filter_line1, &style_line, 0);
  lv_obj_align(filter_line1, LV_ALIGN_TOP_LEFT, 60, 13);

  lv_obj_t * filter_line2;
  filter_line2 = lv_line_create(tab_filter);
  lv_line_set_points(filter_line2, line_pointslp, 4);
  lv_obj_add_style(filter_line2, &style_line, 0);
  lv_obj_align(filter_line2, LV_ALIGN_TOP_LEFT, 300, 13);

  lv_obj_t * filter_line3;
  filter_line3 = lv_line_create(tab_filter);
  lv_line_set_points(filter_line3, line_pointsbp, 6);
  lv_obj_add_style(filter_line3, &style_line, 0);
  lv_obj_align(filter_line3, LV_ALIGN_TOP_LEFT, 60, 143);

  lv_obj_t * filter_line4;
  filter_line4 = lv_line_create(tab_filter);
  lv_line_set_points(filter_line4, line_pointsbpct, 2);
  lv_obj_add_style(filter_line4, &style_line2, 0);
  lv_obj_align(filter_line4, LV_ALIGN_TOP_LEFT, 60, 143);

  lv_obj_t * filter_line5;
  filter_line5 = lv_line_create(tab_filter);
  lv_line_set_points(filter_line5, line_pointsbp, 6);
  lv_obj_add_style(filter_line5, &style_line, 0);
  lv_obj_align(filter_line5, LV_ALIGN_TOP_LEFT, 300, 143);

  lv_obj_t * filter_line6;
  filter_line6 = lv_line_create(tab_filter);
  lv_line_set_points(filter_line6, line_pointsbw, 2);
  lv_obj_add_style(filter_line6, &style_line, 0);
  lv_obj_align(filter_line6, LV_ALIGN_TOP_LEFT, 300, 143);

  lv_obj_t * filter_line7;
  filter_line7 = lv_line_create(tab_filter);
  lv_line_set_points(filter_line7, line_pointsbwl, 2);
  lv_obj_add_style(filter_line7, &style_line2, 0);
  lv_obj_align(filter_line7, LV_ALIGN_TOP_LEFT, 300, 143);

  lv_obj_t * filter_line8;
  filter_line8 = lv_line_create(tab_filter);
  lv_line_set_points(filter_line8, line_pointsbwr, 2);
  lv_obj_add_style(filter_line8, &style_line2, 0);
  lv_obj_align(filter_line8, LV_ALIGN_TOP_LEFT, 300, 143);

  dropdown_filter_mode = lv_dropdown_create(tab_filter);
  lv_dropdown_set_options(dropdown_filter_mode, "Bw 1\n"
                          "Bw 2\n"
                          "Bw 3\n"
                          "Bw 4\n"
                          "Cs 1\n"
                          "Cs 2\n"
                          "Cs 3\n"
                          "Cs 4");

  lv_dropdown_set_dir(dropdown_filter_mode, LV_DIR_BOTTOM);
  lv_dropdown_set_symbol(dropdown_filter_mode, LV_SYMBOL_UP);
  lv_obj_set_width(dropdown_filter_mode, 149);
  lv_obj_set_height(dropdown_filter_mode, 50);
  lv_obj_add_style(dropdown_filter_mode, &dropdown_style1, LV_PART_MAIN);
  lv_obj_t * dlist = lv_dropdown_get_list(dropdown_filter_mode);
  lv_obj_add_style(dlist, &dropdown_style1, LV_PART_MAIN);

  lv_obj_align(dropdown_filter_mode, LV_ALIGN_TOP_LEFT, 0, 268);
  lv_obj_add_event_cb(dropdown_filter_mode, event_filter_mode, LV_EVENT_ALL, NULL);

  btn_filter_reset = lv_btn_create(tab_filter);
  lv_obj_add_style(btn_filter_reset, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_filter_reset, event_filter_reset, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_filter_reset);
  lv_label_set_text(label, "Reset");
  lv_obj_center(label);
  lv_obj_align(btn_filter_reset, LV_ALIGN_TOP_MID, 0, 268);

  btn_filter_back = lv_btn_create(tab_filter);
  lv_obj_add_style(btn_filter_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_filter_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_filter_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_filter_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab peak

  cb_peak_on = lv_checkbox_create(tab_peak);
  lv_checkbox_set_text(cb_peak_on, "On");
  lv_obj_align(cb_peak_on, LV_ALIGN_TOP_LEFT, 5, 78);
  lv_obj_add_style(cb_peak_on, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_peak_on, event_peak_switch, LV_EVENT_ALL, NULL);

  spinbox_peak_bandpass = lv_spinbox_create(tab_peak);
  // since bw is min 50, adjust limits
  lv_spinbox_set_range(spinbox_peak_bandpass, 100 + 25, 4000 - 75);
  lv_spinbox_set_digit_format(spinbox_peak_bandpass, 4, 0);

  lv_spinbox_set_step(spinbox_peak_bandpass, 1);
  lv_obj_set_width(spinbox_peak_bandpass, 75);
  lv_obj_set_height(spinbox_peak_bandpass, 46);

  lv_obj_align(spinbox_peak_bandpass, LV_ALIGN_TOP_LEFT, 150, 73);
  lv_obj_add_style(spinbox_peak_bandpass, &spinbox_style1, LV_PART_MAIN);

  lv_obj_add_event_cb(spinbox_peak_bandpass, event_peak_bandpass_frequency_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_peak_bandpass);

  btn_peak_bandpass_inc = lv_button_create(tab_peak);
  lv_obj_set_size(btn_peak_bandpass_inc, obj_height, obj_height);
  lv_obj_align_to(btn_peak_bandpass_inc, spinbox_peak_bandpass, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_peak_bandpass_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_peak_bandpass_inc, increment_peak_bp_event_cb, LV_EVENT_ALL,  NULL);

  btn_peak_bandpass_dec = lv_button_create(tab_peak);
  lv_obj_set_size(btn_peak_bandpass_dec, obj_height, obj_height);
  lv_obj_align_to(btn_peak_bandpass_dec, spinbox_peak_bandpass, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_peak_bandpass_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_peak_bandpass_dec, decrement_peak_bp_event_cb, LV_EVENT_ALL, NULL);

  spinbox_peak_bandwidth = lv_spinbox_create(tab_peak);
  lv_spinbox_set_range(spinbox_peak_bandwidth, 5, 500);
  lv_spinbox_set_digit_format(spinbox_peak_bandwidth, 3, 0);

  lv_spinbox_set_step(spinbox_peak_bandwidth, 1);
  lv_obj_set_width(spinbox_peak_bandwidth, 65);
  lv_obj_set_height(spinbox_peak_bandwidth, 46);

  lv_obj_align(spinbox_peak_bandwidth , LV_ALIGN_TOP_LEFT, 360, 73);
  lv_obj_add_style(spinbox_peak_bandwidth , &spinbox_style1, LV_PART_MAIN);

  lv_obj_add_event_cb(spinbox_peak_bandwidth , event_peak_bandpass_width_change, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_peak_bandwidth);

  btn_peak_bandwidth_inc = lv_button_create(tab_peak);
  lv_obj_set_size(btn_peak_bandwidth_inc, obj_height, obj_height);
  lv_obj_align_to(btn_peak_bandwidth_inc, spinbox_peak_bandwidth, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_peak_bandwidth_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_peak_bandwidth_inc, increment_peak_bw_event_cb, LV_EVENT_ALL,  NULL);

  btn_peak_bandwidth_dec = lv_button_create(tab_peak);
  lv_obj_set_size(btn_peak_bandwidth_dec, obj_height, obj_height);
  lv_obj_align_to(btn_peak_bandwidth_dec, spinbox_peak_bandwidth, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_peak_bandwidth_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_peak_bandwidth_dec, decrement_peak_bw_event_cb, LV_EVENT_ALL, NULL);

  lv_obj_t * peak_line3;
  peak_line3 = lv_line_create(tab_peak);
  lv_line_set_points(peak_line3, line_pointsbp, 6);
  lv_obj_add_style(peak_line3, &style_line, 0);
  lv_obj_align(peak_line3, LV_ALIGN_TOP_LEFT, 127, 13);

  lv_obj_t * peak_line4;
  peak_line4 = lv_line_create(tab_peak);
  lv_line_set_points(peak_line4, line_pointsbpct, 2);
  lv_obj_add_style(peak_line4, &style_line2, 0);
  lv_obj_align(peak_line4, LV_ALIGN_TOP_LEFT, 127, 13);

  lv_obj_t * peak_line5;
  peak_line5 = lv_line_create(tab_peak);
  lv_line_set_points(peak_line5, line_pointsbp, 6);
  lv_obj_add_style(peak_line5, &style_line, 0);
  lv_obj_align(peak_line5, LV_ALIGN_TOP_LEFT, 332, 13);

  lv_obj_t * peak_line6;
  peak_line6 = lv_line_create(tab_peak);
  lv_line_set_points(peak_line6, line_pointsbw, 2);
  lv_obj_add_style(peak_line6, &style_line, 0);
  lv_obj_align(peak_line6, LV_ALIGN_TOP_LEFT, 332, 13);

  lv_obj_t * peak_line7;
  peak_line7 = lv_line_create(tab_peak);
  lv_line_set_points(peak_line7, line_pointsbwl, 2);
  lv_obj_add_style(peak_line7, &style_line2, 0);
  lv_obj_align(peak_line7, LV_ALIGN_TOP_LEFT, 332, 13);

  lv_obj_t * peak_line8;
  peak_line8 = lv_line_create(tab_peak);
  lv_line_set_points(peak_line8, line_pointsbwr, 2);
  lv_obj_add_style(peak_line8, &style_line2, 0);
  lv_obj_align(peak_line8, LV_ALIGN_TOP_LEFT, 332, 13);

  btn_peak_back = lv_btn_create(tab_peak);
  lv_obj_add_style(btn_peak_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_peak_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_peak_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_peak_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab compressor

  lv_obj_t * vuframe2 = lv_obj_create(tab_compressor);
  lv_obj_set_size(vuframe2, 29, 304);
  lv_obj_align(vuframe2, LV_ALIGN_TOP_LEFT, 0, 8);
  lv_obj_set_style_bg_opa(vuframe2, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_radius(vuframe2, 0, LV_PART_MAIN);
  lv_obj_clear_flag(vuframe2, LV_OBJ_FLAG_SCROLLABLE);

  slider_compressor_threshold = lv_slider_create(tab_compressor);

  lv_obj_remove_style_all(slider_compressor_threshold);        /*Remove the styles coming from the theme*/

  lv_obj_add_style(slider_compressor_threshold, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_compressor_threshold, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_compressor_threshold, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_compressor_threshold, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_compressor_threshold, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);

  lv_obj_set_size(slider_compressor_threshold, 15, 185);
  lv_slider_set_range(slider_compressor_threshold, 0, 50);
  lv_obj_align(slider_compressor_threshold, LV_ALIGN_TOP_LEFT, 90, 55);

  lv_obj_add_event_cb(slider_compressor_threshold, event_compressor_threshold_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_compressor_threshold = lv_label_create(tab_compressor);
  lv_label_set_text(lbl_compressor_threshold, "Threshold");
  lv_obj_add_style(lbl_compressor_threshold, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_compressor_threshold, slider_compressor_threshold, LV_ALIGN_OUT_TOP_MID, 0, -25);

  slider_compressor_ratio = lv_slider_create(tab_compressor);

  lv_obj_remove_style_all(slider_compressor_ratio);        /*Remove the styles coming from the theme*/

  lv_obj_add_style(slider_compressor_ratio, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_compressor_ratio, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_compressor_ratio, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_compressor_ratio, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_compressor_ratio, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);

  lv_obj_set_size(slider_compressor_ratio, 15, 185);
  lv_slider_set_range(slider_compressor_ratio, 1, 10);
  lv_obj_align(slider_compressor_ratio, LV_ALIGN_TOP_LEFT, 190, 55);

  lv_obj_add_event_cb(slider_compressor_ratio, event_compressor_ratio_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_compressor_ratio = lv_label_create(tab_compressor);
  lv_label_set_text(lbl_compressor_ratio, "Ratio");
  lv_obj_add_style(lbl_compressor_ratio, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_compressor_ratio, slider_compressor_ratio, LV_ALIGN_OUT_TOP_MID, 0, -25);

  slider_compressor_postgain = lv_slider_create(tab_compressor);

  lv_obj_remove_style_all(slider_compressor_postgain);        /*Remove the styles coming from the theme*/

  lv_obj_add_style(slider_compressor_postgain, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_compressor_postgain, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_compressor_postgain, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_compressor_postgain, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_compressor_postgain, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);

  lv_obj_set_size(slider_compressor_postgain, 15, 185);
  lv_slider_set_range(slider_compressor_postgain, 0, 20);
  lv_obj_align(slider_compressor_postgain, LV_ALIGN_TOP_LEFT, 290, 55);

  lv_obj_add_event_cb(slider_compressor_postgain, event_compressor_postgain_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_compressor_postgain = lv_label_create(tab_compressor);
  lv_label_set_text(lbl_compressor_postgain, "PostGain");
  lv_obj_add_style(lbl_compressor_postgain, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_compressor_postgain, slider_compressor_postgain, LV_ALIGN_OUT_TOP_MID, 0, -25);

  slider_compressor_noisegate = lv_slider_create(tab_compressor);

  lv_obj_remove_style_all(slider_compressor_noisegate);        /*Remove the styles coming from the theme*/

  lv_obj_add_style(slider_compressor_noisegate, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_compressor_noisegate, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_compressor_noisegate, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_compressor_noisegate, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_compressor_noisegate, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);

  lv_obj_set_size(slider_compressor_noisegate, 15, 185);
  lv_slider_set_range(slider_compressor_noisegate, 0, 35);
  lv_obj_align(slider_compressor_noisegate, LV_ALIGN_TOP_LEFT, 390, 55);

  lv_obj_add_event_cb(slider_compressor_noisegate, event_compressor_noisegate_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_compressor_noisegate = lv_label_create(tab_compressor);
  lv_label_set_text(lbl_compressor_noisegate, "Gate");
  lv_obj_add_style(lbl_compressor_noisegate, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(lbl_compressor_noisegate, slider_compressor_noisegate, LV_ALIGN_OUT_TOP_MID, 0, -25);

  btn_compressor_reset = lv_btn_create(tab_compressor);
  lv_obj_add_style(btn_compressor_reset, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_compressor_reset, event_compressor_reset, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_compressor_reset);
  lv_label_set_text(label, "Reset");
  lv_obj_center(label);
  lv_obj_align(btn_compressor_reset, LV_ALIGN_TOP_MID, 0, 268);

  btn_compressor_back = lv_btn_create(tab_compressor);
  lv_obj_add_style(btn_compressor_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_compressor_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_compressor_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_compressor_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab equalizer

  slider_eq0 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq0);
  lv_obj_add_style(slider_eq0, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq0, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq0, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq0, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq0, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq0, 15, 200);
  lv_slider_set_range(slider_eq0, -15, +15);
  lv_obj_align(slider_eq0, LV_ALIGN_TOP_LEFT, 12, 30);
  lv_obj_add_event_cb(slider_eq0, event_eq0_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq0 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq0, "100");
  lv_obj_add_style(lbl_eq0, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq0, slider_eq0, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq1 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq1);
  lv_obj_add_style(slider_eq1, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq1, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq1, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq1, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq1, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq1, 15, 200);
  lv_slider_set_range(slider_eq1, -15, +15);
  lv_obj_align(slider_eq1, LV_ALIGN_TOP_LEFT, 52, 30);
  lv_obj_add_event_cb(slider_eq1, event_eq1_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq1 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq1, "150");
  lv_obj_add_style(lbl_eq1, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq1, slider_eq1, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq2 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq2);
  lv_obj_add_style(slider_eq2, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq2, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq2, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq2, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq2, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq2, 15, 200);
  lv_slider_set_range(slider_eq2, -15, +15);
  lv_obj_align(slider_eq2, LV_ALIGN_TOP_LEFT, 92, 30);
  lv_obj_add_event_cb(slider_eq2, event_eq2_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq2 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq2, "220");
  lv_obj_add_style(lbl_eq2, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq2, slider_eq2, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq3 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq3);
  lv_obj_add_style(slider_eq3, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq3, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq3, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq3, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq3, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq3, 15, 200);
  lv_slider_set_range(slider_eq3, -15, +15);
  lv_obj_align(slider_eq3, LV_ALIGN_TOP_LEFT, 132, 30);
  lv_obj_add_event_cb(slider_eq3, event_eq3_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq3 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq3, "300");
  lv_obj_add_style(lbl_eq3, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq3, slider_eq3, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq4 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq4);
  lv_obj_add_style(slider_eq4, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq4, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq4, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq4, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq4, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);

  lv_obj_set_size(slider_eq4, 15, 200);
  lv_slider_set_range(slider_eq4, -15, +15);
  lv_obj_align(slider_eq4, LV_ALIGN_TOP_LEFT, 172, 30);
  lv_obj_add_event_cb(slider_eq4, event_eq4_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq4 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq4, "430");
  lv_obj_add_style(lbl_eq4, &label_style3, LV_PART_MAIN);

  lv_obj_align_to(lbl_eq4, slider_eq4, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq5 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq5);
  lv_obj_add_style(slider_eq5, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq5, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq5, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq5, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq5, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq5, 15, 200);
  lv_slider_set_range(slider_eq5, -15, +15);
  lv_obj_align(slider_eq5, LV_ALIGN_TOP_LEFT, 212, 30);
  lv_obj_add_event_cb(slider_eq5, event_eq5_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq5 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq5, "600");
  lv_obj_add_style(lbl_eq5, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq5, slider_eq5, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq6 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq6);
  lv_obj_add_style(slider_eq6, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq6, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq6, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq6, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq6, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq6, 15, 200);
  lv_slider_set_range(slider_eq6, -15, +15);
  lv_obj_align(slider_eq6, LV_ALIGN_TOP_LEFT, 252, 30);
  lv_obj_add_event_cb(slider_eq6, event_eq6_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq6 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq6, "850");
  lv_obj_add_style(lbl_eq6, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq6, slider_eq6, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq7 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq7);
  lv_obj_add_style(slider_eq7, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq7, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq7, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq7, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq7, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq7, 15, 200);
  lv_slider_set_range(slider_eq7, -15, +15);
  lv_obj_align(slider_eq7, LV_ALIGN_TOP_LEFT, 292, 30);
  lv_obj_add_event_cb(slider_eq7, event_eq7_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq7 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq7, "1.2K");
  lv_obj_add_style(lbl_eq7, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq7, slider_eq7, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq8 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq8);
  lv_obj_add_style(slider_eq8, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq8, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq8, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq8, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq8, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq8, 15, 200);
  lv_slider_set_range(slider_eq8, -15, +15);
  lv_obj_align(slider_eq8, LV_ALIGN_TOP_LEFT, 332, 30);
  lv_obj_add_event_cb(slider_eq8, event_eq8_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq8 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq8, "1.7K");
  lv_obj_add_style(lbl_eq8, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq8, slider_eq8, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq9 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq9);
  lv_obj_add_style(slider_eq9, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq9, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq9, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq9, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq9, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq9, 15, 200);
  lv_slider_set_range(slider_eq9, -15, +15);
  lv_obj_align(slider_eq9, LV_ALIGN_TOP_LEFT, 372, 30);
  lv_obj_add_event_cb(slider_eq9, event_eq9_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq9 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq9, "2.4K");
  lv_obj_add_style(lbl_eq9, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq9, slider_eq9, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq10 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq10);
  lv_obj_add_style(slider_eq10, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq10, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq10, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq10, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq10, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq10, 15, 200);
  lv_slider_set_range(slider_eq10, -15, +15);
  lv_obj_align(slider_eq10, LV_ALIGN_TOP_LEFT, 412, 30);
  lv_obj_add_event_cb(slider_eq10, event_eq10_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq10 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq10, "3.3K");
  lv_obj_add_style(lbl_eq10, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq10, slider_eq10, LV_ALIGN_OUT_TOP_MID, 0, -15);

  slider_eq11 = lv_slider_create(tab_equalizer);
  lv_obj_remove_style_all(slider_eq11);
  lv_obj_add_style(slider_eq11, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_eq11, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_eq11, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_eq11, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_eq11, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_eq11, 15, 200);
  lv_slider_set_range(slider_eq11, -15, +15);
  lv_obj_align(slider_eq11, LV_ALIGN_TOP_LEFT, 452, 30);
  lv_obj_add_event_cb(slider_eq11, event_eq11_change, LV_EVENT_VALUE_CHANGED, NULL);

  lbl_eq11 = lv_label_create(tab_equalizer);
  lv_label_set_text(lbl_eq11, "3.8K");
  lv_obj_add_style(lbl_eq11, &label_style3, LV_PART_MAIN);
  lv_obj_align_to(lbl_eq11, slider_eq11, LV_ALIGN_OUT_TOP_MID, 0, -15);

  btn_eq_reset = lv_btn_create(tab_equalizer);
  lv_obj_add_style(btn_eq_reset, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_eq_reset, event_eq_reset, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_eq_reset);
  lv_label_set_text(label, "Reset");
  lv_obj_center(label);
  lv_obj_align(btn_eq_reset, LV_ALIGN_TOP_MID, 0, 268);

  btn_eq_back = lv_btn_create(tab_equalizer);
  lv_obj_add_style(btn_eq_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_eq_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_eq_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_eq_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab presets

  cb_presets_selection0 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection0, event_presets_sel0, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection0, LV_ALIGN_TOP_LEFT, 5, 5);

  cb_presets_selection1 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection1, event_presets_sel1, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection1, LV_ALIGN_TOP_LEFT, 5, 35);

  cb_presets_selection2 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection2, event_presets_sel2, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection2, LV_ALIGN_TOP_LEFT, 5, 65);

  cb_presets_selection3 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection3, event_presets_sel3, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection3, LV_ALIGN_TOP_LEFT, 5, 95);

  cb_presets_selection4 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection4, event_presets_sel4, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection4, LV_ALIGN_TOP_LEFT, 5, 125);

  cb_presets_selection5 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection5, event_presets_sel5, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection5, LV_ALIGN_TOP_LEFT, 5, 155);

  cb_presets_selection6 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection6, event_presets_sel6, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection6, LV_ALIGN_TOP_LEFT, 5, 185);

  cb_presets_selection7 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection7, event_presets_sel7, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection7, LV_ALIGN_TOP_LEFT, 5, 215);

  cb_presets_selection8 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection8, event_presets_sel8, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection8, LV_ALIGN_TOP_LEFT, 5, 245);

  cb_presets_selection9 = lv_checkbox_create(tab_presets);
  lv_obj_add_event_cb(cb_presets_selection9, event_presets_sel9, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_presets_selection9, LV_ALIGN_TOP_LEFT, 5, 275);

  // separate or it does not work
  lv_obj_add_style(cb_presets_selection0, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection0, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_presets_selection1, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection1, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_presets_selection2, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection2, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_presets_selection3, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection3, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_presets_selection4, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection4, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_presets_selection5, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection5, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_presets_selection6, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection6, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_presets_selection7, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection7, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_presets_selection8, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection8, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_presets_selection9, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_presets_selection9, &radiobtn_style1, LV_PART_INDICATOR);

  lv_obj_add_style(cb_presets_selection0, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_presets_selection1, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_presets_selection2, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_presets_selection3, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_presets_selection4, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_presets_selection5, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_presets_selection6, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_presets_selection7, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_presets_selection8, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_presets_selection9, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);

  btn_presets_recall = lv_btn_create(tab_presets);
  lv_obj_add_style(btn_presets_recall, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_recall, event_presets_recall, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_recall);
  lv_label_set_text(label, "Recall");
  lv_obj_center(label);
  lv_obj_align(btn_presets_recall, LV_ALIGN_TOP_RIGHT, 0, 134);

  btn_presets_set = lv_btn_create(tab_presets);
  lv_obj_add_style(btn_presets_set, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_set, event_presets_set, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_set);
  lv_label_set_text(label, "Set");
  lv_obj_center(label);
  lv_obj_align(btn_presets_set, LV_ALIGN_TOP_RIGHT, 0, 201);

  btn_presets_back = lv_btn_create(tab_presets);
  lv_obj_add_style(btn_presets_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presets_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_presets_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_presets_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab scope

  btn_scope_back = lv_btn_create(tab_scope);
  lv_obj_add_style(btn_scope_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_scope_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_scope_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_scope_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab spectrum

  lbl_spectrum_cursor = lv_label_create(tab_spectrum);
  lv_label_set_text(lbl_spectrum_cursor, "");
  lv_obj_add_style(lbl_spectrum_cursor, &label_style7, LV_PART_MAIN);
  lv_obj_align(lbl_spectrum_cursor, LV_ALIGN_TOP_LEFT, 5, 250);

  btn_spectrum_back = lv_btn_create(tab_spectrum);
  lv_obj_add_style(btn_spectrum_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_spectrum_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_spectrum_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_spectrum_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // morse

  lv_obj_t * vuframe3 = lv_obj_create(tab_morse);
  lv_obj_set_size(vuframe3, 29, 304);
  lv_obj_align(vuframe3, LV_ALIGN_TOP_LEFT, 0, 8);
  lv_obj_set_style_bg_opa(vuframe3, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_radius(vuframe3, 0, LV_PART_MAIN);
  lv_obj_clear_flag(vuframe3, LV_OBJ_FLAG_SCROLLABLE);

  obj_morse_text_container = lv_obj_create(tab_morse);
  lv_obj_set_size(obj_morse_text_container, 430, 140);  // pas breedte aan op font
  lv_obj_align(obj_morse_text_container, LV_ALIGN_TOP_LEFT, 39, 70);
  lv_obj_set_scroll_dir(obj_morse_text_container, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(obj_morse_text_container, LV_SCROLLBAR_MODE_OFF);

  lv_obj_set_style_clip_corner(obj_morse_text_container, true, 0);

  lbl_morse_text = lv_label_create(obj_morse_text_container);
  lv_label_set_long_mode(lbl_morse_text, LV_LABEL_LONG_WRAP);
  lv_obj_set_width(lbl_morse_text, LV_PCT(100));
  lv_obj_set_height(lbl_morse_text, LV_SIZE_CONTENT);
  lv_obj_align(lbl_morse_text, LV_ALIGN_TOP_LEFT, 0, 0);
  //  lv_obj_set_style_text_font(lbl_morse_text, &lv_font_montserrat_16_mono, 0);  // gebruik monospaced font
  lv_label_set_text(lbl_morse_text, "");
  lv_obj_add_style(lbl_morse_text, &label_style8, LV_PART_MAIN);

  cb_morse_on = lv_checkbox_create(tab_morse);
  lv_checkbox_set_text(cb_morse_on, "On");
  lv_obj_align(cb_morse_on, LV_ALIGN_TOP_LEFT, 40, 272);
  lv_obj_add_style(cb_morse_on, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_morse_on, event_morse_switch, LV_EVENT_ALL, NULL);

  lbl_morse_freq = lv_label_create(tab_morse);
  lv_label_set_text(lbl_morse_freq, "F: 1200Hz");
  lv_obj_add_style(lbl_morse_freq, &label_style7, LV_PART_MAIN);
  lv_obj_align(lbl_morse_freq, LV_ALIGN_TOP_LEFT, 40, 228);

  btn_morse_clear = lv_btn_create(tab_morse);
  lv_obj_add_style(btn_morse_clear, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_morse_clear, event_morse_clear, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_morse_clear);
  lv_label_set_text(label, "Clear");
  lv_obj_center(label);
  lv_obj_align(btn_morse_clear, LV_ALIGN_TOP_MID, 0, 268);

  btn_morse_back = lv_btn_create(tab_morse);
  lv_obj_add_style(btn_morse_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_morse_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_morse_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_morse_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab test

  slider_test_signal_gain = lv_slider_create(tab_test);
  lv_obj_remove_style_all(slider_test_signal_gain);
  lv_obj_add_style(slider_test_signal_gain, &slider_style1_main, LV_PART_MAIN);
  lv_obj_add_style(slider_test_signal_gain, &slider_style1_indicator, LV_PART_INDICATOR);
  lv_obj_add_style(slider_test_signal_gain, &slider_style1_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
  lv_obj_add_style(slider_test_signal_gain, &slider_style1_knob, LV_PART_KNOB);
  lv_obj_add_style(slider_test_signal_gain, &slider_style1_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);
  lv_obj_set_size(slider_test_signal_gain, 15, 200);
  lv_slider_set_range(slider_test_signal_gain, 0, 39);
  lv_obj_align(slider_test_signal_gain, LV_ALIGN_TOP_LEFT, 25, 60);
  lv_obj_add_event_cb(slider_test_signal_gain, event_test_signal_gain_change, LV_EVENT_ALL, NULL);

  lv_obj_t * slider_label = lv_label_create(tab_test);
  lv_label_set_text(slider_label, "Gain");
  lv_obj_add_style(slider_label, &label_style4, LV_PART_MAIN);
  lv_obj_align_to(slider_label, slider_test_signal_gain, LV_ALIGN_OUT_TOP_MID, 0, -25);

  spinbox_test_frequency_1 = lv_spinbox_create(tab_test);
  lv_spinbox_set_range(spinbox_test_frequency_1, 20, 15000);
  lv_spinbox_set_digit_format(spinbox_test_frequency_1, 5, 0);
  lv_spinbox_set_step(spinbox_test_frequency_1, 10);
  lv_obj_set_width(spinbox_test_frequency_1, 80);
  lv_obj_align(spinbox_test_frequency_1, LV_ALIGN_TOP_LEFT, 180, 150);
  lv_obj_add_style(spinbox_test_frequency_1, &spinbox_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(spinbox_test_frequency_1, frequency1_change_event_cb, LV_EVENT_ALL,  NULL);

  obj_height = lv_obj_get_height(spinbox_test_frequency_1);

  btn_test_frequency_1_inc = lv_button_create(tab_test);
  lv_obj_set_size(btn_test_frequency_1_inc, obj_height, obj_height);
  lv_obj_align_to(btn_test_frequency_1_inc, spinbox_test_frequency_1, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_image_src(btn_test_frequency_1_inc, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_test_frequency_1_inc, increment_test_frequency1_event_cb, LV_EVENT_ALL,  NULL);

  btn_test_frequency_1_dec = lv_button_create(tab_test);
  lv_obj_set_size(btn_test_frequency_1_dec, obj_height, obj_height);
  lv_obj_align_to(btn_test_frequency_1_dec, spinbox_test_frequency_1, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_style_bg_image_src(btn_test_frequency_1_dec, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_test_frequency_1_dec, decrement_test_frequency1_event_cb, LV_EVENT_ALL, NULL);

  cb_test_on = lv_checkbox_create(tab_test);
  lv_checkbox_set_text(cb_test_on, "On");
  lv_obj_align(cb_test_on, LV_ALIGN_TOP_LEFT, 87, 40);
  lv_obj_add_style(cb_test_on, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_test_on, event_test_switch, LV_EVENT_ALL, NULL);

  label = lv_label_create(tab_test);
  lv_label_set_text(label, LV_SYMBOL_AUDIO " 1");
  lv_obj_add_style(label, &label_style2, LV_PART_MAIN);
  lv_obj_align_to(label, spinbox_test_frequency_1, LV_ALIGN_OUT_LEFT_MID, -54, 0);

  btn_test_back = lv_btn_create(tab_test);
  lv_obj_add_style(btn_test_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_test_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_test_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_test_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab settings

  cb_settings_input0 = lv_checkbox_create(tab_settings);
  lv_checkbox_set_text(cb_settings_input0, "Speaker in");
  lv_obj_add_event_cb(cb_settings_input0, event_settings_select0, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_settings_input0, LV_ALIGN_TOP_LEFT, 5, 10);

  cb_settings_input1 = lv_checkbox_create(tab_settings);
  lv_checkbox_set_text(cb_settings_input1, "Line in");
  lv_obj_add_event_cb(cb_settings_input1, event_settings_select1, LV_EVENT_ALL, NULL);
  lv_obj_align(cb_settings_input1, LV_ALIGN_TOP_LEFT, 5, 60);

  // separate or it does not work
  lv_obj_add_style(cb_settings_input0, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_settings_input0, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_settings_input1, &radiobtn_style1, LV_PART_MAIN);
  lv_obj_add_style(cb_settings_input1, &radiobtn_style1, LV_PART_INDICATOR);
  lv_obj_add_style(cb_settings_input0, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);
  lv_obj_add_style(cb_settings_input1, &radiobtn_checked_style1, LV_PART_INDICATOR | LV_STATE_CHECKED);

  btn_settings_back = lv_btn_create(tab_settings);
  lv_obj_add_style(btn_settings_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_settings_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_settings_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_settings_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab edit

  edit_txtentry  = lv_textarea_create(tab_edit);
  lv_obj_add_style(edit_txtentry, &label_style1, LV_PART_MAIN);
  lv_textarea_set_one_line(edit_txtentry, true);
  lv_obj_set_size(edit_txtentry, 400, 45);
  lv_obj_align(edit_txtentry, LV_ALIGN_TOP_MID, 00, 10);
  edit_keyboard = lv_keyboard_create(tab_edit);
  lv_keyboard_set_textarea(edit_keyboard, edit_txtentry);
  lv_obj_align(edit_keyboard, LV_ALIGN_TOP_MID, 00, 100);
  lv_obj_add_event_cb(edit_keyboard,  event_keyboardhandler, LV_EVENT_ALL, NULL);

  // tab dynamic

  cb_dynamic_on = lv_checkbox_create(tab_dynamic);
  lv_checkbox_set_text(cb_dynamic_on, "Dynamic Noise Filter on");
  lv_obj_align(cb_dynamic_on, LV_ALIGN_TOP_LEFT, 5, 10);
  lv_obj_add_style(cb_dynamic_on, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_dynamic_on, event_dynamic_denoiser_switch, LV_EVENT_ALL, NULL);

  cb_dynamic_autonotch = lv_checkbox_create(tab_dynamic);
  lv_checkbox_set_text(cb_dynamic_autonotch, "Dynamic Notch Filter on");
  lv_obj_align(cb_dynamic_autonotch, LV_ALIGN_TOP_LEFT, 5, 60);
  lv_obj_add_style(cb_dynamic_autonotch, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_dynamic_autonotch, event_dynamic_notch_switch, LV_EVENT_ALL, NULL);

  cb_dynamic_agc = lv_checkbox_create(tab_dynamic);
  lv_checkbox_set_text(cb_dynamic_agc, "AGC on");
  lv_obj_align(cb_dynamic_agc, LV_ALIGN_TOP_LEFT, 5, 110);
  lv_obj_add_style(cb_dynamic_agc, &cb_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(cb_dynamic_agc, event_dynamic_agc_switch, LV_EVENT_ALL, NULL);

  btn_dynamic_back = lv_btn_create(tab_dynamic);
  lv_obj_add_style(btn_dynamic_back, &btn_style1, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_dynamic_back, event_menuscr, LV_EVENT_CLICKED, NULL);
  label = lv_label_create(btn_dynamic_back);
  lv_label_set_text(label, LV_SYMBOL_NEW_LINE);
  lv_obj_center(label);
  lv_obj_align(btn_dynamic_back, LV_ALIGN_TOP_RIGHT, 0, 268);

  // tab tab_presetsmain

  btn_presetsmain_preset0 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset0, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset0, event_mainpresets_sel0, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset0 = lv_label_create(btn_presetsmain_preset0);
  lv_obj_add_style(lbl_presetsmain_preset0, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset0, LV_ALIGN_TOP_LEFT, 0, 0);

  btn_presetsmain_preset1 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset1, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset1, event_mainpresets_sel1, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset1 = lv_label_create(btn_presetsmain_preset1);
  lv_obj_add_style(lbl_presetsmain_preset1, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset1, LV_ALIGN_TOP_LEFT, 0, 32);

  btn_presetsmain_preset2 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset2, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset2, event_mainpresets_sel2, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset2 = lv_label_create(btn_presetsmain_preset2);
  lv_obj_add_style(lbl_presetsmain_preset2, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset2, LV_ALIGN_TOP_LEFT, 0, 64);

  btn_presetsmain_preset3 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset3, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset3, event_mainpresets_sel3, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset3 = lv_label_create(btn_presetsmain_preset3);
  lv_obj_add_style(lbl_presetsmain_preset3, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset3, LV_ALIGN_TOP_LEFT, 0, 96);

  btn_presetsmain_preset4 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset4, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset4, event_mainpresets_sel4, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset4 = lv_label_create(btn_presetsmain_preset4);
  lv_obj_add_style(lbl_presetsmain_preset4, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset4, LV_ALIGN_TOP_LEFT, 0, 128);

  btn_presetsmain_preset5 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset5, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset5, event_mainpresets_sel5, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset5 = lv_label_create(btn_presetsmain_preset5);
  lv_obj_add_style(lbl_presetsmain_preset5, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset5, LV_ALIGN_TOP_LEFT, 0, 160);

  btn_presetsmain_preset6 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset6, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset6, event_mainpresets_sel6, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset6 = lv_label_create(btn_presetsmain_preset6);
  lv_obj_add_style(lbl_presetsmain_preset6, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset6, LV_ALIGN_TOP_LEFT, 0, 192);

  btn_presetsmain_preset7 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset7, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset7, event_mainpresets_sel7, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset7 = lv_label_create(btn_presetsmain_preset7);
  lv_obj_add_style(lbl_presetsmain_preset7, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset7, LV_ALIGN_TOP_LEFT, 0, 224);

  btn_presetsmain_preset8 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset8, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset8, event_mainpresets_sel8, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset8 = lv_label_create(btn_presetsmain_preset8);
  lv_obj_add_style(lbl_presetsmain_preset8, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset8, LV_ALIGN_TOP_LEFT, 0, 256);

  btn_presetsmain_preset9 = lv_btn_create(tab_presetsmain);
  lv_obj_add_style(btn_presetsmain_preset9, &btn_style2, LV_PART_MAIN);
  lv_obj_add_event_cb(btn_presetsmain_preset9, event_mainpresets_sel9, LV_EVENT_CLICKED, NULL);
  lbl_presetsmain_preset9 = lv_label_create(btn_presetsmain_preset9);
  lv_obj_add_style(lbl_presetsmain_preset9, &btn_label_style2, LV_PART_MAIN);
  lv_obj_align(btn_presetsmain_preset9, LV_ALIGN_TOP_LEFT, 0, 288);

}

// screen (tab) related

static void event_mainscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    cleanup();
    lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
  }
}

static void event_menuscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    cleanup();
    lv_tabview_set_act(tabview, TAB_MENU_REF, LV_ANIM_OFF);
  }
}

static void event_notchscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    cleanup();
    lv_tabview_set_act(tabview, TAB_NOTCH_REF, LV_ANIM_OFF);
  }
}

static void event_filterscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    cleanup();
    lv_tabview_set_act(tabview, TAB_FILTER_REF, LV_ANIM_OFF);
  }
}

static void event_peakscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    cleanup();
    lv_tabview_set_act(tabview, TAB_PEAK_REF, LV_ANIM_OFF);
  }
}

static void event_dynamicscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    cleanup();
    lv_tabview_set_act(tabview, TAB_DYNAMIC_REF, LV_ANIM_OFF);
  }
}

static void event_compressorscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_COMPRESSOR_REF, LV_ANIM_OFF);
  }
}

static void event_eqscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_EQUALIZER_REF, LV_ANIM_OFF);
  }
}

static void event_scopescr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    audio_scope_ready = false;
    audio_scope_fill_block_count = 0;
    audio_scope_trigger = true;
    lv_tabview_set_act(tabview, TAB_SCOPE_REF, LV_ANIM_OFF);
    // trigger frame draw
    audio_screen_update_done = false;
  }
}

static void event_spectrumscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    audio_scope_ready = false;
    audio_scope_fill_block_count = 0;
    audio_scope_trigger = true;
    lv_tabview_set_act(tabview, TAB_SPECTRUM_REF, LV_ANIM_OFF);
    // trigger frame draw
    audio_screen_update_done = false;
    audio_spectrum_peaktrigger = true;
    audio_spectrum_bandpasstrigger = true;
  }
}

static void event_morsescr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    morse_reset();
    // reset if outside limits
    if (filter_peak_on) {
      if (filter_peak_bandpass_freq > 1200 or filter_peak_bandpass_freq < 400) {
        lv_spinbox_set_value(spinbox_peak_bandpass, 1000);
        filter_peak_bandpass_freq = 1000;
      }
    }
    else {
      if (filter_bandpass_freq > 1200 or filter_bandpass_freq < 400) {
        lv_spinbox_set_value(spinbox_filter_bandpass, 1000);
        filter_bandpass_freq = 1000;
        lv_spinbox_set_value(spinbox_filter_bandwidth, 50);
        filter_bandwidth = 50;
      }
    }
    // sync filter freq
    morse_setlabel();
    lv_tabview_set_act(tabview, TAB_MORSE_REF, LV_ANIM_OFF);
  }
}

static void event_presetsscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_PRESETS_REF, LV_ANIM_OFF);
  }
}

static void event_testscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_TEST_REF, LV_ANIM_OFF);
  }
}

static void event_settingsscreen(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    lv_tabview_set_act(tabview, TAB_SETTINGS_REF, LV_ANIM_OFF);
  }
}

static void event_presetsmainscr(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    cleanup();
    lv_tabview_set_act(tabview, TAB_PRESETSMAIN_REF, LV_ANIM_OFF);
  }
}

// cleanup when going back to the menu screen
void cleanup(void) {
  switch (lv_tabview_get_tab_act(tabview)) {
    case TAB_NOTCH_REF:
      // notch screen
      audio_selected_notch = 0;
      set_notch_labels(audio_selected_notch);
      set_notch_cursors(audio_selected_notch);
      break;

    case TAB_FILTER_REF:
      // filter screen
      audio_filter_mode_hi_lo = true;
      set_filter_cursors(audio_filter_mode_hi_lo);
      break;

    case TAB_SCOPE_REF:
      scope_clear();
      break;

    case TAB_MORSE_REF:
      lv_obj_clear_state(cb_morse_on, LV_STATE_CHECKED);
      morse_buffer_ready = false;
      morse_on = false;
      break;
  }
}

// main screen

static void event_gain_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_gain_previous = audio_gain;
    audio_gain = lv_slider_get_value(obj);
    if (audio_gain != audio_gain_previous) {
      set_gain();
    }
    if (!audio_rotary_selected_gain) {
      audio_rotary_selected_gain = true;
      set_main_labels(0);
    }
  }
}

// update gain
void update_gain(void) {
  // update slider
  lv_slider_set_value(slider_main_gain, audio_gain, LV_ANIM_OFF);
  set_gain();
}

// update dsp gain potmeter input gain
void set_gain(void) {
  // limit range
  // float db_value = -50.0f * (1.0f - ((float)audio_gain / (float)lv_slider_get_max_value(slider_main_gain)));
  float db_value = -50.0f + (float) audio_gain;
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  dsp.volume(MOD_SINGLE1_GAIN1940ALGNS1_ADDR, db_value);
  xSemaphoreGive(i2cMutex);
}

static void event_output_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_line_out_previous = audio_line_out;
    audio_line_out = lv_slider_get_value(obj);
    if (audio_line_out != audio_line_out_previous) {
      set_output();
    }
    if (audio_rotary_selected_gain) {
      audio_rotary_selected_gain = false;
      set_main_labels(0);
    }
  }
}

// update output
void update_output(void) {
  lv_slider_set_value(slider_main_output, audio_line_out, LV_ANIM_OFF);
  set_output();
  // setting the value does not trigger the value change event, do it manually
  // lv_obj_send_event(slider_main_output, LV_EVENT_VALUE_CHANGED, NULL);
}

// update dsp gain potmeter line output
void set_output(void) {
  // limit range
  // float db_value = -30.0f * (1.0f - ((float)audio_line_out / (float)lv_slider_get_max_value(slider_main_output)));
  float db_value = -50.0f + (float)audio_line_out;
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  dsp.volume(MOD_SINGLE1_2_GAIN1940ALGNS5_ADDR, db_value);
  xSemaphoreGive(i2cMutex);
}

static void event_speaker_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_speaker_previous = audio_speaker;
    audio_speaker = lv_slider_get_value(obj);
    if (audio_speaker != audio_speaker_previous) {
      set_speaker();
    }
    if (!audio_rotary_selected_speaker) {
      audio_rotary_selected_speaker = true;
      set_main_labels(1);
    }
  }
}

// update speaker
void update_speaker(void) {
  lv_slider_set_value(slider_main_speaker, audio_speaker, LV_ANIM_OFF);
  set_speaker();
}

// update dsp gain potmeter speaker
void set_speaker(void) {
  // limit range
  float db_value = -50.0f + (float)audio_speaker;
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  dsp.volume(MOD_SINGLE1_5_GAIN1940ALGNS7_ADDR, db_value);
  xSemaphoreGive(i2cMutex);
}

static void event_headphone_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_headphone_previous = audio_headphone;
    audio_headphone = lv_slider_get_value(obj);
    if (audio_headphone != audio_headphone_previous) {
      set_headphone();
    }
    if (audio_rotary_selected_speaker) {
      audio_rotary_selected_speaker = false;
      set_main_labels(1);
    }
  }
}

// update headphone
void update_headphone(void) {
  lv_slider_set_value(slider_main_headphone, audio_headphone, LV_ANIM_OFF);
  set_headphone();
}

// update dsp gain potmeter headphone or speaker
void set_headphone(void) {
  // limit range
  float db_value = -50.0f + (float)audio_headphone;
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  dsp.volume(MOD_SINGLE1_4_GAIN1940ALGNS8_ADDR, db_value);
  xSemaphoreGive(i2cMutex);
}

// mute the input
void mute_input(boolean mute_on) {
  // bug in lib, needs 0 for mute
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  dsp.mute(MOD_MUTE4_ALG0_MUTEONOFF_ADDR, !mute_on);
  xSemaphoreGive(i2cMutex);
  audio_input_muted = mute_on;
}

// select input 0 = speaker 1 = line
void select_input_source(uint8_t inputchannel) {
  audio_input_selected = inputchannel;
  switch (inputchannel) {
    case 0:
      // speaker input
      xSemaphoreTake(i2cMutex, portMAX_DELAY);
      dsp.mux(MOD_NX1_1_MONOSWSLEW_ADDR, 0, 0);
      xSemaphoreGive(i2cMutex);
      break;

    case 1:
      // line input
      xSemaphoreTake(i2cMutex, portMAX_DELAY);
      dsp.mux(MOD_NX1_1_MONOSWSLEW_ADDR, 1, 0);
      xSemaphoreGive(i2cMutex);
      break;

    case 2:
      // sine wave test signal
      xSemaphoreTake(i2cMutex, portMAX_DELAY);
      dsp.mux(MOD_NX1_1_MONOSWSLEW_ADDR, 2, 0);
      xSemaphoreGive(i2cMutex);
      break;
  }
}

// input stage amplifier gain
void set_input_gain(int32_t in_gain) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  dsp.gain(MOD_GAIN1_GAIN1940ALGNS2_ADDR, (int32_t) in_gain);
  xSemaphoreGive(i2cMutex);
}

// line output stage amplifier gain
void set_line_out_gain(int32_t out_gain) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  dsp.gain(MOD_GAIN1_4_GAIN1940ALGNS3_ADDR, (int32_t) out_gain);
  xSemaphoreGive(i2cMutex);
}

// speaker output stage amplifier gain
void set_hp_out_gain(int32_t out_gain) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  dsp.gain(MOD_GAIN1_3_GAIN1940ALGNS4_ADDR, (int32_t) out_gain);
  xSemaphoreGive(i2cMutex);
}

void set_main_labels(uint8_t labelset) {
  lv_color_t selected_color = lv_palette_main(LV_PALETTE_GREEN);
  lv_color_t nonselected_color = lv_color_white();
  if (labelset == 0) {
    lv_obj_set_style_text_color(lbl_main_gain, (audio_rotary_selected_gain) ? selected_color : nonselected_color, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_main_output, (!audio_rotary_selected_gain) ? selected_color : nonselected_color, LV_PART_MAIN);
  }
  else {
    lv_obj_set_style_text_color(lbl_main_speaker, (audio_rotary_selected_speaker) ? selected_color : nonselected_color, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_main_headphone, (!audio_rotary_selected_speaker) ? selected_color : nonselected_color, LV_PART_MAIN);
  }
}

// set the mode for the esp32, routing or not
void set_esp32_mode(uint8_t espmode) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  switch (espmode) {
    case 0:
      // nothing, just input i2s signal for scope or spectrum
      // 0 = input to esp32 before eq 1 = input to esp32 after eq
      dsp.mux(MOD_NX1_2_2_MONOSWSLEW_ADDR, 1, 0);
      // 0 = normal 1 = output stage connects directly from esp32
      dsp.mux(MOD_NX1_2_4_MONOSWSLEW_ADDR, 0, 0);
      // 0 = normal 1 = input to eq from esp32
      dsp.mux(MOD_NX1_2_3_MONOSWSLEW_ADDR, 0, 0);
      break;

    case 1:
      // put esp32 before eq
      // 0 = input to esp32 before eq 1 = input to esp32 after eq
      dsp.mux(MOD_NX1_2_2_MONOSWSLEW_ADDR, 0, 0);
      // 0 = normal 1 = output stage connects directly from esp32
      dsp.mux(MOD_NX1_2_4_MONOSWSLEW_ADDR, 0, 0);
      // 0 = normal 1 = input to eq from esp32
      dsp.mux(MOD_NX1_2_3_MONOSWSLEW_ADDR, 1, 0);
      break;

    case 2:
      // put esp32 after eq
      // 0 = input to esp32 before eq 1 = input to esp32 after eq
      dsp.mux(MOD_NX1_2_2_MONOSWSLEW_ADDR, 1, 0);
      // 0 = normal 1 = output stage connects directly from esp32
      dsp.mux(MOD_NX1_2_4_MONOSWSLEW_ADDR, 1, 0);
      // 0 = normal 1 = input to eq from esp32
      dsp.mux(MOD_NX1_2_3_MONOSWSLEW_ADDR, 0, 0);
      break;

    case 3:
      // test mode with gain adjust
      // 0 = input to esp32 before eq 1 = input to esp32 after eq
      dsp.mux(MOD_NX1_2_2_MONOSWSLEW_ADDR, 2, 0);
      // 0 = normal 1 = output stage connects directly from esp32
      dsp.mux(MOD_NX1_2_4_MONOSWSLEW_ADDR, 0, 0);
      // 0 = normal 1 = input to eq from esp32
      dsp.mux(MOD_NX1_2_3_MONOSWSLEW_ADDR, 0, 0);
      break;

    case 4:
      // test mode direct from sine generator
      // 0 = input to esp32 before eq 1 = input to esp32 after eq
      dsp.mux(MOD_NX1_2_2_MONOSWSLEW_ADDR, 3, 0);
      // 0 = normal 1 = output stage connects directly from esp32
      dsp.mux(MOD_NX1_2_4_MONOSWSLEW_ADDR, 0, 0);
      // 0 = normal 1 = input to eq from esp32
      dsp.mux(MOD_NX1_2_3_MONOSWSLEW_ADDR, 0, 0);
      break;
  }
  xSemaphoreGive(i2cMutex);
}

// mute after filters to avoid click when adjusting filters
void mute_filters(boolean mute_on) {
  // bug in lib, needs 0 for mute
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  dsp.mute(MOD_MUTE4_2_ALG0_MUTEONOFF_ADDR, !mute_on);
  xSemaphoreGive(i2cMutex);
}

// select hi/lo filters
void select_hi_lo_filters(uint8_t filter) {
  if (filter > 3) {
    filter = filter - 4;
  }
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  switch (filter) {
    case 0:
      // 1 filter
      dsp.mux(MOD_NX1_2_5_MONOSWSLEW_ADDR, 1, 0);
      break;

    case 1:
      // 2 filters
      dsp.mux(MOD_NX1_2_5_MONOSWSLEW_ADDR, 2, 0);
      break;

    case 2:
      // 3 filters
      dsp.mux(MOD_NX1_2_5_MONOSWSLEW_ADDR, 3, 0);
      break;

    case 3:
      // 4 filters
      dsp.mux(MOD_NX1_2_5_MONOSWSLEW_ADDR, 4, 0);
      break;
  }
  xSemaphoreGive(i2cMutex);
}

void mute_output(boolean mute_on) {
  // bug in lib, needs 0 for mute
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  dsp.mute(MOD_MUTE3_2_MUTENOSLEWALG1MUTE_ADDR, !mute_on);
  xSemaphoreGive(i2cMutex);
}

void show_tx_led(boolean show_msg) {
  if (show_msg) {
    lv_obj_clear_flag(led_main_tx, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lbl_main_tx, LV_OBJ_FLAG_HIDDEN);
  }
  else {
    lv_obj_add_flag(led_main_tx, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lbl_main_tx, LV_OBJ_FLAG_HIDDEN);
  }
}

// notch screen

void increment_notch_frequency0_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_notch_frequency_0);
  }
}

static void decrement_notch_frequency0_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_notch_frequency_0);
  }
}

void increment_notch_frequency1_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_notch_frequency_1);
  }
}

static void decrement_notch_frequency1_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_notch_frequency_1);
  }
}

void increment_notch_frequency2_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_notch_frequency_2);
  }
}

static void decrement_notch_frequency2_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_notch_frequency_2);
  }
}

void increment_notch_width0_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_notch_width_0);
  }
}

static void decrement_notch_width0_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_notch_width_0);
  }
}

void increment_notch_width1_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_notch_width_1);
  }
}

static void decrement_notch_width1_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_notch_width_1);
  }
}

void increment_notch_width2_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_notch_width_2);
  }
}

static void decrement_notch_width2_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_notch_width_2);
  }
}

void inc_notch_frequency(uint8_t selected_no) {
  // Array of objects
  lv_obj_t* spinboxes[] = {spinbox_notch_frequency_0, spinbox_notch_frequency_1, spinbox_notch_frequency_2};

  // Ensure selected_no is within range
  if (selected_no >= sizeof(spinboxes) / sizeof(spinboxes[0])) {
    return; // Invalid index, do nothing
  }
  lv_spinbox_increment(spinboxes[selected_no]);
}

void dec_notch_frequency(uint8_t selected_no) {
  // Array of objects
  lv_obj_t* spinboxes[] = {spinbox_notch_frequency_0, spinbox_notch_frequency_1, spinbox_notch_frequency_2};

  // Ensure selected_no is within range
  if (selected_no >= sizeof(spinboxes) / sizeof(spinboxes[0])) {
    return; // Invalid index, do nothing
  }
  lv_spinbox_decrement(spinboxes[selected_no]);
}

void inc_notch_width(uint8_t selected_no) {
  // Array of objects
  lv_obj_t* spinboxes[] = {spinbox_notch_width_0, spinbox_notch_width_1, spinbox_notch_width_2};

  // Ensure selected_no is within range
  if (selected_no >= sizeof(spinboxes) / sizeof(spinboxes[0])) {
    return; // Invalid index, do nothing
  }
  lv_spinbox_increment(spinboxes[selected_no]);
}

void dec_notch_width(uint8_t selected_no) {
  // Array of objects
  lv_obj_t* spinboxes[] = {spinbox_notch_width_0, spinbox_notch_width_1, spinbox_notch_width_2};

  // Ensure selected_no is within range
  if (selected_no >= sizeof(spinboxes) / sizeof(spinboxes[0])) {
    return; // Invalid index, do nothing
  }
  lv_spinbox_decrement(spinboxes[selected_no]);
}

void set_notch_labels(uint8_t label_no) {
  lv_color_t selected_color = lv_palette_main(LV_PALETTE_GREEN);
  lv_color_t nonselected_color = lv_color_white();

  // Array of label objects
  lv_obj_t* labels[] = {lbl_notch0, lbl_notch1, lbl_notch2};

  // Ensure label_no is within range
  if (label_no >= sizeof(labels) / sizeof(labels[0])) {
    return; // Invalid index, do nothing
  }

  // Set colors for all labels
  for (uint8_t i = 0; i < sizeof(labels) / sizeof(labels[0]); i++) {
    lv_obj_set_style_text_color(labels[i], (i == label_no) ? selected_color : nonselected_color, LV_PART_MAIN);
  }
}

static void event_notch_frequency0_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (!audio_notch_lock_updates) {
      audio_notch_lock_updates = true;
      filter_notch_freq0 = lv_spinbox_get_value(spinbox_notch_frequency_0);
      notch_check_set(0, filter_notch_freq0, filter_notch_bandwidth0, filter_notch_on0, true);
      if (audio_selected_notch != 0) {
        audio_selected_notch = 0;
        set_notch_labels(audio_selected_notch);
        set_notch_cursors(audio_selected_notch);
      }
      audio_notch_lock_updates = false;
    }
  }
}

static void event_notch_frequency1_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (!audio_notch_lock_updates) {
      audio_notch_lock_updates = true;
      filter_notch_freq1 = lv_spinbox_get_value(spinbox_notch_frequency_1);
      notch_check_set(1, filter_notch_freq1, filter_notch_bandwidth1, filter_notch_on1, true);
      if (audio_selected_notch != 1) {
        audio_selected_notch = 1;
        set_notch_labels(audio_selected_notch);
        set_notch_cursors(audio_selected_notch);
      }
      audio_notch_lock_updates = false;
    }
  }
}

static void event_notch_frequency2_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (!audio_notch_lock_updates) {
      audio_notch_lock_updates = true;
      filter_notch_freq2 = lv_spinbox_get_value(spinbox_notch_frequency_2);
      notch_check_set(2, filter_notch_freq2, filter_notch_bandwidth2, filter_notch_on2, true);
      if (audio_selected_notch != 2) {
        audio_selected_notch = 2;
        set_notch_labels(audio_selected_notch);
        set_notch_cursors(audio_selected_notch);
      }
      audio_notch_lock_updates = false;
    }
  }
}

static void event_notch_width0_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (!audio_notch_lock_updates) {
      audio_notch_lock_updates = true;
      filter_notch_bandwidth0 = lv_spinbox_get_value(spinbox_notch_width_0);
      notch_check_set(0, filter_notch_freq0, filter_notch_bandwidth0, filter_notch_on0, false);
      if (audio_selected_notch != 0) {
        audio_selected_notch = 0;
        set_notch_labels(audio_selected_notch);
        set_notch_cursors(audio_selected_notch);
      }
      audio_notch_lock_updates = false;
    }
  }
}

static void event_notch_width1_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (!audio_notch_lock_updates) {
      audio_notch_lock_updates = true;
      filter_notch_bandwidth1 = lv_spinbox_get_value(spinbox_notch_width_1);

      notch_check_set(1, filter_notch_freq1, filter_notch_bandwidth1, filter_notch_on1, false);
      if (audio_selected_notch != 1) {
        audio_selected_notch = 1;
        set_notch_labels(audio_selected_notch);
        set_notch_cursors(audio_selected_notch);
      }
      audio_notch_lock_updates = false;
    }
  }
}

static void event_notch_width2_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (!audio_notch_lock_updates) {
      audio_notch_lock_updates = true;
      filter_notch_bandwidth2 = lv_spinbox_get_value(spinbox_notch_width_2);

      notch_check_set(2, filter_notch_freq2, filter_notch_bandwidth2, filter_notch_on2, false);
      if (audio_selected_notch != 2) {
        audio_selected_notch = 2;
        set_notch_labels(audio_selected_notch);
        set_notch_cursors(audio_selected_notch);
      }
      audio_notch_lock_updates = false;
    }
  }
}

static void event_notch_switch_0(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (!audio_notch_lock_updates) {
      audio_notch_lock_updates = true;
      filter_notch_on0 = lv_obj_get_state(obj) & LV_STATE_CHECKED;

      notch_check_set(0, filter_notch_freq0, filter_notch_bandwidth0, filter_notch_on0, true);

      if (audio_selected_notch != 0) {
        audio_selected_notch = 0;
        set_notch_labels(audio_selected_notch);
        set_notch_cursors(audio_selected_notch);
      }
      audio_notch_lock_updates = false;
    }
  }
}

static void event_notch_switch_1(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (!audio_notch_lock_updates) {
      audio_notch_lock_updates = true;
      filter_notch_on1 = lv_obj_get_state(obj) & LV_STATE_CHECKED;
      notch_check_set(1, filter_notch_freq1, filter_notch_bandwidth1, filter_notch_on1, true);
      if (audio_selected_notch != 1) {
        audio_selected_notch = 1;
        set_notch_labels(audio_selected_notch);
        set_notch_cursors(audio_selected_notch);
      }
      audio_notch_lock_updates = false;
    }
  }
}

static void event_notch_switch_2(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (!audio_notch_lock_updates) {
      audio_notch_lock_updates = true;
      filter_notch_on2 = lv_obj_get_state(obj) & LV_STATE_CHECKED;
      notch_check_set(2, filter_notch_freq2, filter_notch_bandwidth2, filter_notch_on2, true);
      if (audio_selected_notch != 2) {
        audio_selected_notch = 2;
        set_notch_labels(audio_selected_notch);
        set_notch_cursors(audio_selected_notch);
      }
      audio_notch_lock_updates = false;
    }
  }
}

void update_notch_filters(void) {
  if (filter_notch_on0) {
    lv_obj_add_state(cb_notch_on0, LV_STATE_CHECKED);
  }
  else {
    lv_obj_clear_state(cb_notch_on0, LV_STATE_CHECKED);
  }
  if (filter_notch_on1) {
    lv_obj_add_state(cb_notch_on1, LV_STATE_CHECKED);
  }
  else {
    lv_obj_clear_state(cb_notch_on1, LV_STATE_CHECKED);
  }
  if (filter_notch_on2) {
    lv_obj_add_state(cb_notch_on2, LV_STATE_CHECKED);
  }
  else {
    lv_obj_clear_state(cb_notch_on2, LV_STATE_CHECKED);
  }
  lv_spinbox_set_value(spinbox_notch_frequency_0, filter_notch_freq0);
  lv_spinbox_set_value(spinbox_notch_frequency_1, filter_notch_freq1);
  lv_spinbox_set_value(spinbox_notch_frequency_2, filter_notch_freq2);

  lv_spinbox_set_value(spinbox_notch_width_0, filter_notch_bandwidth0);
  lv_spinbox_set_value(spinbox_notch_width_1, filter_notch_bandwidth1);
  lv_spinbox_set_value(spinbox_notch_width_2, filter_notch_bandwidth2);
}

// check parameters and set filter
void notch_check_set(uint8_t filter_no,  uint16_t filter_centerfrequency, uint16_t filter_width, boolean filter_on, boolean doupdate) {
  // calc Q and check if Q is within range
  boolean adjust_bw = false;
  float calc_q = filter_centerfrequency / filter_width;
  // check limits
  if (calc_q < 0.5f) {
    float new_bw = filter_centerfrequency / 0.5;
    filter_width = (uint16_t) new_bw;
    adjust_bw = true;
  }
  if (calc_q > 50.0f) {
    float new_bw = filter_centerfrequency / 50;
    filter_width = (uint16_t) new_bw;
    adjust_bw = true;
  }
  // no update if called from notch width event otherwise stack canary error, infinite loop
  if (adjust_bw and doupdate) {
    switch (filter_no) {
      case 0:
        filter_notch_bandwidth0 = filter_width;
        lv_spinbox_set_value(spinbox_notch_width_0, filter_width);
        break;

      case 1:
        filter_notch_bandwidth1 = filter_width;
        lv_spinbox_set_value(spinbox_notch_width_1, filter_width);
        break;

      case 2:
        filter_notch_bandwidth2 = filter_width;
        lv_spinbox_set_value(spinbox_notch_width_2, filter_width);
        break;
    }
  }
  set_notch_filter(filter_no, filter_on, filter_centerfrequency, filter_width);
}

void set_notch_cursors(uint8_t selected_no) {
  switch (selected_no) {
    case 0:
      spinbox_show_cursor(spinbox_notch_frequency_0, true);
      spinbox_show_cursor(spinbox_notch_frequency_1, false);
      spinbox_show_cursor(spinbox_notch_frequency_2, false);
      spinbox_show_cursor(spinbox_notch_width_0, true);
      spinbox_show_cursor(spinbox_notch_width_1, false);
      spinbox_show_cursor(spinbox_notch_width_2, false);
      break;

    case 1:
      spinbox_show_cursor(spinbox_notch_frequency_0, false);
      spinbox_show_cursor(spinbox_notch_frequency_1, true);
      spinbox_show_cursor(spinbox_notch_frequency_2, false);
      spinbox_show_cursor(spinbox_notch_width_0, false);
      spinbox_show_cursor(spinbox_notch_width_1, true);
      spinbox_show_cursor(spinbox_notch_width_2, false);
      break;

    case 2:
      spinbox_show_cursor(spinbox_notch_frequency_0, false);
      spinbox_show_cursor(spinbox_notch_frequency_1, false);
      spinbox_show_cursor(spinbox_notch_frequency_2, true);
      spinbox_show_cursor(spinbox_notch_width_0, false);
      spinbox_show_cursor(spinbox_notch_width_1, false);
      spinbox_show_cursor(spinbox_notch_width_2, true);
      break;
  }
}

// calculate and set notch filter
void set_notch_filter(uint8_t filter_number, boolean filter_on, uint16_t filter_centerfrequency, uint16_t filter_width ) {
  mute_filters(true);
  if (!filter_on) {
    // write deactivate parameters
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    switch (filter_number) {
      case 0:
        dsp.safeload_writeRegister(MOD_GENFILTER1_ALG0_STAGE0_B0_ADDR, 0x00800000, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_ALG0_STAGE0_B1_ADDR, 0x00000000, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_ALG0_STAGE0_B2_ADDR, 0x00000000, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_ALG0_STAGE0_A0_ADDR, 0x00000000, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_ALG0_STAGE0_A1_ADDR, 0x00000000, true);
        break;

      case 1:
        dsp.safeload_writeRegister(MOD_GENFILTER1_2_ALG0_STAGE0_B0_ADDR, 0x00800000, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_2_ALG0_STAGE0_B1_ADDR, 0x00000000, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_2_ALG0_STAGE0_B2_ADDR, 0x00000000, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_2_ALG0_STAGE0_A0_ADDR, 0x00000000, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_2_ALG0_STAGE0_A1_ADDR, 0x00000000, true);
        break;

      case 2:
        dsp.safeload_writeRegister(MOD_GENFILTER1_3_ALG0_STAGE0_B0_ADDR, 0x00800000, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_3_ALG0_STAGE0_B1_ADDR, 0x00000000, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_3_ALG0_STAGE0_B2_ADDR, 0x00000000, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_3_ALG0_STAGE0_A0_ADDR, 0x00000000, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_3_ALG0_STAGE0_A1_ADDR, 0x00000000, true);
        break;
    }
    xSemaphoreGive(i2cMutex);
  }
  else {
    // calculate Q = Fo/BW
    float calc_q = filter_centerfrequency / filter_width;

    // check limits
    if (calc_q < 0.5f) calc_q = 0.5f;
    if (calc_q > 50.0f) calc_q = 50.0f;

    // parameters
    float p_boost = 1.0f;
    float p_fs = 48000.0f;
    float p_gain = 0.01f;

    // intermediate values
    float i_Ax = powf(10.0f, p_boost / 40.0f);
    float i_omega = (PI * 2.0f * (float) filter_centerfrequency) / p_fs;
    float i_sn = sinf(i_omega);
    float i_cs = cosf(i_omega);
    float i_alpha = i_sn / (2.0f * calc_q);
    float i_a0 = 1.0f + (i_alpha / i_Ax);
    float i_gainlinear = powf(10.0f, p_gain / 20.0f) / i_a0;
    float i_dtomega = i_omega / calc_q;
    float i_b = 1.0f / (1.0f + tanf(i_dtomega / 2.0f));

    // final values
    float calc_B0 = i_gainlinear * i_b * i_a0;
    float calc_B1 = i_gainlinear * i_a0 * (-2.0f * i_b * i_cs);
    float calc_B2 = calc_B0;
    float calc_A0 = -2.0f * i_b * i_cs;
    float calc_A1 = (2.0f * i_b) - 1.0f;

    // write parameters
    uint8_t B0_data[5], B1_data[5], B2_data[5], A0_data[5], A1_data[5];

    dsp.floatToFixed(calc_B0, B0_data);
    dsp.floatToFixed(calc_B1, B1_data);
    dsp.floatToFixed(calc_B2, B2_data);
    // negate result, same as sigmadsp does
    dsp.floatToFixed(-calc_A0, A0_data);
    // negate result, same as sigmadsp does
    dsp.floatToFixed(-calc_A1, A1_data);

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    switch (filter_number) {
      case 0:
        dsp.safeload_writeRegister(MOD_GENFILTER1_ALG0_STAGE0_B0_ADDR, B0_data, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_ALG0_STAGE0_B1_ADDR, B1_data, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_ALG0_STAGE0_B2_ADDR, B2_data, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_ALG0_STAGE0_A0_ADDR, A0_data, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_ALG0_STAGE0_A1_ADDR, A1_data, true);
        break;
      case 1:
        dsp.safeload_writeRegister(MOD_GENFILTER1_2_ALG0_STAGE0_B0_ADDR, B0_data, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_2_ALG0_STAGE0_B1_ADDR, B1_data, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_2_ALG0_STAGE0_B2_ADDR, B2_data, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_2_ALG0_STAGE0_A0_ADDR, A0_data, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_2_ALG0_STAGE0_A1_ADDR, A1_data, true);
        break;
      case 2:
        dsp.safeload_writeRegister(MOD_GENFILTER1_3_ALG0_STAGE0_B0_ADDR, B0_data, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_3_ALG0_STAGE0_B1_ADDR, B1_data, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_3_ALG0_STAGE0_B2_ADDR, B2_data, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_3_ALG0_STAGE0_A0_ADDR, A0_data, false);
        dsp.safeload_writeRegister(MOD_GENFILTER1_3_ALG0_STAGE0_A1_ADDR, A1_data, true);
        break;
    }
    xSemaphoreGive(i2cMutex);
  }
  mute_filters(false);
}

// filter screen

void set_filter_cursors(boolean selected_hilomode) {
  if (selected_hilomode) {
    spinbox_show_cursor(spinbox_filter_highpass, true);
    spinbox_show_cursor(spinbox_filter_lowpass, true);
    spinbox_show_cursor(spinbox_filter_bandpass, false);
    spinbox_show_cursor(spinbox_filter_bandwidth, false);
  }
  else {
    spinbox_show_cursor(spinbox_filter_highpass, false);
    spinbox_show_cursor(spinbox_filter_lowpass, false);
    spinbox_show_cursor(spinbox_filter_bandpass, true);
    spinbox_show_cursor(spinbox_filter_bandwidth, true);
  }
}

static void event_filter_highpass_frequency_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (!audio_filter_lock_updates) {
      audio_filter_lock_updates = true;
      filter_highpass_freq = lv_spinbox_get_value(spinbox_filter_highpass);
      boolean update_lowpass = false;
      // lowpass must be at least 50hz higher than hipass
      if (lv_spinbox_get_value(spinbox_filter_lowpass) < lv_spinbox_get_value(spinbox_filter_highpass) + 50) {
        lv_spinbox_set_value(spinbox_filter_lowpass, lv_spinbox_get_value(spinbox_filter_highpass) + 50);
        filter_lowpass_freq = lv_spinbox_get_value(spinbox_filter_lowpass);
        update_lowpass = true;
      }
      // recalc bandpass and bandwidth
      lv_spinbox_set_value(spinbox_filter_bandpass, ((lv_spinbox_get_value(spinbox_filter_lowpass) - lv_spinbox_get_value(spinbox_filter_highpass)) / 2) + lv_spinbox_get_value(spinbox_filter_highpass));
      filter_bandpass_freq = lv_spinbox_get_value(spinbox_filter_bandpass);
      lv_spinbox_set_value(spinbox_peak_bandpass, lv_spinbox_get_value(spinbox_filter_bandpass));
      filter_peak_bandpass_freq = lv_spinbox_get_value(spinbox_peak_bandpass);
      lv_spinbox_set_value(spinbox_filter_bandwidth, (lv_spinbox_get_value(spinbox_filter_lowpass) - lv_spinbox_get_value(spinbox_filter_highpass)));
      filter_bandwidth = lv_spinbox_get_value(spinbox_filter_bandwidth);
      set_highpass_filter(audio_filter_mode);
      if (update_lowpass) {
        set_lowpass_filter(audio_filter_mode);
      }
      if (!audio_filter_mode_hi_lo) {
        audio_filter_mode_hi_lo = true;
        set_filter_cursors(audio_filter_mode_hi_lo);
      }
      audio_spectrum_bandpasstrigger = true;
      audio_filter_lock_updates = false;
    }
  }
}

static void event_filter_lowpass_frequency_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (!audio_filter_lock_updates) {
      audio_filter_lock_updates = true;
      filter_lowpass_freq = lv_spinbox_get_value(spinbox_filter_lowpass);
      boolean update_highpass = false;
      // hi must be at least 50hz lower than lo
      if (lv_spinbox_get_value(spinbox_filter_lowpass) < lv_spinbox_get_value(spinbox_filter_highpass) + 50) {
        lv_spinbox_set_value(spinbox_filter_highpass, lv_spinbox_get_value(spinbox_filter_lowpass) - 50);
        filter_highpass_freq = lv_spinbox_get_value(spinbox_filter_highpass);
        update_highpass = true;
      }
      // recalc bandpass and bandwidth
      lv_spinbox_set_value(spinbox_filter_bandpass, ((lv_spinbox_get_value(spinbox_filter_lowpass) - lv_spinbox_get_value(spinbox_filter_highpass)) / 2) + lv_spinbox_get_value(spinbox_filter_highpass));
      filter_bandpass_freq = lv_spinbox_get_value(spinbox_filter_bandpass);
      lv_spinbox_set_value(spinbox_peak_bandpass, lv_spinbox_get_value(spinbox_filter_bandpass));
      filter_peak_bandpass_freq = lv_spinbox_get_value(spinbox_peak_bandpass);
      lv_spinbox_set_value(spinbox_filter_bandwidth, (lv_spinbox_get_value(spinbox_filter_lowpass) - lv_spinbox_get_value(spinbox_filter_highpass)));
      filter_bandwidth = lv_spinbox_get_value(spinbox_filter_bandwidth);
      set_lowpass_filter(audio_filter_mode);
      if (update_highpass) {
        set_highpass_filter(audio_filter_mode);
      }
      if (!audio_filter_mode_hi_lo) {
        audio_filter_mode_hi_lo = true;
        set_filter_cursors(audio_filter_mode_hi_lo);
      }
      audio_spectrum_bandpasstrigger = true;
      audio_filter_lock_updates = false;
    }
  }
}

static void event_filter_bandpass_frequency_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (!audio_filter_lock_updates) {
      audio_filter_lock_updates = true;

      filter_bandpass_freq = lv_spinbox_get_value(spinbox_filter_bandpass);
      // check if new value makes the hi setting out of range
      if ((lv_spinbox_get_value(spinbox_filter_bandpass) - (lv_spinbox_get_value(spinbox_filter_bandwidth) / 2)) < 60) {
        // adjust bandwidth
        lv_spinbox_set_value(spinbox_filter_bandwidth, (lv_spinbox_get_value(spinbox_filter_bandpass) - 60) * 2);

        filter_bandwidth = lv_spinbox_get_value(spinbox_filter_bandwidth);
      }
      lv_spinbox_set_value(spinbox_filter_highpass, lv_spinbox_get_value(spinbox_filter_bandpass) - (lv_spinbox_get_value(spinbox_filter_bandwidth) / 2));
      filter_highpass_freq = lv_spinbox_get_value(spinbox_filter_highpass);
      lv_spinbox_set_value(spinbox_filter_lowpass, lv_spinbox_get_value(spinbox_filter_highpass) + lv_spinbox_get_value(spinbox_filter_bandwidth));
      filter_lowpass_freq = lv_spinbox_get_value(spinbox_filter_lowpass);
      set_highpass_filter(audio_filter_mode);
      set_lowpass_filter(audio_filter_mode);
      if (audio_filter_mode_hi_lo) {
        audio_filter_mode_hi_lo = false;
        set_filter_cursors(audio_filter_mode_hi_lo);
      }
      // set other center freq in peaking screen
      lv_spinbox_set_value(spinbox_peak_bandpass, lv_spinbox_get_value(spinbox_filter_bandpass));
      filter_peak_bandpass_freq = lv_spinbox_get_value(spinbox_peak_bandpass);
      set_peak_filter(filter_peak_on, filter_peak_bandpass_freq, filter_peak_bandwidth, true, 0);
      audio_spectrum_bandpasstrigger = true;
      audio_filter_lock_updates = false;
    }
  }
}

static void event_peak_bandpass_frequency_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    filter_peak_bandpass_freq = lv_spinbox_get_value(obj);
    // this will trigger the other functions
    if (!audio_filter_lock_updates) {
      lv_spinbox_set_value(spinbox_filter_bandpass, lv_spinbox_get_value(spinbox_peak_bandpass));
      filter_bandpass_freq = lv_spinbox_get_value(spinbox_filter_bandpass);
    }
    audio_spectrum_bandpasstrigger = true;
  }
}

static void event_filter_bandpass_width_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    if (!audio_filter_lock_updates) {
      audio_filter_lock_updates = true;
      filter_bandwidth = lv_spinbox_get_value(spinbox_filter_bandwidth);
      // check if new value makes the hi setting out of range
      if ((lv_spinbox_get_value(spinbox_filter_bandpass) - (lv_spinbox_get_value(spinbox_filter_bandwidth) / 2)) < 60) {
        // adjust bandpass
        lv_spinbox_set_value(spinbox_filter_bandpass, 60 + (lv_spinbox_get_value(spinbox_filter_bandwidth) / 2) );
        filter_bandpass_freq = lv_spinbox_get_value(spinbox_filter_bandpass);
        lv_spinbox_set_value(spinbox_peak_bandpass, lv_spinbox_get_value(spinbox_filter_bandpass));
        filter_peak_bandpass_freq = lv_spinbox_get_value(spinbox_peak_bandpass);
      }
      lv_spinbox_set_value(spinbox_filter_highpass, lv_spinbox_get_value(spinbox_filter_bandpass) - (lv_spinbox_get_value(spinbox_filter_bandwidth) / 2));
      filter_highpass_freq = lv_spinbox_get_value(spinbox_filter_highpass);
      lv_spinbox_set_value(spinbox_filter_lowpass, lv_spinbox_get_value(spinbox_filter_highpass) + lv_spinbox_get_value(spinbox_filter_bandwidth));
      filter_lowpass_freq = lv_spinbox_get_value(spinbox_filter_lowpass);
      set_highpass_filter(audio_filter_mode);
      set_lowpass_filter(audio_filter_mode);
      if (audio_filter_mode_hi_lo) {
        audio_filter_mode_hi_lo = false;
        set_filter_cursors(audio_filter_mode_hi_lo);
      }
      audio_spectrum_bandpasstrigger = true;
      audio_filter_lock_updates = false;
    }
  }
}

void increment_filter_high_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_filter_highpass);
  }
}

static void decrement_filter_high_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_filter_highpass);
  }
}

void increment_filter_low_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_filter_lowpass);
  }
}

static void decrement_filter_low_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_filter_lowpass);
  }
}

void increment_filter_bp_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_filter_bandpass);
  }
}

static void decrement_filter_bp_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_filter_bandpass);
  }
}

void update_filters(void) {
  lv_dropdown_set_selected(dropdown_filter_mode, audio_filter_mode);
  lv_spinbox_set_value(spinbox_filter_highpass, filter_highpass_freq);
  lv_spinbox_set_value(spinbox_filter_lowpass, filter_lowpass_freq);
  lv_spinbox_set_value(spinbox_filter_bandpass, filter_bandpass_freq);
  lv_spinbox_set_value(spinbox_filter_bandwidth, filter_bandwidth);
}

void increment_peak_bp_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_peak_bandpass);
  }
}

static void decrement_peak_bp_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_peak_bandpass);
  }
}

void increment_filter_bw_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_filter_bandwidth);
  }
}

static void decrement_filter_bw_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_filter_bandwidth);
  }
}

// calculate and set highpass filter
void set_highpass_filter(uint8_t filtertype) {
  // type 0-3 butterworth
  // type 4-7 chebyshev

  mute_filters(true);

  float calc_A0 = 0;
  float calc_A1 = 0;
  float calc_B0 = 0;
  float calc_B1 = 0;
  float calc_B2 = 0;

  if (filtertype < 4) {
    uint16_t filter_frequency = lv_spinbox_get_value(spinbox_filter_highpass);
    // FILTER_2ND_BUTTERWORTH_HIGH_PASS:
    // parameters
    float p_boost = 1.0f;
    float p_fs = 48000.0f;
    float p_gain = 0.01f;

    // intermediate values
    float i_Ax = powf(10.0f, p_boost / 40.0f);
    float i_omega = (PI * 2.0f * (float) filter_frequency) / p_fs;
    float i_sn = sinf(i_omega);
    float i_cs = cosf(i_omega);
    float i_alpha = i_sn / (2.0 * (1.0 / sqrt(2.0)));
    float i_a0 = 1.0f + i_alpha;

    // final values
    calc_A0 = -((0.0 - 2.0 * i_cs) / i_a0);
    calc_A1 = -((1.0 - i_alpha) / i_a0);
    calc_B1 = (0.0 - (1.0 + i_cs)) / i_a0 * pow(10.0, p_gain / 20.0);
    calc_B0 = (0.0 - calc_B1) / 2.0;
    calc_B2 = calc_B0;

  }
  else {
    // FILTER_2ND_CHEBYSHEV_HIGH_PASS
    uint16_t filter_frequency = lv_spinbox_get_value(spinbox_filter_highpass);

    // parameters
    float p_ripple = 0.1f;
    float p_fs = 48000.0f;
    float p_gain = 0.01f;

    // intermediate values
    float i_omega = 1.0 / tan(PI * 2.0f * (float) filter_frequency / p_fs / 2.0);
    float i_num = sqrt(pow(10.0, 0.1 * p_ripple) - 1.0);
    float i_num1 = pow(i_num, 2.0);
    float i_num3 = i_omega * sinh(0.5 * log(1.0 / i_num + sqrt(1.0 / i_num1 + 1.0)));
    float i_num4 = PI * 3.0 / 4.0;
    float i_Ax = i_omega * sin(i_num4);
    float i_num8 = pow(i_Ax, 2.0);
    float i_num9 = pow(i_num3, 2.0);
    float i_num5 = sqrt(1.0 / (1.0 + i_num1));
    float i_num6 = 1.0 - 2.0 * i_num3 * cos(i_num4) + i_num9 + i_num8;
    float i_num7 = (i_num9 + i_num8) / i_num6;

    // final values
    calc_A0 = -(-2.0 * (i_num9 + i_num8 - 1.0) / i_num6);
    calc_A1 = -((1.0 + 2.0 * i_num3 * cos(i_num4) + i_num9 + i_num8) / i_num6);
    calc_B0 = i_num5 * i_num7 * pow(10.0, p_gain / 20.0);
    calc_B1 = (0.0 - calc_B0) * 2.0;
    calc_B2 = calc_B0;
  }


  // same parameters for all filters
  uint8_t B0_data[5], B1_data[5], B2_data[5], A0_data[5], A1_data[5];
  dsp.floatToFixed(calc_B0, B0_data);
  dsp.floatToFixed(calc_B1, B1_data);
  dsp.floatToFixed(calc_B2, B2_data);
  dsp.floatToFixed(calc_A0, A0_data);
  dsp.floatToFixed(calc_A1, A1_data);

  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  // filter 1 allways
  dsp.safeload_writeRegister(MOD_GENFILTER1_4_ALG0_STAGE0_B0_ADDR, B0_data, false);
  dsp.safeload_writeRegister(MOD_GENFILTER1_4_ALG0_STAGE0_B1_ADDR, B1_data, false);
  dsp.safeload_writeRegister(MOD_GENFILTER1_4_ALG0_STAGE0_B2_ADDR, B2_data, false);
  dsp.safeload_writeRegister(MOD_GENFILTER1_4_ALG0_STAGE0_A0_ADDR, A0_data, false);
  dsp.safeload_writeRegister(MOD_GENFILTER1_4_ALG0_STAGE0_A1_ADDR, A1_data, true);
  if ((filtertype > 0 and filtertype < 4) or (filtertype > 0 and filtertype < 4)) {
    dsp.safeload_writeRegister(MOD_GENFILTER1_6_ALG0_STAGE0_B0_ADDR, B0_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_6_ALG0_STAGE0_B1_ADDR, B1_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_6_ALG0_STAGE0_B2_ADDR, B2_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_6_ALG0_STAGE0_A0_ADDR, A0_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_6_ALG0_STAGE0_A1_ADDR, A1_data, true);
  }
  if ((filtertype > 1 and filtertype < 4) or (filtertype > 1 and filtertype < 4)) {
    dsp.safeload_writeRegister(MOD_GENFILTER1_8_ALG0_STAGE0_B0_ADDR, B0_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_8_ALG0_STAGE0_B1_ADDR, B1_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_8_ALG0_STAGE0_B2_ADDR, B2_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_8_ALG0_STAGE0_A0_ADDR, A0_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_8_ALG0_STAGE0_A1_ADDR, A1_data, true);
  }
  if ((filtertype > 2 and filtertype < 4) or (filtertype > 2 and filtertype < 4)) {
    dsp.safeload_writeRegister(MOD_GENFILTER1_10_ALG0_STAGE0_B0_ADDR, B0_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_10_ALG0_STAGE0_B1_ADDR, B1_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_10_ALG0_STAGE0_B2_ADDR, B2_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_10_ALG0_STAGE0_A0_ADDR, A0_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_10_ALG0_STAGE0_A1_ADDR, A1_data, true);
  }
  xSemaphoreGive(i2cMutex);
  mute_filters(false);
}

// calculate and set lowpass filter
void set_lowpass_filter(uint8_t filtertype) {
  // type 0-3 butterworth
  // type 4-7 chebyshev
  mute_filters(true);

  float calc_A0 = 0;
  float calc_A1 = 0;
  float calc_B0 = 0;
  float calc_B1 = 0;
  float calc_B2 = 0;

  if (filtertype < 4) {

    uint16_t filter_frequency = lv_spinbox_get_value(spinbox_filter_lowpass);
    // FILTER_2ND_BUTTERWORTH_LOW_PASS:
    // parameters
    float p_boost = 1.0f;
    float p_fs = 48000.0f;
    float p_gain = 0.01f;

    // intermediate values
    float i_Ax = powf(10.0f, p_boost / 40.0f);
    float i_omega = (PI * 2.0f * (float) filter_frequency) / p_fs;
    float i_sn = sinf(i_omega);
    float i_cs = cosf(i_omega);
    float i_alpha = i_sn / (2.0 * (1.0 / sqrt(2.0)));
    float i_a0 = 1.0f + i_alpha;

    // final values
    calc_A0 = -((0.0 - 2.0 * i_cs) / i_a0);
    calc_A1 = -((1.0 - i_alpha) / i_a0);
    calc_B1 = (1.0 - i_cs) / i_a0 * pow(10.0, p_gain / 20.0);
    calc_B0 = calc_B1 / 2.0;
    calc_B2 = calc_B0;
  }
  else {
    // FILTER_2ND_CHEBYSHEV_LOW_PASS
    uint16_t filter_frequency = lv_spinbox_get_value(spinbox_filter_lowpass);

    // parameters
    float p_ripple = 0.1f;
    float p_fs = 48000.0f;
    float p_gain = 0.01f;

    // intermediate values
    float i_omega = tan(PI * 2.0f * (float) filter_frequency / p_fs / 2.0);
    float i_num = sqrt(pow(10.0, 0.1 * p_ripple) - 1.0);
    float i_num1 = pow(i_num, 2.0);
    float i_num3 = i_omega * sinh(0.5 * log(1.0 / i_num + sqrt(1.0 / i_num1 + 1.0)));
    float i_num4 = PI * 3.0 / 4.0;
    float i_Ax = i_omega * sin(i_num4);
    float i_num8 = pow(i_Ax, 2.0);
    float i_num9 = pow(i_num3, 2.0);
    float i_num5 = sqrt(1.0 / (1.0 + i_num1));
    float i_num6 = 1.0 - 2.0 * i_num3 * cos(i_num4) + i_num9 + i_num8;
    float i_num7 = (i_num9 + i_num8) / i_num6;

    // final values
    calc_A0 = -(2.0 * (i_num9 + i_num8 - 1.0) / i_num6);
    calc_A1 = -((1.0 + 2.0 * i_num3 * cos(i_num4) + i_num9 + i_num8) / i_num6);
    calc_B0 = i_num5 * i_num7 * pow(10.0, p_gain / 20.0);
    calc_B1 = calc_B0 * 2.0;
    calc_B2 = calc_B0;
  }

  // same parameters for all filters
  uint8_t B0_data[5], B1_data[5], B2_data[5], A0_data[5], A1_data[5];
  dsp.floatToFixed(calc_B0, B0_data);
  dsp.floatToFixed(calc_B1, B1_data);
  dsp.floatToFixed(calc_B2, B2_data);
  dsp.floatToFixed(calc_A0, A0_data);
  dsp.floatToFixed(calc_A1, A1_data);

  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  // filter 1 allways
  dsp.safeload_writeRegister(MOD_GENFILTER1_5_ALG0_STAGE0_B0_ADDR, B0_data, false);
  dsp.safeload_writeRegister(MOD_GENFILTER1_5_ALG0_STAGE0_B1_ADDR, B1_data, false);
  dsp.safeload_writeRegister(MOD_GENFILTER1_5_ALG0_STAGE0_B2_ADDR, B2_data, false);
  dsp.safeload_writeRegister(MOD_GENFILTER1_5_ALG0_STAGE0_A0_ADDR, A0_data, false);
  dsp.safeload_writeRegister(MOD_GENFILTER1_5_ALG0_STAGE0_A1_ADDR, A1_data, true);
  if ((filtertype > 0 and filtertype < 4) or (filtertype > 0 and filtertype < 4)) {
    dsp.safeload_writeRegister(MOD_GENFILTER1_7_ALG0_STAGE0_B0_ADDR, B0_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_7_ALG0_STAGE0_B1_ADDR, B1_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_7_ALG0_STAGE0_B2_ADDR, B2_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_7_ALG0_STAGE0_A0_ADDR, A0_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_7_ALG0_STAGE0_A1_ADDR, A1_data, true);
  }
  if ((filtertype > 1 and filtertype < 4) or (filtertype > 1 and filtertype < 4)) {
    dsp.safeload_writeRegister(MOD_GENFILTER1_9_ALG0_STAGE0_B0_ADDR, B0_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_9_ALG0_STAGE0_B1_ADDR, B1_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_9_ALG0_STAGE0_B2_ADDR, B2_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_9_ALG0_STAGE0_A0_ADDR, A0_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_9_ALG0_STAGE0_A1_ADDR, A1_data, true);
  }
  if ((filtertype > 2 and filtertype < 4) or (filtertype > 2 and filtertype < 4)) {
    dsp.safeload_writeRegister(MOD_GENFILTER1_11_ALG0_STAGE0_B0_ADDR, B0_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_11_ALG0_STAGE0_B1_ADDR, B1_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_11_ALG0_STAGE0_B2_ADDR, B2_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_11_ALG0_STAGE0_A0_ADDR, A0_data, false);
    dsp.safeload_writeRegister(MOD_GENFILTER1_11_ALG0_STAGE0_A1_ADDR, A1_data, true);
  }
  xSemaphoreGive(i2cMutex);
  mute_filters(false);
}

static void event_filter_mode(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    audio_filter_mode = lv_dropdown_get_selected(obj);
    select_hi_lo_filters(audio_filter_mode);
    set_highpass_filter(audio_filter_mode);
    set_lowpass_filter(audio_filter_mode);
  }
}

static void event_filter_reset(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    audio_filter_lock_updates = true;
    lv_spinbox_set_value(spinbox_filter_highpass, 100);
    filter_highpass_freq = 100;
    lv_spinbox_set_value(spinbox_filter_lowpass, 4000);
    filter_lowpass_freq = 4000;
    lv_spinbox_set_value(spinbox_filter_bandpass, 2050);
    filter_bandpass_freq = 2050;
    lv_spinbox_set_value(spinbox_peak_bandpass, 2050);
    filter_peak_bandpass_freq = 2050;
    lv_spinbox_set_value(spinbox_filter_bandwidth, 3900);
    filter_bandwidth = 3900;

    audio_filter_lock_updates = false;
    set_highpass_filter(audio_filter_mode);
    set_lowpass_filter(audio_filter_mode);
  }
}

// peaking screen

static void event_peak_bandpass_width_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    filter_peak_bandwidth = lv_spinbox_get_value(obj);
    set_peak_filter(filter_peak_on, filter_peak_bandpass_freq, filter_peak_bandwidth, false, 0);
    audio_spectrum_bandpasstrigger = true;
  }
}

void increment_peak_bw_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_peak_bandwidth);
  }
}

static void decrement_peak_bw_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_peak_bandwidth);
  }
}

static void event_peak_switch(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    filter_peak_on = lv_obj_get_state(obj) & LV_STATE_CHECKED;
    set_peak_filter(filter_peak_on, filter_peak_bandpass_freq, filter_peak_bandwidth, true, 0);
    audio_spectrum_bandpasstrigger = true;
  }
}

// calculate and set peak filter
void set_peak_filter(boolean filter_on, uint16_t filter_centerfrequency, uint16_t filter_width, boolean doupdate, uint8_t filter_number ) {
  // calc Q and check if Q is within range

  boolean adjust_bw = false;

  float calc_q = filter_centerfrequency / filter_width;

  // check limits
  if (calc_q < 0.5f) {
    float new_bw = filter_centerfrequency / 0.5;
    filter_width = (uint16_t) new_bw;
    adjust_bw = true;
  }
  if (calc_q > 50.0f) {
    float new_bw = filter_centerfrequency / 50;
    filter_width = (uint16_t) new_bw;
    adjust_bw = true;
  }
  // no update if called from width event because it will retrigger and crash with canary stack error, infinite loop
  if (adjust_bw and doupdate and filter_number == 0) {
    lv_spinbox_set_value(spinbox_peak_bandwidth, filter_width);
    filter_peak_bandwidth = filter_width;
  }
  if (filter_number == 0) {
    mute_filters(true);
  }
  if (!filter_on) {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    // write deactivate parameters
    if (filter_number == 0) {
      dsp.safeload_writeRegister(MOD_GENFILTER1_12_ALG0_STAGE0_B0_ADDR, 0x00800000, false);
      dsp.safeload_writeRegister(MOD_GENFILTER1_12_ALG0_STAGE0_B1_ADDR, 0x00000000, false);
      dsp.safeload_writeRegister(MOD_GENFILTER1_12_ALG0_STAGE0_B2_ADDR, 0x00000000, false);
      dsp.safeload_writeRegister(MOD_GENFILTER1_12_ALG0_STAGE0_A0_ADDR, 0x00000000, false);
      dsp.safeload_writeRegister(MOD_GENFILTER1_12_ALG0_STAGE0_A1_ADDR, 0x00000000, true);
    }
    xSemaphoreGive(i2cMutex);
  }
  else {
    // calculate Q = Fo/BW
    float calc_q = filter_centerfrequency / filter_width;

    // check limits
    if (calc_q < 0.5f) calc_q = 0.5f;
    if (calc_q > 50.0f) calc_q = 50.0f;

    // parameters
    float p_fs = 48000.0f;
    float p_gain = 0.01f;

    // intermediate values
    float i_omega = (PI * 2.0f * (float) filter_centerfrequency) / p_fs;
    float i_sn = sinf(i_omega);
    float i_cs = cosf(i_omega);
    float i_alpha = i_sn / (2.0f * calc_q);
    float i_a0 = 1.0f + i_alpha;
    float i_gainlinear = powf(10.0f, p_gain / 20.0f);

    // final values
    float calc_B0 = (i_gainlinear * i_alpha) / (1.0 + i_alpha);
    float calc_B1 = 0;
    float calc_B2 = -calc_B0;
    float calc_A0 = (-2 * i_cs) / (1.0 + i_alpha);
    float calc_A1 = (1.0 - i_alpha) / (1.0 + i_alpha);

    // write parameters
    uint8_t storeData[5];

    uint8_t B0_data[5], B1_data[5], B2_data[5], A0_data[5], A1_data[5];
    dsp.floatToFixed(calc_B0, B0_data);
    dsp.floatToFixed(calc_B1, B1_data);
    dsp.floatToFixed(calc_B2, B2_data);
    // negate result, same as sigmadsp does
    dsp.floatToFixed(-calc_A0, A0_data);
    // negate result, same as sigmadsp does
    dsp.floatToFixed(-calc_A1, A1_data);
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    if (filter_number == 0) {
      dsp.safeload_writeRegister(MOD_GENFILTER1_12_ALG0_STAGE0_B0_ADDR, B0_data, false);
      dsp.safeload_writeRegister(MOD_GENFILTER1_12_ALG0_STAGE0_B1_ADDR, B1_data, false);
      dsp.safeload_writeRegister(MOD_GENFILTER1_12_ALG0_STAGE0_B2_ADDR, B2_data, false);
      dsp.safeload_writeRegister(MOD_GENFILTER1_12_ALG0_STAGE0_A0_ADDR, A0_data, false);
      dsp.safeload_writeRegister(MOD_GENFILTER1_12_ALG0_STAGE0_A1_ADDR, A1_data, true);
    }
    xSemaphoreGive(i2cMutex);
  }
  if (filter_number == 0) {
    mute_filters(false);
  }
}


void update_peak_filter(void) {
  if (filter_peak_on) {
    lv_obj_add_state(cb_peak_on, LV_STATE_CHECKED);
  }
  else {
    lv_obj_clear_state(cb_peak_on, LV_STATE_CHECKED);
  }
  lv_spinbox_set_value(spinbox_peak_bandpass, filter_peak_bandpass_freq);
  lv_spinbox_set_value(spinbox_peak_bandwidth, filter_peak_bandwidth);
}


// dynamic screen

static void event_dynamic_denoiser_switch(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_dynamic_denoiser = true;
      if (lv_spinbox_get_value(spinbox_filter_highpass) < 300) {
        lv_spinbox_set_value(spinbox_filter_highpass, 300);
      }
      if (!audio_dynamic_notch and lmsTaskHandle) {
        lms_task_set_enabled(lmsTaskHandle, true);
        set_esp32_mode(1);
      }
      if (lmsTaskHandle) {
        lms_task_enable_denoiser(lmsTaskHandle, audio_dynamic_denoiser);
      }
    }
    else {
      audio_dynamic_denoiser = false;
      if (lmsTaskHandle) {
        lms_task_enable_denoiser(lmsTaskHandle, audio_dynamic_denoiser);
      }
      if (!audio_dynamic_notch and lmsTaskHandle) {
        set_esp32_mode(0);
        lms_task_set_enabled(lmsTaskHandle, false);
      }
    }
  }
}

static void event_dynamic_notch_switch(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_dynamic_notch = true;
      if (lv_spinbox_get_value(spinbox_filter_highpass) < 300) {
        lv_spinbox_set_value(spinbox_filter_highpass, 300);
      }
      if (!audio_dynamic_denoiser and lmsTaskHandle) {
        lms_task_set_enabled(lmsTaskHandle, true);
        set_esp32_mode(1);
      }
      if (lmsTaskHandle) {
        lms_task_enable_notch(lmsTaskHandle, audio_dynamic_notch);
      }
    }
    else {
      audio_dynamic_notch = false;
      if (lmsTaskHandle) {
        lms_task_enable_notch(lmsTaskHandle, audio_dynamic_notch);
      }
      if (!audio_dynamic_denoiser and lmsTaskHandle) {
        set_esp32_mode(0);
        lms_task_set_enabled(lmsTaskHandle, false);
      }
    }
  }
}

static void event_dynamic_agc_switch(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_dynamic_agc = true;
    }
    else {
      audio_dynamic_agc = false;
    }
    if (lmsTaskHandle) {
      lms_task_set_agc_enabled(lmsTaskHandle, audio_dynamic_agc);
    }
  }
}

void update_dynamic(void) {
  if (audio_dynamic_denoiser) {
    lv_obj_add_state(cb_dynamic_on, LV_STATE_CHECKED);
  }
  else {
    lv_obj_clear_state(cb_dynamic_on, LV_STATE_CHECKED);
  }
  if (audio_dynamic_notch) {
    lv_obj_add_state(cb_dynamic_autonotch, LV_STATE_CHECKED);
  }
  else {
    lv_obj_clear_state(cb_dynamic_autonotch, LV_STATE_CHECKED);
  }
  if (audio_dynamic_agc) {
    lv_obj_add_state(cb_dynamic_agc, LV_STATE_CHECKED);
  }
  else {
    lv_obj_clear_state(cb_dynamic_agc, LV_STATE_CHECKED);
  }
}

// Per-sample AGC processing
float apply_agc(LMS_Context_t *lms_c, float sample, bool mode_notch) {
  // Reset AGC state if requested
  if (lms_c->agc_needs_reset) {
    lms_c->agc_first_run = true;
    lms_c->agc_needs_reset = false;
  }
  // Initialize on first run or after reset
  if (lms_c->agc_first_run) {
    lms_c->agc_env = fabsf(sample);
    lms_c->agc_rms = lms_c->agc_env * lms_c->agc_env;
    lms_c->agc_gain = 0.1f; // Start from low gain
    lms_c->agc_first_run = false;
    lms_c->agc_soft_start_counter = (int)(0.02f * 48000.0f); // 20ms
    // Set mode-specific parameters
    if (mode_notch) {
      lms_c->agc_current_target = lms_c->agc_target_notch;
    } else {
      lms_c->agc_current_target = lms_c->agc_target_denoise;
    }
  }
  // 1. Envelope detection
  float abs_sample = fabsf(sample);
  float alpha_attack = expf(-1.0f / (lms_c->agc_attack_ms * AGC_SAMPLE_RATE * 0.001f));
  float alpha_release = expf(-1.0f / (lms_c->agc_release_ms * AGC_SAMPLE_RATE * 0.001f));
  if (abs_sample > lms_c->agc_env) {
    lms_c->agc_env = alpha_attack * lms_c->agc_env + (1 - alpha_attack) * abs_sample;
  } else {
    lms_c->agc_env = alpha_release * lms_c->agc_env + (1 - alpha_release) * abs_sample;
  }
  // 2. RMS estimation
  lms_c->agc_rms = 0.999f * lms_c->agc_rms + 0.001f * (lms_c->agc_env * lms_c->agc_env);
  // 3. Soft-start gain ramp
  if (lms_c->agc_soft_start_counter > 0) {
    // Calculate desired gain
    float desired_gain = lms_c->agc_current_target / (sqrtf(lms_c->agc_rms) + EPSILON);
    desired_gain = fmaxf(fminf(desired_gain, lms_c->agc_max_gain), lms_c->agc_min_gain);
    // Linear ramp from initial gain to desired gain
    float progress = 1.0f - (lms_c->agc_soft_start_counter / (float)(0.01f * AGC_SAMPLE_RATE));
    lms_c->agc_gain = lms_c->agc_min_gain + progress * (desired_gain - lms_c->agc_min_gain);
    lms_c->agc_soft_start_counter--;
  }
  // 4. Normal AGC operation
  else {
    // Calculate desired gain
    float desired_gain = lms_c->agc_current_target / (sqrtf(lms_c->agc_rms) + EPSILON);
    desired_gain = fmaxf(fminf(desired_gain, lms_c->agc_max_gain), lms_c->agc_min_gain);
    // Smooth gain changes
    float alpha_smooth = (desired_gain > lms_c->agc_gain) ? alpha_attack : alpha_release;
    lms_c->agc_gain = alpha_smooth * lms_c->agc_gain + (1 - alpha_smooth) * desired_gain;
  }
  // Apply mode-specific final gain
  float output_gain = mode_notch ? lms_c->agc_output_gain_notch : lms_c->agc_output_gain_denoise;
  return sample * lms_c->agc_gain * lms_c->agc_output_gain;
}

// Reset filter state when changing modes
void reset_filter_state(LMS_Context_t *lms_c) {
  int d_max = (lms_c->NDEL_NOTCH > lms_c->NDEL_DENOISE) ? lms_c->NDEL_NOTCH : lms_c->NDEL_DENOISE;
  for (int i = 0; i < d_max; i++)
    lms_c->dline[i] = 0.0f;
  for (int i = 0; i < lms_c->NFIR; i++)
    lms_c->x[i] = 0.0f;
  lms_c->d_head = 0;
  lms_c->x_head = 0;
  lms_c->decay_counter = 0;
  lms_c->agc_needs_reset = true;
}

// LMS task function with cascading filters
static void lms_task_func(void *pv) {
  LMS_Context_t *lms_c = (LMS_Context_t*)pv;
  if (!lms_c) vTaskDelete(NULL);
  while (1) {
    uint32_t new_commands;
    if (xTaskNotifyWait(ULONG_MAX, ULONG_MAX, &new_commands, 0) == pdTRUE) {
      // Process enable commands
      if (new_commands & CMD_ENABLE_SET) {
        if (!lms_c->enabled) {
          lms_c->enabled = true;
          lms_c->agc_needs_reset = true;
        }
      }
      if (new_commands & CMD_ENABLE_CLEAR) {
        if (lms_c->enabled) {
          lms_c->enabled = false;
        }
      }
      // process denoiser enable/disable
      if (new_commands & CMD_DENOISER_ENABLE) {
        if (!lms_c->denoiser_enabled) {
          lms_c->denoiser_enabled = true;
          reset_filter_state(lms_c);
        }
      }
      if (new_commands & CMD_DENOISER_DISABLE) {
        if (lms_c->denoiser_enabled) {
          lms_c->denoiser_enabled = false;
          reset_filter_state(lms_c);
        }
      }
      // process notch enable/disable
      if (new_commands & CMD_NOTCH_ENABLE) {
        if (!lms_c->notch_enabled) {
          lms_c->notch_enabled = true;
          reset_filter_state(lms_c);
        }
      }
      if (new_commands & CMD_NOTCH_DISABLE) {
        if (lms_c->notch_enabled) {
          lms_c->notch_enabled = false;
          reset_filter_state(lms_c);
        }
      }
      // process mode commands (for compatibility)
      if (new_commands & CMD_MODE_NOTCH) {
        if (!lms_c->mode_notch) {
          lms_c->mode_notch = true;
          lms_c->d_len = lms_c->NDEL_NOTCH;
          reset_filter_state(lms_c);
        }
      }
      if (new_commands & CMD_MODE_DENOISE) {
        if (lms_c->mode_notch) {
          lms_c->mode_notch = false;
          lms_c->d_len = lms_c->NDEL_DENOISE;
          reset_filter_state(lms_c);
        }
      }
      // process AGC commands
      if (new_commands & CMD_AGC_ENABLE) {
        if (!lms_c->agc_enabled) {
          lms_c->agc_enabled = true;
          lms_c->agc_needs_reset = true;
        }
      }
      if (new_commands & CMD_AGC_DISABLE) {
        if (lms_c->agc_enabled) {
          lms_c->agc_enabled = false;
        }
      }
      if (new_commands & CMD_AGC_RESET) {
        lms_c->agc_needs_reset = true;
      }
      if (new_commands & CMD_STOP) {
        lms_c->stop_requested = true;
      }
    }
    ringbuffer_read(lms_c->inBuf, BLOCK_SIZE);
    // we need 1024 or 2048 samples
    uint8_t buffercount = 0;
    switch (lv_tabview_get_tab_act(tabview)) {
      case TAB_SCOPE_REF:
        // scope screen
        buffercount = 4;
        break;

      case TAB_SPECTRUM_REF:
        // spectrum screen
        buffercount = 8;
        break;
    }
    // tap for scope and spectrum
    if (audio_scope_trigger && audio_scope_fill_block_count < buffercount) {
      memcpy(&audio_scope_buffer[audio_scope_fill_block_count * BLOCK_SIZE], lms_c->inBuf, BLOCK_SIZE * sizeof(int32_t));
      audio_scope_fill_block_count++;
      if (audio_scope_fill_block_count == buffercount) {
        audio_scope_ready = true;
        audio_scope_trigger = false;
      }
    }
    if (lv_tabview_get_tab_act(tabview) == TAB_MORSE_REF and morse_on and !morse_buffer_ready) {
      memcpy(morse_inputbuffer, lms_c->inBuf, BLOCK_SIZE * sizeof(int32_t));
      morse_buffer_ready = true;
    }
    for (int n = 0; n < BLOCK_SIZE; ++n) {
      float in_f = (float)(lms_c->inBuf[n] >> 8) * lms_c->PCM24_SCALE;
      float processed = in_f;
      // apply Denoiser if enabled
      if (lms_c->denoiser_enabled) {
        // set denoiser mode
        lms_c->mode_notch = false;
        lms_c->d_len = lms_c->NDEL_DENOISE;
        // 1. Get delayed reference
        float delayed = lms_c->dline[lms_c->d_head];
        // 2. Update delay line
        lms_c->dline[lms_c->d_head] = processed;
        lms_c->d_head = (lms_c->d_head + 1) % lms_c->d_len;
        // 3. Update FIR state
        lms_c->x[lms_c->x_head] = delayed;
        lms_c->x_head = (lms_c->x_head + 1) % lms_c->NFIR;
        // 4. Coefficient decay
        lms_c->w[lms_c->decay_counter] *= lms_c->DECAY_DENOISE;
        lms_c->decay_counter = (lms_c->decay_counter + 1) % lms_c->NFIR;
        // 5. Compute FIR output
        float y = 0.0f;
        int xi = lms_c->x_head;
        for (int k = 0; k < lms_c->NFIR; ++k) {
          y += lms_c->w[k] * lms_c->x[xi];
          xi = (xi + 1) % lms_c->NFIR;
        }
        // 6. Error calculation
        float e = delayed - y;
        // 7. LMS update if enabled
        if (lms_c->enabled) {
          xi = lms_c->x_head;
          for (int k = 0; k < lms_c->NFIR; ++k) {
            lms_c->w[k] += lms_c->BETA_DENOISE * e * lms_c->x[xi];
            xi = (xi + 1) % lms_c->NFIR;
          }
        }
        processed = y * lms_c->gain_denoise;
      }

      // apply Notch if enabled
      if (lms_c->notch_enabled) {
        // Set notch mode
        lms_c->mode_notch = true;
        lms_c->d_len = lms_c->NDEL_NOTCH;
        // 1. get delayed reference
        float delayed = lms_c->dline[lms_c->d_head];
        // 2. update delay line
        lms_c->dline[lms_c->d_head] = processed;
        lms_c->d_head = (lms_c->d_head + 1) % lms_c->d_len;
        // 3. update FIR state
        lms_c->x[lms_c->x_head] = delayed;
        lms_c->x_head = (lms_c->x_head + 1) % lms_c->NFIR;
        // 4. coefficient decay
        lms_c->w[lms_c->decay_counter] *= lms_c->DECAY_NOTCH;
        lms_c->decay_counter = (lms_c->decay_counter + 1) % lms_c->NFIR;
        // 5. Compute FIR output
        float y = 0.0f;
        int xi = lms_c->x_head;
        for (int k = 0; k < lms_c->NFIR; ++k) {
          y += lms_c->w[k] * lms_c->x[xi];
          xi = (xi + 1) % lms_c->NFIR;
        }
        // 6. error calculation
        float e = delayed - y;
        // 7. LMS update if enabled
        if (lms_c->enabled) {
          xi = lms_c->x_head;
          for (int k = 0; k < lms_c->NFIR; ++k) {
            lms_c->w[k] += lms_c->BETA_NOTCH * e * lms_c->x[xi];
            xi = (xi + 1) % lms_c->NFIR;
          }
        }
        processed = e * lms_c->gain_notch;
      }
      float out_f;
      if (lms_c->enabled) {
        out_f = processed;
        // apply AGC if enabled
        if (lms_c->agc_enabled) {
          // determine active filter for AGC parameters
          bool use_notch_params = lms_c->notch_enabled;
          out_f = apply_agc(lms_c, out_f, use_notch_params);
        }
      } else {
        out_f = in_f;  // bypass mode
      }
      // convert to int32 and clip
      out_f = fmaxf(fminf(out_f, 0.999999f), -0.999999f);
      lms_c->outBuf[n] = (int32_t)lrintf(out_f * lms_c->PCM24_INV) << 8;
    }
    // I2S Write
    size_t bytesWritten;
    i2s_write(I2S_NUM_0, lms_c->outBuf, BLOCK_SIZE * sizeof(int32_t), &bytesWritten, portMAX_DELAY);
  }
}

// start LMS task with cascading filters
TaskHandle_t start_lms_task(bool startEnabled, bool startDenoiser, bool startNotch) {
  if (lmsTaskHandle) {
    // update existing task
    lms_task_set_enabled(lmsTaskHandle, startEnabled);
    uint32_t denoiser_cmd = startDenoiser ? CMD_DENOISER_ENABLE : CMD_DENOISER_DISABLE;
    uint32_t notch_cmd = startNotch ? CMD_NOTCH_ENABLE : CMD_NOTCH_DISABLE;
    xTaskNotify(lmsTaskHandle, denoiser_cmd, eSetBits);
    xTaskNotify(lmsTaskHandle, notch_cmd, eSetBits);
    return lmsTaskHandle;
  }
  // allocate and initialize context
  LMS_Context_t *lms_c = (LMS_Context_t*)malloc(sizeof(LMS_Context_t));
  if (!lms_c) return NULL;
  memset(lms_c, 0, sizeof(LMS_Context_t));
  // sampling rate conversion
  const float FS_REF = 16611.0f;
  const float FS_TARGET = 48000.0f;
  const float FS_RATIO = FS_REF / FS_TARGET;
  // fixed configuration
  lms_c->NFIR = 24;
  const int NDEL_NOTCH_16K = 65;
  lms_c->NDEL_NOTCH = (int)lroundf(NDEL_NOTCH_16K * (FS_TARGET / FS_REF)); // 188 samples
  lms_c->NDEL_DENOISE = 1;
  lms_c->d_len = startNotch ? lms_c->NDEL_NOTCH : (startDenoiser ? lms_c->NDEL_DENOISE : 1);
  // PCM scaling
  lms_c->PCM24_SCALE = 1.0f / 8388608.0f;  // 2^23
  lms_c->PCM24_INV = 8388608.0f;
  // scaled parameters
  lms_c->BETA_NOTCH = 0.125f * sqrtf(16611.0f / 48000.0f); // 0.073
  lms_c->BETA_DENOISE = 0.1875f * sqrtf(16611.0f / 48000.0f); // 0.110
  lms_c->DECAY_NOTCH = powf(0.9991455078125f, 16611.0f / 48000.0f);
  lms_c->DECAY_DENOISE = powf(0.98046875f, 16611.0f / 48000.0f);
  // allocate buffers
  const int BLOCK_SZ = 256;
  lms_c->inBuf = (int32_t*)malloc(sizeof(int32_t) * BLOCK_SZ);
  lms_c->outBuf = (int32_t*)malloc(sizeof(int32_t) * BLOCK_SZ);
  lms_c->x = (float*)malloc(sizeof(float) * lms_c->NFIR);
  lms_c->w = (float*)malloc(sizeof(float) * lms_c->NFIR);
  int d_max = (lms_c->NDEL_NOTCH > lms_c->NDEL_DENOISE) ? lms_c->NDEL_NOTCH : lms_c->NDEL_DENOISE;
  lms_c->dline = (float*)malloc(sizeof(float) * d_max);
  // check allocations
  if (!lms_c->inBuf || !lms_c->outBuf || !lms_c->x || !lms_c->w || !lms_c->dline) {
    if (lms_c->inBuf) free(lms_c->inBuf);
    if (lms_c->outBuf) free(lms_c->outBuf);
    if (lms_c->x) free(lms_c->x);
    if (lms_c->w) free(lms_c->w);
    if (lms_c->dline) free(lms_c->dline);
    free(lms_c);
    return NULL;
  }
  // initialize state
  for (int i = 0; i < BLOCK_SZ; i++) {
    lms_c->inBuf[i] = 0;
    lms_c->outBuf[i] = 0;
  }
  for (int i = 0; i < lms_c->NFIR; i++) {
    lms_c->x[i] = 0.0f;
    lms_c->w[i] = (esp_random() / (float)UINT32_MAX) * 0.01f;
  }
  for (int i = 0; i < d_max; i++)
    lms_c->dline[i] = 0.0f;
  lms_c->x_head = 0;
  lms_c->d_head = lms_c->d_len - 1;
  lms_c->decay_counter = 0;
  lms_c->enabled = startEnabled;
  lms_c->mode_notch = startNotch;
  lms_c->denoiser_enabled = startDenoiser;
  lms_c->notch_enabled = startNotch;
  lms_c->stop_requested = false;
  // AGC parameters
  lms_c->agc_enabled = false;
  lms_c->agc_needs_reset = true;
  lms_c->agc_output_gain_notch = 0.9f;
  lms_c->agc_output_gain_denoise = 1.0f;
  lms_c->gain_notch = 1.0f;
  lms_c->gain_denoise = 2.8f;
  lms_c->agc_target_notch = 0.04f;
  lms_c->agc_target_denoise = 0.09f;
  lms_c->agc_output_gain = 0.8f;
  lms_c->agc_max_gain = 5.0f;
  lms_c->agc_min_gain = 0.1f;
  lms_c->agc_attack_ms = 20.0f;
  lms_c->agc_release_ms = 5000.0f;
  lms_c->agc_first_run = true;
  // initialize current target based on starting mode
  lms_c->agc_current_target = startNotch ? lms_c->agc_target_notch : lms_c->agc_target_denoise;
  // create task
  BaseType_t ok = xTaskCreatePinnedToCore(
                    lms_task_func,
                    "LMS_Task",
                    8192,
                    (void*)lms_c,
                    configMAX_PRIORITIES - 1,
                    &lmsTaskHandle,
                    0
                  );
  if (ok != pdPASS) {
    free(lms_c->inBuf); free(lms_c->outBuf); free(lms_c->x);
    free(lms_c->w); free(lms_c->dline); free(lms_c);
    return NULL;
  }
  return lmsTaskHandle;
}

// control functions
void lms_task_set_enabled(TaskHandle_t handle, bool enable) {
  uint32_t cmd = enable ? CMD_ENABLE_SET : CMD_ENABLE_CLEAR;
  xTaskNotify(handle, cmd, eSetBits);
}

void lms_task_enable_denoiser(TaskHandle_t handle, bool enable) {
  uint32_t cmd = enable ? CMD_DENOISER_ENABLE : CMD_DENOISER_DISABLE;
  xTaskNotify(handle, cmd, eSetBits);
}

void lms_task_enable_notch(TaskHandle_t handle, bool enable) {
  uint32_t cmd = enable ? CMD_NOTCH_ENABLE : CMD_NOTCH_DISABLE;
  xTaskNotify(handle, cmd, eSetBits);
}

void lms_task_set_agc_enabled(TaskHandle_t handle, bool enable) {
  uint32_t cmd = enable ? CMD_AGC_ENABLE : CMD_AGC_DISABLE;
  xTaskNotify(handle, cmd, eSetBits);
}

void lms_task_reset_agc(TaskHandle_t handle) {
  xTaskNotify(handle, CMD_AGC_RESET, eSetBits);
}

void stop_lms_task(TaskHandle_t handle) {
  xTaskNotify(handle, CMD_STOP, eSetBits);
}

// compressor screen

static void event_compressor_threshold_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_compressor_threshold_previous = audio_compressor_threshold;
    audio_compressor_threshold = lv_slider_get_value(obj);
    if (audio_compressor_threshold != audio_compressor_threshold_previous) {
      audio_compressor_update();
    }
    if (!audio_rotary_selected_threshold) {
      audio_rotary_selected_threshold = true;
      set_compressor_labels(0);
    }
  }
}

// update compressor threshold
void update_compressor_threshold(void) {
  lv_slider_set_value(slider_compressor_threshold, audio_compressor_threshold , LV_ANIM_OFF);
}

static void event_compressor_ratio_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_compressor_ratio_previous = audio_compressor_ratio;
    audio_compressor_ratio = lv_slider_get_value(obj);
    if (audio_compressor_ratio != audio_compressor_ratio_previous) {
      audio_compressor_update();
    }
    if (audio_rotary_selected_threshold) {
      audio_rotary_selected_threshold = false;
      set_compressor_labels(0);
    }
  }
}

// update compressor ratio
void update_compressor_ratio(void) {
  lv_slider_set_value(slider_compressor_ratio, audio_compressor_ratio, LV_ANIM_OFF);
}

static void event_compressor_postgain_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_compressor_postgain_previous = audio_compressor_postgain;
    audio_compressor_postgain = lv_slider_get_value(obj);
    if (audio_compressor_postgain != audio_compressor_postgain_previous) {
      audio_compressor_update();
    }
    if (!audio_rotary_selected_postgain) {
      audio_rotary_selected_postgain = true;
      set_compressor_labels(1);
    }
  }
}

// update compressor post gain
void update_compressor_postgain(void) {
  lv_slider_set_value(slider_compressor_postgain, audio_compressor_postgain, LV_ANIM_OFF);
}

static void event_compressor_noisegate_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_compressor_noisegate_previous = audio_compressor_noisegate;
    audio_compressor_noisegate = lv_slider_get_value(obj);
    if (audio_compressor_noisegate != audio_compressor_noisegate_previous) {
      audio_noisegate_update();
    }
    if (audio_rotary_selected_postgain) {
      audio_rotary_selected_postgain = false;
      set_compressor_labels(1);
    }
  }
}

void set_compressor_labels(uint8_t labelset) {
  lv_color_t selected_color = lv_palette_main(LV_PALETTE_GREEN);
  lv_color_t nonselected_color = lv_color_white();
  if (labelset == 0) {
    lv_obj_set_style_text_color(lbl_compressor_threshold, (audio_rotary_selected_threshold) ? selected_color : nonselected_color, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_compressor_ratio, (!audio_rotary_selected_threshold) ? selected_color : nonselected_color, LV_PART_MAIN);
  }
  else {
    lv_obj_set_style_text_color(lbl_compressor_postgain, (audio_rotary_selected_postgain) ? selected_color : nonselected_color, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_compressor_noisegate, (!audio_rotary_selected_postgain) ? selected_color : nonselected_color, LV_PART_MAIN);
  }
}

// update compressor noisegate
void update_compressor_noisegate(void) {
  lv_slider_set_value(slider_compressor_noisegate, audio_compressor_noisegate , LV_ANIM_OFF);
}

// linspace for dB range: -90 to +6
void linspace(float x1, float x2, float n, float *vect) {
  float v = (abs(x1) + abs(x2)) / (n - 1);
  for (int ctr = 0; ctr < n; ctr++) {
    vect[ctr] = x1 + (v * ctr);
  }
}

// 5-point smoothing filter
void curveSmooth5(float in[], float out[], int curveLength) {
  out[0] = in[0];
  out[1] = in[1];
  for (int j = 2; j < curveLength - 2; j++) {
    float sum = 0;
    for (int k = -2; k <= 2; k++) {
      sum += in[j + k];
    }
    out[j] = sum / 5.0f;
  }
  out[curveLength - 2] = in[curveLength - 2];
  out[curveLength - 1] = in[curveLength - 1];
}

// generate compression curve
void MakeCompressCurve(audio_compressor_t &comp) {
  const int len = 34;
  float x[len];
  float y[len];
  float curve[len];

  linspace(-90.0f, 6.0f, len, x);

  float t = comp.threshold;
  float r = comp.ratio;
  float k = comp.knee;
  float halfK = k * 0.5f;
  for (int i = 0; i < len; i++) {
    float in = x[i];
    if (k > 0 && in > t - halfK && in < t + halfK) {
      // soft knee (quadratic interpolation)
      float xk = in - (t - halfK);
      float yk = xk * xk / (2.0f * k);
      y[i] = in + (1.0f / r - 1.0f) * yk;
    } else if (in >= t + halfK) {
      // above knee: compressed
      y[i] = t + (in - t) / r;
    } else {
      // below knee: passthrough
      y[i] = in;
    }
  }

  // apply optional smoothing
  curveSmooth5(y, curve, len);
  for (int i = 0; i < len; i++) { // Coefficients of the curve calculation
    curve[i] = powf(10, (y[i] - x[i]) / 20); // Ratios of the linearized values of vect. y and x
  }
  uint16_t startMemoryAddress = MOD_COMPRESSOR1_ALG0_MONOALG10_ADDR;
  // create buffer to store converted data
  uint8_t storeData[5];
  uint8_t storeData1[5];
  uint8_t storeData2[5];
  uint8_t storeData3[5];
  uint8_t storeData4[5];

  float postgain_par = pow(10, comp.postgain / 20);
  dsp.floatToFixed(postgain_par, storeData1);
  const float FS = dsp.FS;
  // code is changed to reflect the actual values in sigmastudio, code in lib is incorrect
  float rms_tc = comp.rms_tc;  // in dB/second, e.g., 120
  float attack_par = fabsf(1.0f - powf(10.0f, -rms_tc / (10.0f * FS)));

  dsp.floatToFixed(attack_par, storeData2);

  // hold
  float hold_par = comp.hold * FS / 1000;
  dsp.floatToFixed(hold_par, storeData3);

  // decay (dB/s)
  float DecayUI = comp.decay;  // e.g. 10
  float raw = FS / 26373.0f * DecayUI;
  float decay_par = raw * 1.19209289550781e-7f;

  dsp.floatToFixed(decay_par, storeData4);

  // parameter load into Sigma DSP
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  for (int i = 0; i < 34; i++) {
    dsp.floatToFixed(curve[i], storeData);
    dsp.safeload_writeRegister(startMemoryAddress++, storeData, false);
  }
  dsp.safeload_writeRegister(MOD_COMPRESSOR1_ALG0_MONOALG1POSTGAIN_ADDR, storeData1, true);
  dsp.safeload_writeRegister(MOD_COMPRESSOR1_ALG0_MONOALG1RMS_ADDR, storeData2, true);
  dsp.safeload_writeRegister(MOD_COMPRESSOR1_ALG0_MONOALG1HOLD_ADDR, storeData3, true);
  dsp.safeload_writeRegister(MOD_COMPRESSOR1_ALG0_MONOALG1DECAY_ADDR, storeData4, true);
  xSemaphoreGive(i2cMutex);
}

// update compressor parameters
void audio_compressor_update(void) {
  audio_compressor_parameters.threshold = -60.0f + (float) audio_compressor_threshold;
  audio_compressor_parameters.ratio = (float) audio_compressor_ratio;
  audio_compressor_parameters.postgain = (float) audio_compressor_postgain;
  MakeCompressCurve(audio_compressor_parameters);
}

// generate noise gate curve
void MakeNoiseGateCurve(audio_noisegate_t &comp) {
  const int len = 34;
  float x[len];
  float y[len];
  float curve[len];

  linspace(-90.0f, 6.0f, len, x);

  // noise gate parameters
  float t_gate = comp.gateThreshold;
  float k_gate = comp.gateKnee;
  float g_floor = comp.gateFloor;
  float halfK = k_gate * 0.5f;

  for (int i = 0; i < len; i++) {
    float in = x[i];
    float lowerBound = t_gate - halfK;
    float upperBound = t_gate + halfK;

    if (k_gate > 0 && in > lowerBound && in < upperBound) {
      float xk = in - lowerBound;
      float g_scale = (in - g_floor) / (2.0f * halfK * halfK);
      y[i] = g_floor + xk * xk * g_scale;
    }
    else if (in <= lowerBound) {
      y[i] = g_floor;
    }
    else {
      y[i] = in;
    }
  }
  // apply smoothing
  curveSmooth5(y, curve, len);
  // convert to linear gain factors
  for (int i = 0; i < len; i++) {
    curve[i] = powf(10, (y[i] - x[i]) / 20.0f);
  }

  // DSP upload
  uint16_t startMemoryAddress = MOD_COMPRESSOR2_ALG0_MONONOPOSTGAIN3DBFIX10_ADDR;
  uint8_t storeData[5];
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  for (int i = 0; i < 34; i++) {
    dsp.floatToFixed(curve[i], storeData);
    dsp.safeload_writeRegister(startMemoryAddress++, storeData, false);
  }
  xSemaphoreGive(i2cMutex);
}

// update compressor parameters
void audio_noisegate_update(void) {
  audio_noisegate_parameters.gateThreshold =  -90 + audio_compressor_noisegate;
  audio_noisegate_parameters.gateKnee = -10;
  audio_noisegate_parameters.gateFloor = -110;
  MakeNoiseGateCurve(audio_noisegate_parameters);
}

static void event_compressor_reset(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mute_output(1);
    audio_compressor_threshold = 0;
    update_compressor_threshold();
    audio_compressor_ratio = 1;
    update_compressor_ratio();
    audio_compressor_postgain = 0;
    update_compressor_postgain();
    audio_compressor_update();
    audio_compressor_noisegate = 0;
    update_compressor_noisegate();
    audio_noisegate_update();
    mute_output(0);
  }
}

// equalizer screen

static void event_eq0_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_eq0_previous = audio_eq0;
    audio_eq0 = lv_slider_get_value(obj);
    if (audio_eq0 != audio_eq0_previous) {
      eqBand0.boost = audio_eq0;
      set_eq(0);
    }
    if (audio_selected_eq != 0) {
      audio_selected_eq = 0;
      set_eq_labels(audio_selected_eq);
    }
  }
}

static void event_eq1_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_eq1_previous = audio_eq1;
    audio_eq1 = lv_slider_get_value(obj);
    if (audio_eq1 != audio_eq1_previous) {
      eqBand1.boost = audio_eq1;
      set_eq(1);
    }
    if (audio_selected_eq != 1) {
      audio_selected_eq = 1;
      set_eq_labels(audio_selected_eq);
    }
  }
}

static void event_eq2_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_eq2_previous = audio_eq2;
    audio_eq2 = lv_slider_get_value(obj);
    if (audio_eq2 != audio_eq2_previous) {
      eqBand2.boost = audio_eq2;
      set_eq(2);
    }
    if (audio_selected_eq != 2) {
      audio_selected_eq = 2;
      set_eq_labels(audio_selected_eq);
    }
  }
}

static void event_eq3_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_eq3_previous = audio_eq3;
    audio_eq3 = lv_slider_get_value(obj);
    if (audio_eq3 != audio_eq3_previous) {
      eqBand3.boost = audio_eq3;
      set_eq(3);
    }
    if (audio_selected_eq != 3) {
      audio_selected_eq = 3;
      set_eq_labels(audio_selected_eq);
    }
  }
}

static void event_eq4_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_eq4_previous = audio_eq4;
    audio_eq4 = lv_slider_get_value(obj);
    if (audio_eq4 != audio_eq4_previous) {
      eqBand4.boost = audio_eq4;
      set_eq(4);
    }
    if (audio_selected_eq != 4) {
      audio_selected_eq = 4;
      set_eq_labels(audio_selected_eq);
    }
  }
}

static void event_eq5_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_eq5_previous = audio_eq5;
    audio_eq5 = lv_slider_get_value(obj);
    if (audio_eq5 != audio_eq5_previous) {
      eqBand5.boost = audio_eq5;
      set_eq(5);
    }
    if (audio_selected_eq != 5) {
      audio_selected_eq = 5;
      set_eq_labels(audio_selected_eq);
    }
  }
}

static void event_eq6_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_eq6_previous = audio_eq6;
    audio_eq6 = lv_slider_get_value(obj);
    if (audio_eq6 != audio_eq6_previous) {
      eqBand6.boost = audio_eq6;
      set_eq(6);
    }
    if (audio_selected_eq != 6) {
      audio_selected_eq = 6;
      set_eq_labels(audio_selected_eq);
    }
  }
}

static void event_eq7_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_eq7_previous = audio_eq7;
    audio_eq7 = lv_slider_get_value(obj);
    if (audio_eq7 != audio_eq7_previous) {
      eqBand7.boost = audio_eq7;
      set_eq(7);
    }
    if (audio_selected_eq != 7) {
      audio_selected_eq = 7;
      set_eq_labels(audio_selected_eq);
    }
  }
}

static void event_eq8_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_eq8_previous = audio_eq8;
    audio_eq8 = lv_slider_get_value(obj);
    if (audio_eq8 != audio_eq8_previous) {
      eqBand8.boost = audio_eq8;
      set_eq(8);
    }
    if (audio_selected_eq != 8) {
      audio_selected_eq = 8;
      set_eq_labels(audio_selected_eq);
    }
  }
}

static void event_eq9_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_eq9_previous = audio_eq9;
    audio_eq9 = lv_slider_get_value(obj);
    if (audio_eq9 != audio_eq9_previous) {
      eqBand9.boost = audio_eq9;
      set_eq(9);
    }
    if (audio_selected_eq != 9) {
      audio_selected_eq = 9;
      set_eq_labels(audio_selected_eq);
    }
  }
}

static void event_eq10_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_eq10_previous = audio_eq10;
    audio_eq10 = lv_slider_get_value(obj);
    if (audio_eq10 != audio_eq10_previous) {
      eqBand10.boost = audio_eq10;
      set_eq(10);
    }
    if (audio_selected_eq != 10) {
      audio_selected_eq = 10;
      set_eq_labels(audio_selected_eq);
    }
  }
}

static void event_eq11_change(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_eq11_previous = audio_eq11;
    audio_eq11 = lv_slider_get_value(obj);
    if (audio_eq11 != audio_eq11_previous) {
      eqBand11.boost = audio_eq11;
      set_eq(11);
    }
    if (audio_selected_eq != 11) {
      audio_selected_eq = 11;
      set_eq_labels(audio_selected_eq);
    }
  }
}

static void event_eq_reset(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    audio_eq0 = 0;
    audio_eq1 = 0;
    audio_eq2 = 0;
    audio_eq3 = 0;
    audio_eq4 = 0;
    audio_eq5 = 0;
    audio_eq6 = 0;
    audio_eq7 = 0;
    audio_eq8 = 0;
    audio_eq9 = 0;
    audio_eq10 = 0;
    audio_eq11 = 0;
    update_eq_all_sliders();
  }
}

void set_eq_labels(uint8_t label_no) {
  lv_color_t selected_color = lv_palette_main(LV_PALETTE_GREEN);
  lv_color_t nonselected_color = lv_color_white();

  // array of label objects
  lv_obj_t* labels[] = {lbl_eq0, lbl_eq1, lbl_eq2, lbl_eq3, lbl_eq4, lbl_eq5,
                        lbl_eq6, lbl_eq7, lbl_eq8, lbl_eq9, lbl_eq10, lbl_eq11
                       };

  // ensure label_no is within range
  if (label_no >= sizeof(labels) / sizeof(labels[0])) {
    return; // Invalid index, do nothing
  }

  // set colors for all labels
  for (uint8_t i = 0; i < sizeof(labels) / sizeof(labels[0]); i++) {
    lv_obj_set_style_text_color(labels[i], (i == label_no) ? selected_color : nonselected_color, LV_PART_MAIN);
  }
}

void inc_eq_slider(uint8_t slider_no) {
  // array of slider objects
  lv_obj_t* sliders[] = {
    slider_eq0, slider_eq1, slider_eq2, slider_eq3, slider_eq4, slider_eq5,
    slider_eq6, slider_eq7, slider_eq8, slider_eq9, slider_eq10, slider_eq11
  };
  // array of pointers to audio_eq variables
  int32_t* audio_eq[] = {
    &audio_eq0, &audio_eq1, &audio_eq2, &audio_eq3, &audio_eq4, &audio_eq5,
    &audio_eq6, &audio_eq7, &audio_eq8, &audio_eq9, &audio_eq10, &audio_eq11
  };
  // ensure slider_no is within range
  if (slider_no >= sizeof(sliders) / sizeof(sliders[0])) {
    return; // Invalid index, do nothing
  }
  if (*audio_eq[slider_no] < 15) {
    // updating the slider will trigger the update of the audio_eq value
    lv_slider_set_value(sliders[slider_no], lv_slider_get_value(sliders[slider_no]) + 1, LV_ANIM_OFF);
    // setting the value does not trigger the value change event, do it manually
    lv_obj_send_event(sliders[slider_no], LV_EVENT_VALUE_CHANGED, NULL);
  }
}

void dec_eq_slider(uint8_t slider_no) {
  // array of slider objects
  lv_obj_t* sliders[] = {
    slider_eq0, slider_eq1, slider_eq2, slider_eq3, slider_eq4, slider_eq5,
    slider_eq6, slider_eq7, slider_eq8, slider_eq9, slider_eq10, slider_eq11
  };
  // array of pointers to audio_eq variables
  int32_t* audio_eq[] = {
    &audio_eq0, &audio_eq1, &audio_eq2, &audio_eq3, &audio_eq4, &audio_eq5,
    &audio_eq6, &audio_eq7, &audio_eq8, &audio_eq9, &audio_eq10, &audio_eq11
  };
  // ensure slider_no is within range
  if (slider_no >= sizeof(sliders) / sizeof(sliders[0])) {
    return; // Invalid index, do nothing
  }
  if (*audio_eq[slider_no] > -15) {
    // updating the slider will trigger the update of the audio_eq value
    lv_slider_set_value(sliders[slider_no], lv_slider_get_value(sliders[slider_no]) - 1, LV_ANIM_OFF);
    // setting the value does not trigger the value change event, do it manually
    lv_obj_send_event(sliders[slider_no], LV_EVENT_VALUE_CHANGED, NULL);
  }
}

void update_eq_all_sliders(void) {
  eqBand0.boost = audio_eq0;
  lv_slider_set_value(slider_eq0, audio_eq0, LV_ANIM_OFF);
  set_eq(0);
  eqBand1.boost = audio_eq1;
  lv_slider_set_value(slider_eq1, audio_eq1, LV_ANIM_OFF);
  set_eq(1);
  eqBand2.boost = audio_eq2;
  lv_slider_set_value(slider_eq2, audio_eq2, LV_ANIM_OFF);
  set_eq(2);
  eqBand3.boost = audio_eq3;
  lv_slider_set_value(slider_eq3, audio_eq3, LV_ANIM_OFF);
  set_eq(3);
  eqBand4.boost = audio_eq4;
  lv_slider_set_value(slider_eq4, audio_eq4, LV_ANIM_OFF);
  set_eq(4);
  eqBand5.boost = audio_eq5;
  lv_slider_set_value(slider_eq5, audio_eq5, LV_ANIM_OFF);
  set_eq(5);
  eqBand6.boost = audio_eq6;
  lv_slider_set_value(slider_eq6, audio_eq6, LV_ANIM_OFF);
  set_eq(6);
  eqBand7.boost = audio_eq7;
  lv_slider_set_value(slider_eq7, audio_eq7, LV_ANIM_OFF);
  set_eq(7);
  eqBand8.boost = audio_eq8;
  lv_slider_set_value(slider_eq8, audio_eq8, LV_ANIM_OFF);
  set_eq(8);
  eqBand9.boost = audio_eq9;
  lv_slider_set_value(slider_eq9, audio_eq9, LV_ANIM_OFF);
  set_eq(9);
  eqBand10.boost = audio_eq10;
  lv_slider_set_value(slider_eq10, audio_eq10, LV_ANIM_OFF);
  set_eq(10);
  eqBand11.boost = audio_eq11;
  lv_slider_set_value(slider_eq11, audio_eq11, LV_ANIM_OFF);
  set_eq(11);
}

// update equalizer
void set_eq(uint8_t band) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  switch (band) {
    case 0:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE0_B0_ADDR, eqBand0);
      break;

    case 1:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE1_B0_ADDR, eqBand1);
      break;

    case 2:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE2_B0_ADDR, eqBand2);
      break;

    case 3:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE3_B0_ADDR, eqBand3);
      break;

    case 4:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE4_B0_ADDR, eqBand4);
      break;

    case 5:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE5_B0_ADDR, eqBand5);
      break;

    case 6:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE6_B0_ADDR, eqBand6);
      break;

    case 7:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE7_B0_ADDR, eqBand7);
      break;

    case 8:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE8_B0_ADDR, eqBand8);
      break;

    case 9:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE9_B0_ADDR, eqBand9);
      break;

    case 10:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE10_B0_ADDR, eqBand10);
      break;

    case 11:
      dsp.EQsecondOrder(MOD_MIDEQ2_ALG0_STAGE11_B0_ADDR, eqBand11);
      break;
  }
  xSemaphoreGive(i2cMutex);
}

// morse screen

static void event_morse_clear(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    morse_reset();
  }
}

void morse_reset(void) {
  morse_full_buffer = "";
  lv_label_set_text(lbl_morse_text, morse_full_buffer.c_str());
  morse_ticker_head = 0;
  morse_ticker_tail = 0;
  for (int i = 0; i < MORSE_TICKER_BUF_SIZE; i++) {
    morse_ticker_buf[i] = 0;
  }
  morse_ticker_changed = true;  // Forceer hertekenen
  morse_decoder.unit_samples = 2880;
  morse_decoder.m_state = STATE_SPACE;
  morse_decoder.noise_floor = MORSE_NOISE_FLOOR;
  morse_decoder.signal_peak = 10;
  morse_decoder.peak_enveloppe = 0.0f;
}

static void event_morse_switch(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      morse_reset();
      morse_buffer_ready = false;
      morse_on = true;
    }
    else {
      morse_buffer_ready = false;
      morse_on = false;
    }
  }
}

void morse_setlabel(void) {
  // since the peaking filter updates both we can use just this one
  filter_bandpass_freq = lv_spinbox_get_value(spinbox_filter_bandpass);
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "F: %ldHz", filter_bandpass_freq);
  lv_label_set_text(lbl_morse_freq, buffer);
}

// dynamic minimum duration functions
uint32_t get_min_mark_duration() {
  const uint32_t absolute_min = SAMPLE_RATE * 10 / 1000;  // 10ms
  uint32_t relative_min = morse_decoder.unit_samples * 3 / 10;  // 30%
  return (relative_min > absolute_min) ? relative_min : absolute_min;
}

uint32_t get_min_space_duration() {
  return get_min_mark_duration();  // Same calculation
}

void handle_space_end(uint32_t duration) {
  // calculate dynamic minimum duration
  const uint32_t absolute_min = SAMPLE_RATE * 10 / 1000;
  uint32_t min_duration = morse_decoder.unit_samples * 3 / 10;
  min_duration = (min_duration > absolute_min) ? min_duration : absolute_min;
  // filter short spaces
  if (duration < min_duration) return;
  // calculate durations
  float gap_ms = duration * 1000.0 / SAMPLE_RATE;
  float units = (float)duration / morse_decoder.unit_samples;
  float wpm = 1200.0 / (morse_decoder.unit_samples * 1000.0 / SAMPLE_RATE);
  float expected_word_gap = 7.0 * morse_decoder.unit_samples * 1000.0 / SAMPLE_RATE;
  // character gap detection
  if (units > 1.5f) {
    decode_symbol();
    // word gap detection (absolute time)
    if (gap_ms > expected_word_gap * 0.6) {  // 60% of expected
      morse_buffer_put(' ');
      morse_ticker_put(' ');
    }
    else {
      morse_ticker_put('|');
    }
  }
  else {
  }
}

char morse_to_char(const char* code) {
  for (int i = 0; morse_table[i].code != NULL; ++i) {
    if (strcmp(morse_table[i].code, code) == 0) {
      return morse_table[i].symbol;
    }
  }
  return 0;  // not found
}

void decode_symbol() {
  if (morse_decoder.sym_len == 0) return;
  morse_decoder.symbol[morse_decoder.sym_len] = '\0';  // Null-terminate
  char ch_ = morse_to_char(morse_decoder.symbol);
  if (ch_) morse_buffer_put(ch_);
  else morse_buffer_put('?');
  morse_decoder.sym_len = 0;  // Reset buffer
}

void update_gap_thresholds() {
  // character gap = 3 unit times
  morse_decoder.char_gap_threshold = morse_decoder.unit_samples * 3;
  // word gap = 7 unit times
  morse_decoder.word_gap_threshold = morse_decoder.unit_samples * 7;
  // minimum gap thresholds (for slow speeds)
  const uint32_t min_char_gap = SAMPLE_RATE * 50 / 1000;  // 50ms
  const uint32_t min_word_gap = SAMPLE_RATE * 150 / 1000; // 150ms
  if (morse_decoder.char_gap_threshold < min_char_gap) {
    morse_decoder.char_gap_threshold = min_char_gap;
  }
  if (morse_decoder.word_gap_threshold < min_word_gap) {
    morse_decoder.word_gap_threshold = min_word_gap;
  }
}

// Morse processing task
void morse_task(void* arg) {
  int32_t process_buffer[BLOCK_SIZE];
  float envelope[BLOCK_SIZE];
  // precompute constants outside the loop
  const float dsp_convertion = 1.0f / 8388608.0f;
  const float alpha = (256.0f - 8.0f) / 256.0f;
  const float beta = 8.0f / 256.0f;
  // initialize Morse decoder
  memset(&morse_decoder, 0, sizeof(morse_decoder));
  morse_decoder.m_state = STATE_SPACE;
  morse_decoder.noise_floor = MORSE_NOISE_FLOOR;
  morse_decoder.signal_peak = 10;
  morse_decoder.unit_samples = (SAMPLE_RATE * 60) / 1000; // 20WPM
  update_gap_thresholds();
  // timer for reset of speed and flush the last character
  uint32_t last_calibration_time = millis();
  // variables for timing calibration
  uint32_t dot_durations[10] = {0};
  uint32_t dash_durations[10] = {0};
  uint8_t dot_index = 0;
  uint8_t dash_index = 0;
  uint32_t dot_count = 0;
  uint32_t dash_count = 0;
  uint32_t avg_dot_duration = 0;
  uint32_t avg_dash_duration = 0;

  while (true) {
    if (morse_buffer_ready) {
      memcpy(process_buffer, morse_inputbuffer, BLOCK_SIZE * sizeof(int32_t));
      morse_buffer_ready = false;
      // step 1: Envelope detection
      float block_min = 10000;
      float block_max = 0;
      for (int i = 0; i < BLOCK_SIZE; i++) {
        // Convert DSP data and take absolute value
        int32_t sample_ = process_buffer[i];
        float absolute_sample = fabsf((float)sample_ * dsp_convertion);
        // peak follower with attack/decay for envelope
        if (absolute_sample >= morse_decoder.peak_enveloppe) {
          morse_decoder.peak_enveloppe = absolute_sample;
        } else {
          morse_decoder.peak_enveloppe = alpha * morse_decoder.peak_enveloppe + beta * absolute_sample;
        }
        envelope[i] = morse_decoder.peak_enveloppe;
        // update block min/max
        block_min = min(block_min, absolute_sample);
        block_max = max(block_max, morse_decoder.peak_enveloppe);
      }

      // step 2: update noise floor and signal peak
      if (morse_decoder.m_state == STATE_SPACE) {
        // only update noise floor during silence
        morse_decoder.noise_floor = (morse_decoder.noise_floor * 15 + block_min) / 16;
        if (morse_decoder.noise_floor < MORSE_NOISE_FLOOR) {
          morse_decoder.noise_floor = MORSE_NOISE_FLOOR;
        }
      }

      // update signal peak
      if (block_max > morse_decoder.signal_peak) {
        // fast attack
        morse_decoder.signal_peak = (morse_decoder.signal_peak * 0.1f) + (block_max * 0.9f);
      } else {
        // slow decay
        morse_decoder.signal_peak = (morse_decoder.signal_peak * 0.99f) + (block_max * 0.01f);
      }

      // step 3: calculate dynamic threshold
      float dynamic_range = morse_decoder.signal_peak - morse_decoder.noise_floor;
      float base_threshold, hysteresis;

      // guard against extremely low dynamic range
      if (dynamic_range < 0.5f) {
        morse_decoder.threshold = morse_decoder.noise_floor + 0.25f;
      } else {
        // base threshold at 25% into the signal above noise
        base_threshold = morse_decoder.noise_floor + 0.25f * dynamic_range;
        hysteresis = 0.1f * dynamic_range;  // 10% of signal range

        // add or subtract hysteresis depending on state
        if (morse_decoder.m_state == STATE_SPACE) {
          // make it harder to trigger a dot/dash: raise threshold
          morse_decoder.threshold = base_threshold + hysteresis;
        } else {
          // make it easier to end a pulse: lower threshold
          morse_decoder.threshold = base_threshold - hysteresis;
        }
      }

      // step 4: state machine processing with edge detection
      for (int i = 0; i < BLOCK_SIZE; i++) {
        bool signal_present = (envelope[i] > morse_decoder.threshold);
        // only count duration when signal matches current state
        if ((morse_decoder.m_state == STATE_MARK && signal_present) ||
            (morse_decoder.m_state == STATE_SPACE && !signal_present)) {
          morse_decoder.duration++;
        }

        // check for state transitions
        if (morse_decoder.m_state == STATE_SPACE && signal_present) {
          // transition from space to mark
          uint32_t space_duration = morse_decoder.duration;
          if (space_duration >= get_min_space_duration()) {
            handle_space_end(space_duration);
          }
          morse_decoder.m_state = STATE_MARK;
          morse_decoder.duration = 1;
          morse_decoder.last_calibration_time = millis();
        }
        else if (morse_decoder.m_state == STATE_MARK && !signal_present) {
          // transition from mark to space
          uint32_t mark_duration = morse_decoder.duration;
          if (mark_duration >= get_min_mark_duration()) {
            handle_mark_end(mark_duration);

            // collect timing data for calibration
            float units = (float)mark_duration / morse_decoder.unit_samples;
            if (units < 1.8f) {
              // dot
              dot_durations[dot_index] = mark_duration;
              dot_index = (dot_index + 1) % 10;
              if (dot_count < 10) dot_count++;
            } else {
              // dash
              dash_durations[dash_index] = mark_duration;
              dash_index = (dash_index + 1) % 10;
              if (dash_count < 10) dash_count++;
            }

            // perform calibration if we have enough data
            if (dot_count >= 5) {
              uint32_t sum = 0;
              for (int j = 0; j < dot_count; j++) {
                sum += dot_durations[j];
              }
              avg_dot_duration = sum / dot_count;

              // update unit_samples based on dot duration
              morse_decoder.unit_samples = (morse_decoder.unit_samples * 7 + avg_dot_duration * 3) / 10;
              update_gap_thresholds();
            }
            else if (dash_count >= 3) {
              uint32_t sum = 0;
              for (int j = 0; j < dash_count; j++) {
                sum += dash_durations[j];
              }
              avg_dash_duration = sum / dash_count;

              // update unit_samples based on dash duration (divide by 3 for dot equivalent)
              morse_decoder.unit_samples = (morse_decoder.unit_samples * 7 + (avg_dash_duration / 3) * 3) / 10;
              update_gap_thresholds();
            }
          }
          morse_decoder.m_state = STATE_SPACE;
          morse_decoder.duration = 1;
          morse_decoder.last_calibration_time = millis();
        }
      }
    } else {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // handle calibration timeout (only if morse tab is active and decoder is on)
    if (lv_tabview_get_tab_act(tabview) == TAB_MORSE_REF && morse_on) {
      if (millis() - morse_decoder.last_calibration_time > 5000) {
        // flush last pending symbol
        decode_symbol();

        // reset to default speed (20WPM)
        morse_decoder.unit_samples = (SAMPLE_RATE * 60) / 1000;
        update_gap_thresholds();

        // reset timing calibration arrays
        memset(dot_durations, 0, sizeof(dot_durations));
        memset(dash_durations, 0, sizeof(dash_durations));
        dot_index = 0;
        dash_index = 0;
        dot_count = 0;
        dash_count = 0;

        // reset decoder state
        morse_decoder.m_state = STATE_SPACE;
        morse_decoder.duration = 0;
        morse_decoder.last_calibration_time = millis();

        // Serial.println("Morse decoder reset due to timeout");
      }
    }
  }
}

// update handle_mark_end to remove the individual calibration
void handle_mark_end(uint32_t duration) {
  // calculate dynamic minimum duration
  const uint32_t absolute_min = SAMPLE_RATE * 10 / 1000;  // 10ms
  uint32_t min_duration = morse_decoder.unit_samples * 3 / 10;
  min_duration = (min_duration > absolute_min) ? min_duration : absolute_min;

  // filter noise and invalid signals
  if (duration < min_duration) return;

  // calculate duration in units
  float units = (float)duration / morse_decoder.unit_samples;

  // dot classification (<1.8 units)
  if (units < 1.8f) {
    if (morse_decoder.sym_len < sizeof(morse_decoder.symbol) - 1) {
      morse_decoder.symbol[morse_decoder.sym_len++] = '.';
      morse_ticker_put('.');
    }
  }
  // dash classification (>=1.8 units)
  else {
    if (morse_decoder.sym_len < sizeof(morse_decoder.symbol) - 1) {
      morse_decoder.symbol[morse_decoder.sym_len++] = '-';
      morse_ticker_put('-');
    }
  }

  // calculate WPM
  float wpm = 1200.0 / (morse_decoder.unit_samples * 1000.0 / SAMPLE_RATE);
}

// add character to buffer
void morse_buffer_put(char c) {
  uint8_t next_head = (morse_head + 1) % MORSE_BUF_SIZE;
  if (next_head != morse_tail) {
    morse_buffer[morse_head] = c;
    morse_head = next_head;
  } else {
    // buffer overflow
  }
}

// check if new characters are in the buffer
void check_morse_buffer() {
  while (morse_tail != morse_head) {
    char ch_ = morse_buffer[morse_tail];
    morse_tail = (morse_tail + 1) % MORSE_BUF_SIZE;
    add_morse_char(ch_);
  }
}

// add character to string and update display
void add_morse_char(char ch_) {
  morse_full_buffer += ch_;
  // limit
  if (morse_full_buffer.length() > 400) {
    morse_full_buffer = morse_full_buffer.substring(morse_full_buffer.length() - 400);
  }
  lv_label_set_text(lbl_morse_text, morse_full_buffer.c_str());
  lv_obj_scroll_to_y(obj_morse_text_container, lv_obj_get_height(lbl_morse_text), LV_ANIM_OFF);
}

void morse_ticker_put(char ch_) {
  uint8_t next_head = (morse_ticker_head + 1) % MORSE_TICKER_BUF_SIZE;
  if (next_head != morse_ticker_tail) {
    morse_ticker_buf[morse_ticker_head] = ch_;
    morse_ticker_head = next_head;
  } else {
    morse_ticker_tail = (morse_ticker_tail + 1) % MORSE_TICKER_BUF_SIZE;
    morse_ticker_buf[morse_ticker_head] = ch_;
    morse_ticker_head = next_head;
  }
  morse_ticker_changed = true;
}

void draw_morse_ticker() {
  morseSprite.fillSprite(TFT_BLACK);
  int mid_y = (morseSprite.height() - morse_dot_height) / 2;
  // calc total length of buffer
  int total = 0;
  int i = morse_ticker_tail;
  while (i != morse_ticker_head) {
    char ch_ = morse_ticker_buf[i];
    if (ch_ == '.') total += morse_dot_width + morse_spacing;
    else if (ch_ == '-') total += morse_dash_width + morse_spacing;
    else if (ch_ == ' ') total += morse_space_width + morse_spacing;
    else if (ch_ == '|') total += morse_space_width + morse_spacing2;
    i = (i + 1) % MORSE_TICKER_BUF_SIZE;
  }
  // scroll offset
  if (total > morseSprite.width()) {
    morse_scroll_offset = total - morseSprite.width();
  } else {
    morse_scroll_offset = 0;
  }
  i = morse_ticker_tail;
  int draw_x = -morse_scroll_offset;
  while (i != morse_ticker_head && draw_x < morseSprite.width()) {
    char ch_ = morse_ticker_buf[i];
    int symbol_width = 0;
    int symbol_spacing = (ch_ == '|') ? morse_spacing2 : morse_spacing;
    // get width of symbol
    if (ch_ == '.') symbol_width = morse_dot_width;
    else if (ch_ == '-') symbol_width = morse_dash_width;
    else if (ch_ == ' ') symbol_width = morse_space_width;
    else if (ch_ == '|') symbol_width = morse_space_width;
    // only draw if entire symbol fits inside the sprite
    if (draw_x >= 0 && draw_x + symbol_width <= morseSprite.width()) {
      if (ch_ == '.') {
        morseSprite.fillCircle(draw_x + morse_dot_width / 2, mid_y + morse_dot_height / 2, morse_dot_width / 2, TFT_WHITE);
      } else if (ch_ == '-') {
        morseSprite.fillRect(draw_x, mid_y, morse_dash_width, 3, TFT_WHITE);
      }
    }
    draw_x += symbol_width + symbol_spacing;
    i = (i + 1) % MORSE_TICKER_BUF_SIZE;
  }
  morseSprite.pushSprite(54, 15);
}

// presets

static void event_presets_sel0(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_presets_sel = 0;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(0, TAB_PRESETS_REF,  cb_presets_selection0);
  }
}

static void event_presets_sel1(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_presets_sel = 1;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(1, TAB_PRESETS_REF,  cb_presets_selection1);
  }
}

static void event_presets_sel2(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_presets_sel = 2;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(2, TAB_PRESETS_REF,  cb_presets_selection2);
  }
}

static void event_presets_sel3(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_presets_sel = 3;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(3, TAB_PRESETS_REF,  cb_presets_selection3);
  }
}

static void event_presets_sel4(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_presets_sel = 4;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(4, TAB_PRESETS_REF,  cb_presets_selection4);
  }
}

static void event_presets_sel5(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_presets_sel = 5;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(5, TAB_PRESETS_REF,  cb_presets_selection5);
  }
}

static void event_presets_sel6(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_presets_sel = 6;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(6, TAB_PRESETS_REF,  cb_presets_selection6);
  }
}

static void event_presets_sel7(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_presets_sel = 7;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(7, TAB_PRESETS_REF,  cb_presets_selection7);
  }
}

static void event_presets_sel8(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_presets_sel = 8;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(8, TAB_PRESETS_REF,  cb_presets_selection8);
  }
}

static void event_presets_sel9(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_presets_sel = 9;
      set_presets_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
  if (event == LV_EVENT_DOUBLE_CLICKED) {
    keyboard_start(9, TAB_PRESETS_REF,  cb_presets_selection9);
  }
}

void set_presets_selection_buttons_state(void) {
  for (int i = 0; i < sizeof(preset_checkboxes) / sizeof(preset_checkboxes[0]); i++) {
    bool should_be_checked = (i == audio_presets_sel);
    bool is_checked = (lv_obj_get_state(preset_checkboxes[i]) & LV_STATE_CHECKED);
    // Only change state if it doesn't match what we want
    if (should_be_checked && !is_checked) {
      lv_obj_add_state(preset_checkboxes[i], LV_STATE_CHECKED);
    } else if (!should_be_checked && is_checked) {
      lv_obj_clear_state(preset_checkboxes[i], LV_STATE_CHECKED);
    }
  }
}

void set_presets_labels(void) {
  for (int i = 0; i < sizeof(preset_checkboxes) / sizeof(preset_checkboxes[0]); i++) {
    // add one space for end sign otherwise trunctated
    audio_presets_msg[i].toCharArray(printbuf, 21);
    lv_checkbox_set_text(preset_checkboxes[i], printbuf);
  }
}

void set_presetsmain_labels(void) {
  for (int i = 0; i < sizeof(preset_mainbuttons) / sizeof(preset_mainbuttons[0]); i++) {
    // add one space for end sign otherswise trunctated
    audio_presets_msg[i].toCharArray(printbuf, 21);
    lv_label_set_text(preset_mainbuttons[i], printbuf);
  }
}

static void event_mainpresets_sel0(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(0);
  }
}

static void event_mainpresets_sel1(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(1);
  }
}

static void event_mainpresets_sel2(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(2);
  }
}

static void event_mainpresets_sel3(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(3);
  }
}

static void event_mainpresets_sel4(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(4);
  }
}

static void event_mainpresets_sel5(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(5);
  }
}

static void event_mainpresets_sel6(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(6);
  }
}

static void event_mainpresets_sel7(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(7);
  }
}

static void event_mainpresets_sel8(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(8);
  }
}

static void event_mainpresets_sel9(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    mainpresets_load_return(9);
  }
}

void mainpresets_load_return(uint8_t selected) {
  audio_presets_sel = selected;
  set_presets_selection_buttons_state();
  load_presets(audio_presets_sel);
  lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
}

static void event_presets_set(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  if (event != LV_EVENT_CLICKED) return;
  for (uint8_t i = 0; i < sizeof(preset_checkboxes) / sizeof(preset_checkboxes[0]); i++) {
    if (lv_obj_get_state(preset_checkboxes[i]) & LV_STATE_CHECKED) {
      save_presets(i);
      break; // assuming only one checkbox can be checked at a time
    }
  }
}

static void event_presets_recall(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  if (event != LV_EVENT_CLICKED) return;
  for (uint8_t i = 0; i < sizeof(preset_checkboxes) / sizeof(preset_checkboxes[0]); i++) {
    if (lv_obj_get_state(preset_checkboxes[i]) & LV_STATE_CHECKED) {
      load_presets(i);
      break; // assuming only one checkbox can be checked at a time
    }
  }
}

// scope screen

void scope_draw_screen(boolean fullscreen) {
  if (fullscreen) {
    TFT.drawRect(0, 0, 480, 251, TFT_BLUE);
  }
  TFT.drawLine(0, 125, 479, 125, TFT_GREEN);
}

void scope_sample_plot(void) {
  // setup
  int32_t scope_index = 0;
  float dc_offset = 0.0f;

  memcpy(audio_sample_buffer, audio_scope_buffer, 1024 * sizeof(int32_t));

  // reset sampling state
  audio_scope_ready = false;
  audio_scope_fill_block_count = 0;
  audio_scope_trigger = true;

  // calculate DC offset (average of all samples)
  for (int i = 0; i < 1024; i++) {
    dc_offset += (audio_sample_buffer[i] >> 8) * (1.0f / 8388608.0f);
  }
  dc_offset /= 1024.0f;

  // autoscale calculation
  if (audio_scope_autoscale) {
    float peak = 0.0f;
    for (int i = 0; i < 1024; i++) {
      float sample_val = (audio_sample_buffer[i] >> 8) * (1.0f / 8388608.0f) - dc_offset;
      float abs_sample = fabsf(sample_val);
      if (abs_sample > peak) peak = abs_sample;
    }
    if (peak < 0.0001f) peak = 0.0001f;
    audio_scope_scale = 120.0f / peak;
    if (audio_scope_scale < 1) audio_scope_scale = 1;
    audio_scope_autoscale = false;
  }

  // scale and clamp samples (with DC offset removal)
  for (int i = 0; i < 1024; i++) {
    float sample_val = (audio_sample_buffer[i] >> 8) * (1.0f / 8388608.0f) - dc_offset;
    sample_val *= audio_scope_scale;
    // clamp to vertical bounds with proper rounding
    if (sample_val > 124.0f) sample_val = 124.0f;
    if (sample_val < -124.0f) sample_val = -124.0f;
    audio_sample_buffer[i] = (int32_t)(sample_val + (sample_val >= 0 ? 0.5f : -0.5f)); // Proper rounding
  }

  // trigger point detection at zero crossing
  for (int i = 1; i < 500; i++) {
    if (audio_sample_buffer[i] >= 0 && audio_sample_buffer[i - 1] < 0) {
      scope_index = i;
      break;
    }
  }

  // clear previous trace
  for (uint16_t p = 0; p < 476; p++) {
    TFT.drawLine(p + 1, 125 - audio_scope_previous[p],
                 p + 2, 125 - audio_scope_previous[p + 1], TFT_BLACK);
  }
  scope_draw_screen(false);
  // draw new trace
  for (uint16_t p = 0; p < 476; p++) {
    TFT.drawLine(p + 1, 125 - audio_sample_buffer[scope_index + p],
                 p + 2, 125 - audio_sample_buffer[scope_index + p + 1], TFT_WHITE);
  }

  // store current trace for next erase operation
  for (uint16_t pointer = 0; pointer < 478; pointer++) {
    audio_scope_previous[pointer] = audio_sample_buffer[scope_index + pointer];
  }
}

void scope_clear(void) {
  // blank out previous plot with background color
  for (uint16_t pointer = 0; pointer < 476 ; pointer++) {
    TFT.drawLine(pointer + 1, 125 - audio_scope_previous[pointer],
                 pointer + 2, 125 - audio_scope_previous[pointer + 1], TFT_BLACK);
  }
  scope_draw_screen(false);
  // keep all
  for (uint16_t pointer = 0; pointer < 478 ; pointer++) {
    audio_scope_previous[pointer] = 0;
  }
}

// spectrum screen

void spectrum_sample(void) {
  // Copy samples
  memcpy(audio_sample_buffer, audio_scope_buffer, 2048 * sizeof(int32_t));

  // restart sampling
  audio_scope_ready = false;
  audio_scope_fill_block_count = 0;
  audio_scope_trigger = true;

  // convert to float with proper 24-bit scaling
  for (int i = 0; i < FFT_SIZE; i++) {
    audio_spectrum_vReal[i] = (float)audio_sample_buffer[i] * (1.0f / 8388608.0f);
    audio_spectrum_vImag[i] = 0.0f;
  }

  // run FFT
  FFT.dcRemoval();
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // improved normalization
  const float window_coherent_gain = 0.54f; // Hamming window
  const float fft_norm = 2.0f / (FFT_SIZE * window_coherent_gain);

  // adjust this to set 0dB reference
  const float reference_level = 30.0f;

  // dBFS conversion with calibration
  for (int i = 0; i < FFT_SIZE / 2; i++) {
    float magnitude = audio_spectrum_vReal[i] * fft_norm;
    magnitude = max(magnitude, 1e-12f); // Avoid log(0)
    audio_spectrum_vReal[i] = 20.0f * log10f(magnitude) - reference_level;
  }

  // compute band averages (in dBFS)
  for (int band = 0; band < NUM_BANDS; band++) {
    int startBin = (band * NUM_BINS_TO_PROCESS) / NUM_BANDS;
    int endBin   = ((band + 1) * NUM_BINS_TO_PROCESS) / NUM_BANDS;
    float sum = 0;
    int count = 0;
    for (int bin = startBin; bin < endBin; bin++) {
      if (audio_spectrum_vReal[bin] > -80.0f) { // ignore below -80 dBFS
        sum += audio_spectrum_vReal[bin];
        count++;
      }
    }
    float average = (count > 0) ? (sum / count) : -80.0f;
    audio_spectrum_band_values[band] = average;
  }

  // normalize to 0-127 for display with adjustable range
  const float DISPLAY_MIN_DB = -70.0f;
  const float DISPLAY_MAX_DB = 6.0f; // Increased headroom
  const float db_range = DISPLAY_MAX_DB - DISPLAY_MIN_DB;
  uint8_t peakvalue = 0;
  uint8_t peakpointer = 0;
  for (int band = 0; band < NUM_BANDS; band++) {
    float dbVal = audio_spectrum_band_values[band];
    // clamp to display range
    if (dbVal < DISPLAY_MIN_DB) dbVal = DISPLAY_MIN_DB;
    if (dbVal > DISPLAY_MAX_DB) dbVal = DISPLAY_MAX_DB;
    // map to 0-127
    uint8_t pixVal = (uint8_t)(127.0f * (dbVal - DISPLAY_MIN_DB) / db_range);
    audio_spectrum_band_values[band] = pixVal;
    if (audio_spectrum_peaktrigger) {
      if (pixVal > peakvalue) {
        peakvalue = pixVal;
        peakpointer = band;
      }
    }
  }
  // peak trigger handling
  if (audio_spectrum_peaktrigger) {
    audio_spectrum_cursor = peakpointer;
    audio_spectrum_peaktrigger = false;
    spectrum_update_cursor(audio_spectrum_cursor);
  }
  if (audio_spectrum_bandpasstrigger) {
    spectrum_calc_bandpass();
    audio_spectrum_bandpasstrigger = false;
    spectrum_update_bandpass_marker(audio_spectrum_low_freq, audio_spectrum_high_freq);
  }
}

void spectrum_show(void) {
  uint16_t startpos = 1;
  for (byte band = 0; band < NUM_BANDS; band++) {
    int height = audio_spectrum_band_values[band];
    if (height >= 0) {
      TFT.fillRect(startpos, 129 - height, 4, height, TFT_GREEN);
      TFT.fillRect(startpos, 1, 4, 128 - height, TFT_BLACK);
    }
    startpos += 4;
  }
}

void spectrum_clear(void) {
  uint16_t startpos = 1;
  for (byte band = 0; band < NUM_BANDS; band++) {
    TFT.fillRect(startpos, 1, 4, 128, TFT_BLACK);
    startpos += 4;
  }
}

void spectrum_draw_screen(boolean fullscreen) {
  if (fullscreen) {
    //  TFT.drawRect(0, 0, 480, 247, TFT_BLUE);
    TFT.drawRect(0, 0, 480, 130, TFT_BLUE);
  }
}

// update cursor and value
void spectrum_update_cursor(uint8_t s_cursor) {
  static const uint8_t startX = 1;
  static const uint8_t bandWidth = 4;
  static const uint8_t spacing = 0;
  static const uint8_t cursorY = 135;

  // calculate X-position
  uint16_t x = startX + s_cursor * (bandWidth + spacing);

  // erase previous cursor if any
  if (audio_spectrum_cursor_last_pos != 255) {
    uint16_t lastX = startX + audio_spectrum_cursor_last_pos * (bandWidth + spacing);
    TFT.fillTriangle(lastX + 0, cursorY, lastX + 2, cursorY - 4, lastX + 4, cursorY, TFT_BLACK);
  }
  // draw new cursor
  TFT.fillTriangle(x + 0, cursorY, x + 2, cursorY - 4, x + 4, cursorY, TFT_WHITE);
  // store current position for erase
  audio_spectrum_cursor_last_pos = s_cursor;
  const double binWidth = 48000.0 / 2048.0;
  const int maxBin = 170;
  const int bandCount = 119;
  double bandBinStart = (audio_spectrum_cursor * (double)maxBin) / bandCount;
  double bandBinMid = bandBinStart + ((double)maxBin / bandCount) / 2.0;
  double freq = bandBinMid * binWidth;
  static char buf[20];
  snprintf(buf, sizeof(buf), "f: %.0f Hz", freq);
  lv_label_set_text(lbl_spectrum_cursor, buf);
}

// horizontal line between low and high freq limits
void spectrum_update_bandpass_marker(int32_t startFreq, int32_t endFreq) {
  static const uint8_t startX = 1;
  static const uint8_t bandWidth = 4;
  static const uint8_t spacing = 0;
  static const uint8_t markerY = 145;
  const double binWidth = 48000.0 / 2048.0; // Hz per FFT bin
  const int maxBin = 170;                   // highest displayed bin
  const int bandCount = 119;                // number of beams

  // calc mid of band X position
  auto freq_to_midX = [&](float freq) -> uint16_t {
    double bin   = freq / binWidth;
    double bandf = (bin / maxBin) * bandCount;
    int band     = (int)bandf;
    if (band < 0) band = 0;
    if (band >= bandCount) band = bandCount - 1;
    // mid of band
    return startX + band * (bandWidth + spacing) + (bandWidth / 2);
  };
  uint16_t x1 = freq_to_midX(startFreq);
  uint16_t x2 = freq_to_midX(endFreq);
  if (x2 < x1) std::swap(x1, x2);

  // for now, since we dont update any filters from the spectrum screen, the line will be drawn only once and there is no old line to erase

  //  // erase old marker
  //  static uint16_t old_x1 = 0, old_x2 = 0;
  //  if (old_x2 > old_x1) {
  //    TFT.fillRect(old_x1, markerY - 1, old_x2 - old_x1, 2, TFT_BLACK);
  //    TFT.fillCircle(old_x1, markerY, 2, TFT_BLACK);
  //    TFT.fillCircle(old_x2, markerY, 2, TFT_BLACK);
  //  }

  // draw the new line
  TFT.fillRect(x1, markerY - 1, x2 - x1, 2, TFT_SKYBLUE);
  // end markers
  TFT.fillCircle(x1, markerY, 2, TFT_SKYBLUE);
  TFT.fillCircle(x2, markerY, 2, TFT_SKYBLUE);

  //  // keep old position
  //  old_x1 = x1;
  //  old_x2 = x2;

}

// calculate bandpass to show based on all filters combined
void spectrum_calc_bandpass(void) {
  // start with highpass and lowpass boundaries
  int32_t lowFreq  = filter_highpass_freq;
  int32_t highFreq = filter_lowpass_freq;
  // use peaking filter if active
  if (filter_peak_on) {
    int32_t peakLow  = filter_peak_bandpass_freq - (filter_peak_bandwidth / 2);
    int32_t peakHigh = filter_peak_bandpass_freq + (filter_peak_bandwidth / 2);
    // use smallest overlap
    if (peakLow  > lowFreq)  lowFreq  = peakLow;
    if (peakHigh < highFreq) highFreq = peakHigh;
  }
  // limit to 100 - 4000 Hz
  if (lowFreq  < 100)  lowFreq  = 100;
  if (highFreq > 4000) highFreq = 4000;
  // make sure limits are sane
  if (highFreq < lowFreq) highFreq = lowFreq;
  // store
  audio_spectrum_low_freq  = lowFreq;
  audio_spectrum_high_freq = highFreq;
}


// test screen

static void event_test_switch(lv_event_t * e) {
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    test_switch_on_off();
  }
}

// actual switching is disconnected from event since we get multiple triggers for value changed and toggle does not trigger clicked
void test_switch_on_off(void) {
  if (lv_obj_get_state(cb_test_on) & LV_STATE_CHECKED) {
    set_sine_frequency1(audio_test_frequency1);
    set_sine1_on(true);
    // keep current setting
    audio_input_selected_previous = audio_input_selected;
    // switch to sine input
    select_input_source(2);
  }
  else {
    // switch back to regular input
    select_input_source(audio_input_selected_previous);
    set_sine1_on(false);
  }
}

void increment_test_frequency1_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_increment(spinbox_test_frequency_1);
  }
}

static void decrement_test_frequency1_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
    lv_spinbox_decrement(spinbox_test_frequency_1);
  }
}

static void frequency1_change_event_cb(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_VALUE_CHANGED) {
    audio_test_frequency1 = lv_spinbox_get_value(spinbox_test_frequency_1);
    set_sine_frequency1(audio_test_frequency1);
  }
}

static void event_test_signal_gain_change(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_VALUE_CHANGED) {
    int32_t audio_test_signal_gain_previous = audio_test_signal_gain;
    audio_test_signal_gain = lv_slider_get_value(obj);
    if (audio_test_signal_gain != audio_test_signal_gain_previous) {
      set_test_signal_gain();
    }
  }
}

// update test signal gain
void update_test_signal_gain(void) {
  lv_slider_set_value(slider_test_signal_gain, audio_test_signal_gain, LV_ANIM_OFF);
  set_test_signal_gain();
}

// update dsp test gain potmeter
void set_test_signal_gain(void) {
  // limit range
  float db_value = -50.0f + (float)audio_test_signal_gain;
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  dsp.volume(MOD_SINGLE1_3_GAIN1940ALGNS6_ADDR, db_value);
  xSemaphoreGive(i2cMutex);
}

void set_sine_frequency1(int32_t freq) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  dsp.sineSource(MOD_STATIC_TONE1_ALG0_MASK_ADDR, freq);
  xSemaphoreGive(i2cMutex);
}

void set_sine1_on(boolean switch_on) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  if (switch_on) {
    dsp.safeload_write(MOD_TONE1_ALG0_ON_ADDR, 0x00800000);
  }
  else {
    dsp.safeload_write(MOD_TONE1_ALG0_ON_ADDR, 0x00000000);
  }
  xSemaphoreGive(i2cMutex);
}

// settings

static void event_settings_select0(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_input_selected = 0;
      select_input_source(audio_input_selected);
      set_settings_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
}

static void event_settings_select1(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    if (lv_obj_get_state(obj) & LV_STATE_CHECKED) {
      audio_input_selected = 1;
      select_input_source(audio_input_selected);
      set_settings_selection_buttons_state();
    }
    else {
      // prevent unchecking
      lv_obj_add_state(obj, LV_STATE_CHECKED);
    }
  }
}

void set_settings_selection_buttons_state(void) {
  switch (audio_input_selected) {
    case 0:
      lv_obj_add_state(cb_settings_input0, LV_STATE_CHECKED);
      lv_obj_clear_state(cb_settings_input1, LV_STATE_CHECKED);
      break;

    case 1:
      lv_obj_clear_state(cb_settings_input0, LV_STATE_CHECKED);
      lv_obj_add_state(cb_settings_input1, LV_STATE_CHECKED);
      break;
  }
}

// general

void spinbox_show_cursor(lv_obj_t * spinbox, bool en) {
  lv_obj_set_style_text_color(spinbox, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_text_color(spinbox, (en) ? lv_color_white() : lv_color_black(), LV_PART_CURSOR);
  lv_obj_set_style_bg_opa(spinbox, (en) ? 255 : 0, LV_PART_CURSOR);
}

void checkbox_toggle(lv_obj_t *cbox) {
  if (lv_obj_has_state(cbox, LV_STATE_CHECKED)) {
    lv_obj_clear_state(cbox, LV_STATE_CHECKED);  // Uncheck
  } else {
    lv_obj_add_state(cbox, LV_STATE_CHECKED);    // Check
  }
}

void update_input(void) {
  select_input_source(audio_input_selected);
  set_settings_selection_buttons_state();
}

void keyboard_start(uint8_t caller, uint8_t tab, lv_obj_t * obj ) {
  kb_caller = caller;
  kb_returntab = tab;
  kb_text = "";

  //kb_text = lv_label_get_text(obj);
  if (kb_caller >= 0 and kb_caller < 10) {
    kb_text = lv_checkbox_get_text(obj);
    lv_textarea_set_max_length(edit_txtentry, 20);
  }

  kb_text.toCharArray(printbuf, 40);
  lv_textarea_set_text(edit_txtentry, printbuf);
  lv_tabview_set_act(tabview, TAB_EDIT_REF, LV_ANIM_OFF);
}

static void event_keyboardhandler(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_READY) {
    String kbtxt = lv_textarea_get_text(edit_txtentry);
    if (kb_caller >= 0 and kb_caller < 10) {
      audio_presets_msg[kb_caller] = kbtxt;
      save_presets_names();
      set_presets_labels();
      set_presetsmain_labels();
    }
    lv_tabview_set_act(tabview, kb_returntab, LV_ANIM_OFF);
  }
  if (event == LV_EVENT_CANCEL) {
    lv_tabview_set_act(tabview, kb_returntab, LV_ANIM_OFF);
  }
}

static void event_calibrate(lv_event_t * e)
{
  lv_event_code_t event = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if (event == LV_EVENT_CLICKED) {
    calibratescreen();
    lv_tabview_set_act(tabview, TAB_MAIN_REF, LV_ANIM_OFF);
  }
}

void calibratescreen() {
  TFT.fillScreen(TFT_BLACK);
  // clear this namespace
  preferences.begin("settings", false);
  preferences.clear();
  preferences.end();

  TFT.calibrateTouch(audio_setting_calibration_data, TFT_WHITE, TFT_RED, 14);

  //  Serial.println("New calibration data:");
  //  for (int i = 0; i < 8; i++) {
  //    Serial.printf("  %d: %d\n", i, audio_setting_calibration_data[i]);
  //  }

  save_settings();
  TFT.setTouchCalibrate(audio_setting_calibration_data);
}

void save_settings() {
  preferences.begin("settings", false);
  preferences.putBytes("calib", audio_setting_calibration_data, sizeof(audio_setting_calibration_data));
  preferences.end();
}

bool load_settings() {
  preferences.begin("settings", true);
  size_t len = preferences.getBytes("calib", audio_setting_calibration_data, sizeof(audio_setting_calibration_data));
  preferences.end();
  if (len != sizeof(audio_setting_calibration_data)) {
    // Serial.println("No valid calibration data found.");
    return false;
  }
  // check if all 0
  bool allzero = true;
  for (int i = 0; i < 8; i++) {
    if (audio_setting_calibration_data[i] != 0) {
      allzero = false;
      break;
    }
  }
  if (allzero) {
    // Serial.println("Calibration data empty.");
    return false;
  }
  //  Serial.println("Calibration data loaded:");
  //  for (int i = 0; i < 8; i++) {
  //    Serial.printf("  %d: %d\n", i, audio_setting_calibration_data[i]);
  //  }
  return true;
}

// reset all settings and save
void reset_settings() {
  preferences.begin("settings", false); // Open Preferences in read-write mode
  preferences.clear(); // Clear all stored data
  clear_settings();
  save_settings(); // Save defaults
  preferences.end(); // Close Preferences
}

// reset all settings to default
void clear_settings() {
  for (int i = 0; i < 8; i++) {
    audio_setting_calibration_data[i] = 0; // Default value
  }
}

// debug routine in case of problems
// alternative enable monitoring in lvgl
void print_memory_info_to_serial() {
  // gather memory info
  size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
  size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  size_t total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
  size_t stack_free = uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);
  // print memory information to the serial monitor
  Serial.println("=== Memory Info ===");
  Serial.printf("Heap: %u / %u bytes free\n", (unsigned int)free_heap, (unsigned int)total_heap);
  // if PSRAM is available, display its info
  if (total_psram > 0) {
    Serial.printf("PSRAM: %u / %u bytes free\n", (unsigned int)free_psram, (unsigned int)total_psram);
  } else {
    Serial.println("PSRAM: Not available");
  }
  // display stack memory info
  Serial.printf("Stack: %u bytes free\n", (unsigned int)stack_free);
  Serial.println("====================");
  // free DRAM (internal RAM where `dram0_0_seg` is located)
  size_t free_dram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  size_t largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  size_t total_dram = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
  Serial.printf("Total DRAM: %d bytes\n", total_dram);
  Serial.printf("Free DRAM: %d bytes\n", free_dram);
  Serial.printf("Largest Free Block in DRAM: %d bytes\n", largest_free_block);
  lv_mem_monitor_t mon;
  lv_mem_monitor(&mon);
  Serial.printf("mon.total_size: %d \n", mon.total_size);
  Serial.printf("mon.free_cnt: %d \n", mon.free_cnt);
  Serial.printf("mon.free_size: %d \n", mon.free_size);
  Serial.printf("mon.free_biggest_size: %d \n", mon.free_biggest_size);
  Serial.printf("mon.used_cnt: %d \n", mon.used_cnt);
  Serial.printf("mon.max_used: %d \n", mon.max_used);
  Serial.printf("mon.used_pct: %d \n", mon.used_pct);
  Serial.printf("mon.frag_pct: %d \n", mon.frag_pct);
}

void get_event_name(lv_event_code_t  event) {
  switch (event) {
    case LV_EVENT_ALL:
      Serial.println( "LV_EVENT_ALL");
      break;
    case LV_EVENT_PRESSED:
      Serial.println( "LV_EVENT_PRESSED");
      break;
    case LV_EVENT_PRESSING:
      Serial.println( "LV_EVENT_PRESSING");
      break;
    case LV_EVENT_PRESS_LOST:
      Serial.println( "LV_EVENT_PRESS_LOST");
      break;
    case LV_EVENT_SHORT_CLICKED:
      Serial.println( "LV_EVENT_SHORT_CLICKED");
      break;
    case LV_EVENT_LONG_PRESSED:
      Serial.println( "LV_EVENT_LONG_PRESSED");
      break;
    case LV_EVENT_LONG_PRESSED_REPEAT:
      Serial.println( "LV_EVENT_LONG_PRESSED_REPEAT");
      break;
    case LV_EVENT_CLICKED:
      Serial.println( "LV_EVENT_CLICKED");
      break;
    case LV_EVENT_RELEASED:
      Serial.println( "LV_EVENT_RELEASED");
      break;
    case LV_EVENT_SCROLL_BEGIN:
      Serial.println( "LV_EVENT_SCROLL_BEGIN");
      break;
    case LV_EVENT_SCROLL_THROW_BEGIN:
      Serial.println( "LV_EVENT_SCROLL_THROW_BEGIN");
      break;
    case LV_EVENT_SCROLL_END:
      Serial.println( "LV_EVENT_SCROLL_END");
      break;
    case LV_EVENT_SCROLL:
      Serial.println( "LV_EVENT_SCROLL");
      break;
    case LV_EVENT_GESTURE:
      Serial.println( "LV_EVENT_GESTURE");
      break;
    case LV_EVENT_KEY:
      Serial.println( "LV_EVENT_KEY");
      break;
    case LV_EVENT_ROTARY:
      Serial.println( "LV_EVENT_ROTARY");
      break;
    case LV_EVENT_FOCUSED:
      Serial.println( "LV_EVENT_FOCUSED");
      break;
    case LV_EVENT_DEFOCUSED:
      Serial.println( "LV_EVENT_DEFOCUSED");
      break;
    case LV_EVENT_LEAVE:
      Serial.println( "LV_EVENT_LEAVE");
      break;
    case LV_EVENT_HIT_TEST:
      Serial.println( "LV_EVENT_HIT_TEST");
      break;
    case LV_EVENT_INDEV_RESET:
      Serial.println( "LV_EVENT_INDEV_RESET");
      break;
    case LV_EVENT_HOVER_OVER:
      Serial.println( "LV_EVENT_HOVER_OVER");
      break;
    case LV_EVENT_HOVER_LEAVE:
      Serial.println( "LV_EVENT_HOVER_LEAVE");
      break;
    case LV_EVENT_COVER_CHECK:
      Serial.println( "LV_EVENT_COVER_CHECK");
      break;
    case LV_EVENT_REFR_EXT_DRAW_SIZE:
      Serial.println( "LV_EVENT_REFR_EXT_DRAW_SIZE");
      break;
    case LV_EVENT_DRAW_MAIN_BEGIN:
      Serial.println( "LV_EVENT_DRAW_MAIN_BEGIN");
      break;
    case LV_EVENT_DRAW_MAIN:
      Serial.println( "LV_EVENT_DRAW_MAIN");
      break;
    case LV_EVENT_DRAW_MAIN_END:
      Serial.println( "LV_EVENT_DRAW_MAIN_END");
      break;
    case LV_EVENT_DRAW_POST_BEGIN:
      Serial.println( "LV_EVENT_DRAW_POST_BEGIN");
      break;
    case LV_EVENT_DRAW_POST:
      Serial.println( "LV_EVENT_DRAW_POST");
      break;
    case LV_EVENT_DRAW_POST_END:
      Serial.println( "LV_EVENT_DRAW_POST_END");
      break;
    case LV_EVENT_DRAW_TASK_ADDED:
      Serial.println( "LV_EVENT_DRAW_TASK_ADDED");
      break;
    case LV_EVENT_VALUE_CHANGED:
      Serial.println( "LV_EVENT_VALUE_CHANGED");
      break;
    case LV_EVENT_INSERT:
      Serial.println( "LV_EVENT_INSERT");
      break;
    case LV_EVENT_REFRESH:
      Serial.println( "LV_EVENT_REFRESH");
      break;
    case LV_EVENT_READY:
      Serial.println( "LV_EVENT_READY");
      break;
    case LV_EVENT_CANCEL:
      Serial.println( "LV_EVENT_CANCEL");
      break;
    case LV_EVENT_CREATE:
      Serial.println( "LV_EVENT_CREATE");
      break;
    case LV_EVENT_DELETE:
      Serial.println( "LV_EVENT_DELETE");
      break;
    case LV_EVENT_CHILD_CHANGED:
      Serial.println( "LV_EVENT_CHILD_CHANGED");
      break;
    case LV_EVENT_CHILD_CREATED:
      Serial.println( "LV_EVENT_CHILD_CREATED");
      break;
    case LV_EVENT_CHILD_DELETED:
      Serial.println( "LV_EVENT_CHILD_DELETED");
      break;
    case LV_EVENT_SCREEN_UNLOAD_START:
      Serial.println( "LV_EVENT_SCREEN_UNLOAD_START");
      break;
    case LV_EVENT_SCREEN_LOAD_START:
      Serial.println( "LV_EVENT_SCREEN_LOAD_START");
      break;
    case LV_EVENT_SCREEN_LOADED:
      Serial.println( "LV_EVENT_SCREEN_LOADED");
      break;
    case LV_EVENT_SCREEN_UNLOADED:
      Serial.println( "LV_EVENT_SCREEN_UNLOADED");
      break;
    case LV_EVENT_SIZE_CHANGED:
      Serial.println( "LV_EVENT_SIZE_CHANGED");
      break;
    case LV_EVENT_STYLE_CHANGED:
      Serial.println( "LV_EVENT_STYLE_CHANGED");
      break;
    case LV_EVENT_LAYOUT_CHANGED:
      Serial.println( "LV_EVENT_LAYOUT_CHANGED");
      break;
    case LV_EVENT_GET_SELF_SIZE:
      Serial.println( "LV_EVENT_GET_SELF_SIZE");
      break;
    case LV_EVENT_INVALIDATE_AREA:
      Serial.println( "LV_EVENT_INVALIDATE_AREA");
      break;
    case LV_EVENT_RESOLUTION_CHANGED:
      Serial.println( "LV_EVENT_RESOLUTION_CHANGED");
      break;
    case LV_EVENT_COLOR_FORMAT_CHANGED:
      Serial.println( "LV_EVENT_COLOR_FORMAT_CHANGED");
      break;
    case LV_EVENT_VSYNC:
      Serial.println( "LV_EVENT_VSYNC");
      break;
    default:
      Serial.println( "UNKNOWN_EVENT");
      break;
  }
}

// VU meter with parametric start positions and speech level
void VU_Meter_Show_Horizontal(byte audio_level, uint16_t start_posx, uint16_t start_posy) {
  uint16_t vu_posx = start_posx;
  uint16_t vu_posy = start_posy;
  for (byte vu_ledcount = 1; vu_ledcount <= 50; vu_ledcount++) {
    if (audio_level >= vu_ledcount) {
      if (vu_ledcount <= 35) {
        TFT.fillRect(vu_posx, vu_posy, 5, 25, TFT_GREEN);
      }
      else if (vu_ledcount > 35 && vu_ledcount <= 43) {
        TFT.fillRect(vu_posx, vu_posy, 5, 25, TFT_ORANGE);
      }
      else {
        TFT.fillRect(vu_posx, vu_posy, 5, 25, TFT_RED);
      }
    }
    else {
      // draw blank
      TFT.fillRect(vu_posx, vu_posy, 5, 25, TFT_BLACK);
    }
    vu_posx += 6; // Move to the next LED position
  }
}

void VU_meter_update(void) {
  VU_Meter_Show_Vertical(audio_vu_meter, 2, 310);
}

// vertical VU meter (fills from bottom to top)
void VU_Meter_Show_Vertical(int8_t audio_level, uint16_t start_posx, uint16_t start_posy) {
  uint16_t vu_posx = start_posx;
  for (byte vu_ledcount = 1; vu_ledcount <= 50; vu_ledcount++) {
    // Adjust the first rectangle position (compensate for top-to-bottom drawing)
    uint16_t vu_posy = start_posy - (vu_ledcount * 6);  // Move up for each LED
    // Now draw the LEDs, starting at the corrected position
    if (audio_level >= vu_ledcount) {
      if (vu_ledcount <= 35) {
        TFT.fillRect(vu_posx, vu_posy, 25, 5, TFT_GREEN);
      }
      else if (vu_ledcount > 35 && vu_ledcount <= 43) {
        TFT.fillRect(vu_posx, vu_posy, 25, 5, TFT_ORANGE);
      }
      else {
        TFT.fillRect(vu_posx, vu_posy, 25, 5, TFT_RED);
      }
    }
    else {
      // Draw blank space if the level is less than the LED count
      TFT.fillRect(vu_posx, vu_posy, 25, 5, TFT_BLACK);
    }
  }
}

void vu_meter_read(void) {
  // readback 1 = full scale (1v dc into the readback module)
  // = 2.7v pp
  // = 1.35v peak
  // = 0.95v rms
  // these values are direct out of the dsp chip, reduced by filters and coupler tx!
  // calibrate all green bars lit for 100mv rms at line output
  int32_t readback = 0;
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  readback = dsp.readBack(MOD_READBACK1_ALG0_VAL0_ADDR, MOD_READBACK1_ALG0_VAL0_VALUES, 3);
  xSemaphoreGive(i2cMutex);
  if (readback & 0x800000) readback |= 0xFF000000; // sign extend
  audio_vu_readback = (float)readback / 524288.0f;
  // calibration
  static const float CAL_FACTOR = 1.00f;   // adjust if needed
  static const float VU_REF_0DB  = 0.11f;  // readback-value at 0 dB (100 mV RMS)
  static const int   LEDS_AT_0DB = 36;     // number of leds on at 0 dB
  static const int   VU_LED_COUNT = 50;

  float corrected_value = audio_vu_readback * CAL_FACTOR;

  // limit
  if (corrected_value < 0.000001f) corrected_value = 0.000001f;

  // map log scale
  // 0 dB reference is VU_REF_0DB → LEDS_AT_0DB leds
  float leds_on;

  if (corrected_value <= VU_REF_0DB) {
    // scale from -40 dB to 0 dB  0..LEDS_AT_0DB
    float norm = corrected_value / VU_REF_0DB;
    float db   = 20.0f * log10f(norm); // negatief bij lagere waarden
    float vu_norm = (db + 40.0f) / 40.0f;
    vu_norm = fminf(fmaxf(vu_norm, 0.0f), 1.0f);
    leds_on = vu_norm * LEDS_AT_0DB;
  } else {
    // above 0 dB → scale extra leds until VU_LED_COUNT
    float over_norm = (corrected_value - VU_REF_0DB) / (1.0f - VU_REF_0DB);
    over_norm = fminf(fmaxf(over_norm, 0.0f), 1.0f);
    leds_on = LEDS_AT_0DB + over_norm * (VU_LED_COUNT - LEDS_AT_0DB);
  }
  audio_vu_meter = (int)(leds_on + 0.5f);
}

// save prefs to set 0-9
void save_presets(int setNumber) {
  String ns_config = "audio" + String(setNumber);

  preferences.begin(ns_config.c_str(), false);
  preferences.putUChar("version", 1); // Mark this set as valid
  // main screen
  preferences.putInt("gain", audio_gain);
  preferences.putInt("lineoutput", audio_line_out);
  preferences.putInt("spkroutput", audio_speaker);
  preferences.putInt("hpoutput", audio_headphone);
  // notch screen
  preferences.putInt("notchfreq0", filter_notch_freq0);
  preferences.putInt("notchfreq1", filter_notch_freq1);
  preferences.putInt("notchfreq2", filter_notch_freq2);

  preferences.putInt("notchwidth0", filter_notch_bandwidth0);
  preferences.putInt("notchwidth1", filter_notch_bandwidth1);
  preferences.putInt("notchwidth2", filter_notch_bandwidth2);

  preferences.putBool("notchon0", filter_notch_on0);
  preferences.putBool("notchon1", filter_notch_on1);
  preferences.putBool("notchon2", filter_notch_on2);

  // filter screen
  preferences.putInt("filterhi", filter_highpass_freq);
  preferences.putInt("filterlo", filter_lowpass_freq);
  preferences.putInt("filterbp", filter_bandpass_freq);
  preferences.putInt("filterbw", filter_bandwidth);
  preferences.putUChar("audiosel", audio_filter_mode);

  // peaking screen
  preferences.putInt("peakbp", filter_peak_bandpass_freq);
  preferences.putInt("peakbw", filter_peak_bandwidth);
  preferences.putBool("peakon", filter_peak_on);

  // dynamic screen
  preferences.putBool("dyndenoiseon", audio_dynamic_denoiser);
  preferences.putBool("dynnotchon", audio_dynamic_notch);
  preferences.putBool("dynagcon", audio_dynamic_agc);

  // compressor screen
  preferences.putInt("compthreshold", audio_compressor_threshold);
  preferences.putInt("compratio", audio_compressor_ratio);
  preferences.putInt("comppostgain", audio_compressor_postgain);
  preferences.putInt("compnoisegate", audio_compressor_noisegate);

  // equaliser screen
  preferences.putInt("eq0", audio_eq0);
  preferences.putInt("eq1", audio_eq1);
  preferences.putInt("eq2", audio_eq2);
  preferences.putInt("eq3", audio_eq3);
  preferences.putInt("eq4", audio_eq4);
  preferences.putInt("eq5", audio_eq5);
  preferences.putInt("eq6", audio_eq6);
  preferences.putInt("eq7", audio_eq7);
  preferences.putInt("eq8", audio_eq8);
  preferences.putInt("eq9", audio_eq9);
  preferences.putInt("eq10", audio_eq10);
  preferences.putInt("eq11", audio_eq11);

  // settings screen
  preferences.putUChar("audioseli", audio_input_selected);

  preferences.end();
}

// load prefs from set 0-9
bool load_presets(int setNumber) {
  String ns_config = "audio" + String(setNumber);

  preferences.begin(ns_config.c_str(), true);
  uint8_t version = preferences.getUChar("version", 0);
  if (version != 1) {
    preferences.end();
    return false;
  }

  // main screen
  audio_gain = preferences.getInt("gain", 0);
  audio_line_out = preferences.getInt("lineoutput", 0);
  audio_speaker = preferences.getInt("spkroutput", 0);
  audio_headphone = preferences.getInt("hpoutput", 0);

  // notch screen
  filter_notch_freq0 = preferences.getInt("notchfreq0", 100);
  filter_notch_freq1 = preferences.getInt("notchfreq1", 100);
  filter_notch_freq2 = preferences.getInt("notchfreq2", 100);

  filter_notch_bandwidth0 = preferences.getInt("notchwidth0", 5);
  filter_notch_bandwidth1 = preferences.getInt("notchwidth1", 5);
  filter_notch_bandwidth2 = preferences.getInt("notchwidth2", 5);

  filter_notch_on0 = preferences.getBool("notchon0", false);
  filter_notch_on1 = preferences.getBool("notchon1", false);
  filter_notch_on2 = preferences.getBool("notchon2", false);

  // filter screen
  filter_highpass_freq = preferences.getInt("filterhi", 100);
  filter_lowpass_freq = preferences.getInt("filterlo", 4000);
  filter_bandpass_freq = preferences.getInt("filterbp", 2050);
  filter_bandwidth = preferences.getInt("filterbw", 3900);
  audio_filter_mode = preferences.getUChar("audiosel", 0);

  // peaking screen
  filter_peak_bandpass_freq = preferences.getInt("peakbp", 2050);
  filter_peak_bandwidth = preferences.getInt("peakbw", 5);
  filter_peak_on = preferences.getBool("peakon", false);

  // dynamic screen
  audio_dynamic_denoiser = preferences.getBool("dyndenoiseon", false);
  audio_dynamic_notch = preferences.getBool("dynnotchon", false);
  audio_dynamic_agc = preferences.getBool("dynagcon", false);

  // compressor screen
  audio_compressor_threshold = preferences.getInt("compthreshold", 0);
  audio_compressor_ratio = preferences.getInt("compratio", 0);
  audio_compressor_postgain = preferences.getInt("comppostgain", 0);
  audio_compressor_noisegate = preferences.getInt("compnoisegate", 0);

  // equaliser screen
  audio_eq0 = preferences.getInt("eq0", 0);
  audio_eq1 = preferences.getInt("eq1", 0);
  audio_eq2 = preferences.getInt("eq2", 0);
  audio_eq3 = preferences.getInt("eq3", 0);
  audio_eq4 = preferences.getInt("eq4", 0);
  audio_eq5 = preferences.getInt("eq5", 0);
  audio_eq6 = preferences.getInt("eq6", 0);
  audio_eq7 = preferences.getInt("eq7", 0);
  audio_eq8 = preferences.getInt("eq8", 0);
  audio_eq9 = preferences.getInt("eq9", 0);
  audio_eq10 = preferences.getInt("eq10", 0);
  audio_eq11 = preferences.getInt("eq11", 0);

  // settings screen
  audio_input_selected = preferences.getUChar("audioseli", 0);

  preferences.end();

  apply_presets();

  return true;
}

// set all relevant objects from preferences
void apply_presets(void) {
  // turn off test mode if on
  if (lv_obj_get_state(cb_test_on) & LV_STATE_CHECKED) {
    lv_obj_clear_state(cb_test_on, LV_STATE_CHECKED);
    test_switch_on_off();
  }

  // audio_gain
  update_gain();
  // audio_line_out
  update_output();
  // speaker output
  update_speaker();
  // headphone output
  update_headphone();
  // notch filters
  update_notch_filters();
  // filters
  update_filters();
  // peaking filter
  update_peak_filter();
  // dynamic
  update_dynamic();
  //audio_compressor_threshold
  update_compressor_threshold();
  // audio_compressor_ratio
  update_compressor_ratio();
  // audio_compressor_postgain
  update_compressor_postgain();
  audio_compressor_update();
  // audio_compressor_noisegate
  update_compressor_noisegate();
  audio_noisegate_update();
  // equalisers
  update_eq_all_sliders();
  // input
  update_input();
}

void save_presets_names(void) {
  preferences.begin("pref_names", false);
  for (int i = 0; i < 10; i++) {
    preferences.putString(("presetmsg" + String(i)).c_str(), audio_presets_msg[i]);
  }
  preferences.end();
}

void load_presets_names(void) {
  preferences.begin("pref_names", true);
  for (int i = 0; i < 10; i++) {
    audio_presets_msg[i] = preferences.getString(("presetmsg" + String(i)).c_str(), "Preference");
  }
  preferences.end();
}

void set_limiter(void) {
  const float FS = dsp.FS;
  float rms_dBps     = 100.0f;
  float decay_dBps   = 50.0f;
  float threshold_dB = -8.0f;

  // RMS time constant (coefficient)
  float rmsCoeff = 1.0f - powf(10.0f, -rms_dBps / (10.0f * FS));

  // decay coefficient
  float decayCoeff = powf(2.0f, -decay_dBps);
  float decayComplement = 1.0f - decayCoeff;

  // threshold (convert dBFS → linear)
  float threshLin = powf(10.0f, threshold_dB / 20.0f);

  uint8_t storeData[5];

  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  // write RMS TC
  dsp.floatToFixed(rmsCoeff, storeData);
  dsp.safeload_writeRegister(MOD_LIMITER1_ALG0_RMS_ADDR, storeData, true);

  // write decay
  dsp.floatToFixed(decayCoeff, storeData);
  dsp.safeload_writeRegister(MOD_LIMITER1_ALG0_DECAY_ADDR, storeData, true);

  dsp.floatToFixed(decayComplement, storeData);
  dsp.safeload_writeRegister(MOD_LIMITER1_ALG0_DECAYCOMPLEMENT_ADDR, storeData, true);

  // write threshold
  dsp.floatToFixed(threshLin, storeData);
  dsp.safeload_writeRegister(MOD_LIMITER1_ALG0_THRESHOLD_ADDR, storeData, true);
  xSemaphoreGive(i2cMutex);
}

void ringbuffer_write(const int32_t* data, size_t len) {
  memcpy((void*)&audio_buffer[audio_buffer_write_index], data, len * sizeof(int32_t));
  audio_buffer_write_index = (audio_buffer_write_index + len) % AUDIO_BUFFER_SIZE;
}

bool ringbuffer_read(int32_t* dest, size_t len) {
  size_t available = (audio_buffer_write_index >= audio_buffer_read_index)
                     ? (audio_buffer_write_index - audio_buffer_read_index)
                     : (AUDIO_BUFFER_SIZE - audio_buffer_read_index + audio_buffer_write_index);
  if (available < len) return false;
  memcpy(dest, (void*)&audio_buffer[audio_buffer_read_index], len * sizeof(int32_t));
  audio_buffer_read_index = (audio_buffer_read_index + len) % AUDIO_BUFFER_SIZE;
  return true;
}

// check if data is available in the ringbuffer
size_t ringbuffer_available() {
  return (audio_buffer_write_index >= audio_buffer_read_index)
         ? (audio_buffer_write_index - audio_buffer_read_index)
         : (AUDIO_BUFFER_SIZE - audio_buffer_read_index + audio_buffer_write_index);
}

// background task to read data over I2S to process in the audio task
void i2s_reader_task(void* arg) {
  int32_t temp_buffer[BLOCK_SIZE];
  size_t bytes_read;
  while (true) {
    i2s_read(I2S_NUM, temp_buffer, sizeof(temp_buffer), &bytes_read, portMAX_DELAY);
    if (bytes_read == BLOCK_SIZE * sizeof(int32_t)) {
      ringbuffer_write(temp_buffer, BLOCK_SIZE);
    } else {
      // buffer full
    }
  }
}
