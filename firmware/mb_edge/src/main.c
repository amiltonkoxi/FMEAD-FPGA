/* ============================================================================
 *
 *  FMEAD-FPGA
 *  Multi-Modal Edge Anomaly Detector on FPGA
 *
 *  Target hardware : Digilent Nexys 4 DDR (Artix-7 XC7A100T)
 *  Processor       : MicroBlaze soft-core, bare-metal (no RTOS)
 *  Build           : Xilinx Vitis 2023.x, standalone BSP
 *
 *  Sensors
 *      ADXL362   3-axis accelerometer        (Pmod, SPI)
 *      BME280    temperature / pressure / RH (AXI IIC, I2C, addr 0x76)
 *      XADC      internal FPGA die temperature
 *      PDM MIC   pulse-density microphone    (AXI GPIO sample stream)
 *
 *  Outputs
 *      LEDs, 7-segment display (8 digits, multiplexed)
 *      OLED RGB (optional, Pmod)
 *      ESP32 (Pmod, AT firmware) WiFi access point + HTTP dashboard
 *
 *  Author     : Amilton Koxi
 *  Institution: University of Debrecen, Faculty of Informatics
 *  Supervisor : Prof. Dr. Zoltán Gál
 *
 *  ----------------------------------------------------------------------
 *  Pipeline overview
 *  ----------------------------------------------------------------------
 *
 *      +--------+   +--------+   +--------+   +--------+
 *      |ADXL362 |   |BME280  |   |XADC    |   |PDM MIC |
 *      | (SPI)  |   | (I2C)  |   |(intern)|   |(GPIO)  |
 *      +---+----+   +---+----+   +---+----+   +---+----+
 *          |            |            |            |
 *          v            v            v            v
 *      Motion FSM   Environment   FPGA temp     Acoustic FSM
 *      (NORMAL,     classifier    monitor       (QUIET,
 *       MOVING,                                  CLAP/IMPACT)
 *       VIBRATION,
 *       IMPACT,
 *       FALL)
 *          |            |            |            |
 *          +------------+------+-----+------------+
 *                              |
 *                              v
 *                       Detector fusion
 *                        (g_event, g_risk)
 *                              |
 *              +---------------+---------------+
 *              |               |               |
 *              v               v               v
 *           LEDs +         OLED +          ESP32 HTTP
 *           7-segment      console         (/, /data,
 *                                           /p/<page>)
 *
 *  ----------------------------------------------------------------------
 *  Implementation notes
 *  ----------------------------------------------------------------------
 *
 *  - Integer-only arithmetic. No malloc. No floating point printf.
 *  - HTTP response buffer is bounds-checked (PAGE_BUF_SIZE).
 *  - HTML is pure 7-bit ASCII to keep ESP32 AT byte counts consistent.
 *  - 7-segment scanning is serviced inside UART wait loops so the
 *    display never freezes during ESP32 I/O.
 *
 * ============================================================================
 */

#include "xparameters.h"
#include "xil_printf.h"
#include "xil_types.h"
#include "xil_io.h"
#include "xgpio.h"
#include "xspi.h"
#include "xiic_l.h"
#include "xuartlite_l.h"
#include "xsysmon.h"
#include "sleep.h"
#include "PmodOLEDrgb.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

extern XSpi_Config XSpi_OLEDrgb;
extern u8 rgbOledRgbFont0[];

/* Some BSP headers do not expose this driver helper. */
void OLEDrgb_WriteSPI(PmodOLEDrgb *InstancePtr, u8 *cmd, int nCmd, u8 *data, int nData);

/* ============================================================ */
/* Build options                                                */
/* ============================================================ */

#define UART_VERBOSE            0

/* ============================================================ */
/* Hardware IDs                                                 */
/* ============================================================ */

#define SPI_DEVICE_ID           XPAR_AXI_QUAD_SPI_ADXL362_DEVICE_ID

#if defined(XPAR_AXI_IIC_BME280_BASEADDR)
#define BME280_IIC_BASEADDR     XPAR_AXI_IIC_BME280_BASEADDR
#elif defined(XPAR_AXI_IIC_0_BASEADDR)
#define BME280_IIC_BASEADDR     XPAR_AXI_IIC_0_BASEADDR
#else
#error "No AXI IIC base address found."
#endif

#define ESP_UART_BASEADDR       XPAR_PMODESP32_0_AXI_LITE_UART_BASEADDR
#define ESP_GPIO_BASEADDR       XPAR_PMODESP32_0_AXI_LITE_GPIO_BASEADDR

#define GPIO_CH1_TRI            0x04
#define GPIO_CH2_DATA           0x08
#define GPIO_CH2_TRI            0x0C

#define ESP_SEND_CHUNK          1024

#define SEVENSEG_AN_DEVICE_ID   XPAR_AXI_GPIO_7SEG_AN_DEVICE_ID
#define SEVENSEG_SEG_DEVICE_ID  XPAR_AXI_GPIO_7SEG_SEG_DEVICE_ID

#define XADC_DEVICE_ID          XPAR_XADC_WIZ_0_DEVICE_ID

#define MIC_GPIO_CHANNEL        2
#define MIC_BASELINE_INIT       192

#define PCM_GPIO_DEVICE_ID      XPAR_AXI_GPIO_PCM_DEVICE_ID
#define PCM_SAMPLE_CHANNEL      1
#define PCM_VALID_CHANNEL       2

/* ============================================================ */
/* HTTP response page buffer                                    */
/* All response builders write into this buffer through the     */
/* bounds-checked string helpers (as, au, ai, af2) so that an   */
/* oversized response degrades gracefully instead of corrupting */
/* adjacent global memory.                                      */
/* ============================================================ */

#define PAGE_BUF_SIZE           26000
static char page[PAGE_BUF_SIZE];

/* ============================================================ */
/* ADXL362                                                      */
/* ============================================================ */

#define ADXL362_CMD_WRITE       0x0A
#define ADXL362_CMD_READ        0x0B
#define ADXL362_REG_DEVID_AD    0x00
#define ADXL362_REG_XDATA_L     0x0E
#define ADXL362_REG_SOFT_RST    0x1F
#define ADXL362_REG_FILT_CTL    0x2C
#define ADXL362_REG_PWR_CTL     0x2D
#define ADXL362_SOFT_RST_KEY    0x52
#define ADXL362_MEASURE         0x02
#define ADXL362_100HZ           0x13
#define ADXL362_EXPECTED_ID     0xAD

/* ============================================================ */
/* BME280                                                       */
/* ============================================================ */

#define BME280_ADDR_76          0x76
#define BME280_ADDR_77          0x77
#define BME280_REG_ID           0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_DATA         0xF7
#define BME280_EXPECTED_ID      0x60
#define BME280_RESET_KEY        0xB6

/* ============================================================ */
/* Controls                                                     */
/* ============================================================ */

#define SW_DETECTOR_ENABLE      0x0001
#define SW_LED_BAR_ENABLE       0x0002

#define BTN_CENTER_MASK         0x01
#define BTN_RIGHT_MASK          0x08
#define BTN_LEFT_MASK           0x04

#define SEG_CA                  0x01
#define SEG_CB                  0x02
#define SEG_CC                  0x04
#define SEG_CD                  0x08
#define SEG_CE                  0x10
#define SEG_CF                  0x20
#define SEG_CG                  0x40
#define SEG_CDP                 0x80
#define SEG_DP                  0x80
#define SEG_OFF                 0xFF
#define AN_OFF                  0xFF

/* ============================================================ */
/* Motion detection                                             */
/* Pipeline: raw xyz -> fast EMA -> slow baseline -> score      */
/* and per-frame delta. States: NORMAL, MOVING, VIBRATION,      */
/* IMPACT, FALL CANDIDATE. Hysteresis on MOVING/NORMAL,         */
/* accumulator on VIBRATION, transient hold on IMPACT/FALL.     */
/* ============================================================ */

#define BASELINE_SAMPLES        64
#define COOLDOWN_CYCLES         10
#define EVENT_HOLD_CYCLES       12   /* hold transient events visible */

#define MOTION_FAST_SHIFT       2    /* fast EMA alpha ~ 1/4    */
#define MOTION_REF_SHIFT        7    /* slow baseline ~ 1/128   */
#define MOTION_RECENTER_SHIFT   4    /* quick re-center when still */
#define MOTION_REF_GATE         80   /* baseline only when quiet */
#define MOTION_STILL_JERK       28   /* "still" delta threshold  */
#define MOTION_STILL_FRAMES     18

#define MOTION_ON_MG            260
#define MOTION_OFF_MG           130
#define VIB_FRAME_DELTA         320  /* per-frame shake delta    */
#define VIB_QUIET_DELTA         140
#define VIB_ACC_ON              28
#define VIB_ACC_OFF             10
#define VIB_ACC_INC              8
#define VIB_ACC_DEC              3
#define VIB_ACC_MAX             60

#define IMPACT_DELTA            900
#define FALL_DELTA             1300
#define FALL_SCORE              900
#define ORIENT_CHANGE_TH        500

#define MOTION_ON_FRAMES          3
#define MOTION_OFF_FRAMES         8

#define CALIB_MAX_SPAN          140  /* a bit more tolerant     */
#define CALIB_BOOT_TRIES          4

#define ENV_AMBIENT_WARN_C     3400
#define ENV_AMBIENT_HIGH_C     4000
#define ENV_FPGA_WARN_C        5500
#define ENV_FPGA_HIGH_C        6500
#define ENV_HUM_LOW_CENTI      2500
#define ENV_HUM_HIGH_CENTI     7500
#define ENV_DELTA_WARN_C       1600

/* ============================================================ */
/* Acoustic detection                                           */
/* Two states: QUIET and CLAP / IMPACT. A clap requires four    */
/* conditions simultaneously: peak, active energy, jump above   */
/* noise floor, and crest factor squared. Noise floor adapts    */
/* asymmetrically (fast down, slow up) and is frozen while the  */
/* CLAP hold is active.                                         */
/* ============================================================ */

#define PCM_FRAME_N                 256
#define PCM_HOP_N                   128
#define PCM_WARMUP_FRAMES             6
#define PCM_READY_VALID_SAMPLES      64
#define PCM_VALID_TIMEOUT_CALLS     512

/* Clap / impact triggers (all must be true) */
#define CLAP_PEAK_TH                 96
#define CLAP_ENERGY_TH               12
#define CLAP_JUMP_TH                 10
#define CLAP_CREST2_TH              320

/* Hold + refractory */
#define CLAP_HOLD_FRAMES              4
#define CLAP_REFRACTORY_FRAMES       10

/* Noise floor adaptation (rms2 / peak domain, asymmetric) */
#define NF_FAST_DOWN_SHIFT            3
#define NF_SLOW_UP_SHIFT              7
#define NF_QUIET_LEARN_SHIFT          5

/* Quiet release */
#define QUIET_ENERGY_TH               8
#define QUIET_PEAK_MARGIN            12

#define UI_SCORE_MAX                999

/* ============================================================ */
/* Global devices                                               */
/* ============================================================ */

static XSpi Spi;
static XGpio gpio_leds;
static XGpio gpio_switches;
static XGpio gpio_buttons;
static XGpio gpio_pcm;
static XGpio gpio_7seg_an;
static XGpio gpio_7seg_seg;
static PmodOLEDrgb oled;
static XSysMon xadc_inst;

static u8 g_display_frame[8];
static u8 g_7seg_scan_pos = 0;

#define WEB_PAGE_HOME          0
#define WEB_PAGE_MOTION        1
#define WEB_PAGE_ENVIRONMENT   2
#define WEB_PAGE_ACOUSTIC      3
#define WEB_PAGE_SYSTEM        4

static int g_web_page = WEB_PAGE_HOME;

/* ============================================================ */
/* System state                                                 */
/* ============================================================ */

static int adxl_ok = 0;
static int bme280_ok = 0;
static int bme280_addr = 0;
static int bme280_id = 0;
static int oled_hw_ready = 0;
static int oled_display_ready = 0;
static int xadc_ok = 0;
static int pcm_ok = 0;

/* Motion state */
static int baseline_x = 0, baseline_y = 0, baseline_z = 0;
static int filt_x = 0, filt_y = 0, filt_z = 0;
static int prev_x = 0, prev_y = 0, prev_z = 0;

static int motion_ready = 0;
static int motion_state = 0;
static int vibration_state = 0;
static int motion_on_frames = 0;
static int motion_off_frames = 0;
static int still_frames = 0;
static int vib_acc = 0;
static int prev_buttons = 0;
static int event_hold_cycles = 0;

static int g_x = 0, g_y = 0, g_z = 0;
static int g_score = 0;
static int g_delta = 0;
static int g_alarm_count = 0;
static int g_cooldown = 0;
static int g_max_score = 0;
static int g_switches = 0;
static int g_buttons = 0;

/* Environment */
static int g_temp_centi = 0;
static int g_hum_centi = 0;
static unsigned int g_press_pa = 0;
static int g_fpga_temp_centi = 0;
static int g_env_score = 0;

/* Acoustic features */
static int g_pcm_sample = 0;
static int g_pcm_valid = 0;
static int g_pcm_abs = 0;
static int g_pcm_dc = 0;
static int g_pcm_ac = 0;
static int g_pcm_samples_seen = 0;

static int16_t g_pcm_ring[PCM_FRAME_N];
static unsigned int g_pcm_ring_head = 0;
static unsigned int g_pcm_ring_count = 0;
static unsigned int g_pcm_hop_fill = 0;
static int g_pcm_frame_ready = 0;
static int g_pcm_frame_count = 0;
static int g_pcm_valid_count = 0;
static int g_pcm_no_valid_calls = 0;
static int g_pcm_stream_alive = 0;
static int g_mic_ready = 0;
static int g_pcm_warmup = 0;

static int g_pcm_frame_raw_min = 0;
static int g_pcm_frame_raw_max = 0;
static int g_pcm_frame_raw_mean = 0;
static int g_pcm_frame_rms2 = 0;
static int g_pcm_frame_peak = 0;
static int g_pcm_frame_zcr = 0;
static int g_pcm_frame_crest2 = 0;

static int g_pcm_noise_floor2 = 0;
static int g_pcm_peak_floor = 0;
static int g_pcm_noise_floor = 0;
static int g_pcm_active_energy = 0;
static int g_pcm_active_peak = 0;
static int g_pcm_prev_active_energy = 0;
static int g_pcm_energy_delta = 0;
static int g_pcm_quiet_learn_frames = 0;

/* Acoustic FSM state */
#define ACO_QUIET   0
#define ACO_CLAP    1
static int g_aco_state = ACO_QUIET;
static int g_aco_hold = 0;
static int g_aco_refractory = 0;

/* UI mirrors */
static int g_mic_raw = 0;
static int g_mic_baseline = MIC_BASELINE_INIT;
static int g_mic_diff = 0;
static int g_mic_envelope = 0;
static int g_mic_active = 0;
static int g_mic_score = 0;

/* Event labels */
static const char *g_motion_event = "BOOT";
static const char *g_motion_risk = "LOW";
static const char *g_env_event = "NORMAL";
static const char *g_env_risk = "LOW";
static const char *g_acoustic_event = "QUIET";
static const char *g_acoustic_risk = "LOW";
static const char *g_acoustic_state = "QUIET";
static const char *g_event = "BOOT";
static const char *g_risk = "LOW";

/* ============================================================ */
/* BME280 calibration                                           */
/* ============================================================ */

struct bme280_calib_data {
    u16 dig_T1;
    s16 dig_T2;
    s16 dig_T3;
    u16 dig_P1;
    s16 dig_P2;
    s16 dig_P3;
    s16 dig_P4;
    s16 dig_P5;
    s16 dig_P6;
    s16 dig_P7;
    s16 dig_P8;
    s16 dig_P9;
    u8 dig_H1;
    s16 dig_H2;
    u8 dig_H3;
    s16 dig_H4;
    s16 dig_H5;
    s8 dig_H6;
    s32 t_fine;
};

static struct bme280_calib_data bme_cal;

/* ============================================================ */
/* Bounds-checked string builders for the HTTP page buffer      */
/* ============================================================ */

#define PAGE_SAFE_LIMIT  (PAGE_BUF_SIZE - 64)

static int page_len(char *p)
{
    return (int)(p - page);
}

static int page_full(char *p)
{
    return (page_len(p) >= PAGE_SAFE_LIMIT) ? 1 : 0;
}

static char *as(char *p, const char *s)
{
    while (*s) {
        if (page_full(p)) return p;
        *p++ = *s++;
    }
    return p;
}

static char *au(char *p, unsigned int v)
{
    char t[12];
    int i = 0;

    if (page_full(p)) return p;
    if (!v) {
        *p++ = '0';
        return p;
    }
    while (v) {
        t[i++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while (i > 0) {
        if (page_full(p)) return p;
        *p++ = t[--i];
    }
    return p;
}

static char *ai(char *p, int v)
{
    if (page_full(p)) return p;
    if (v < 0) {
        *p++ = '-';
        v = -v;
    }
    return au(p, (unsigned int)v);
}

static char *af2(char *p, int centi)
{
    int whole;
    int frac;

    if (page_full(p)) return p;
    if (centi < 0) {
        *p++ = '-';
        centi = -centi;
    }
    whole = centi / 100;
    frac = centi % 100;
    p = au(p, (unsigned int)whole);
    if (page_full(p)) return p;
    *p++ = '.';
    if (page_full(p)) return p;
    *p++ = (char)('0' + (frac / 10));
    if (page_full(p)) return p;
    *p++ = (char)('0' + (frac % 10));
    return p;
}

static int iabs(int v)
{
    return (v < 0) ? -v : v;
}

static const char *orientation_name(int x, int y, int z)
{
    int ax = iabs(x);
    int ay = iabs(y);
    int az = iabs(z);

    if (az >= ax && az >= ay) return (z < 0) ? "Face Up" : "Face Down";
    if (ax >= ay && ax >= az) return (x > 0) ? "Right Tilt" : "Left Tilt";
    return (y > 0) ? "Forward Tilt" : "Backward Tilt";
}

/* ============================================================ */
/* XADC                                                         */
/* ============================================================ */

static int xadc_init(void)
{
    XSysMon_Config *cfg;
    int status;

    cfg = XSysMon_LookupConfig(XADC_DEVICE_ID);
    if (cfg == NULL) {
        xil_printf("XADC LOOKUP FAIL\r\n");
        return 0;
    }

    status = XSysMon_CfgInitialize(&xadc_inst, cfg, cfg->BaseAddress);
    if (status != XST_SUCCESS) {
        xil_printf("XADC INIT FAIL\r\n");
        return 0;
    }

    status = XSysMon_SelfTest(&xadc_inst);
    if (status != XST_SUCCESS) {
        xil_printf("XADC SELFTEST FAIL\r\n");
        return 0;
    }

    XSysMon_SetSequencerMode(&xadc_inst, XSM_SEQ_MODE_SAFE);
    xil_printf("XADC OK\r\n");
    return 1;
}

static int xadc_read_fpga_temp_centi(void)
{
    u32 raw32;
    u32 raw;
    int temp_centi;

    if (!xadc_ok) return 0;
    raw32 = XSysMon_GetAdcData(&xadc_inst, XSM_CH_TEMP);
    raw = raw32 >> 4;
    temp_centi = (int)((raw * 50398U) / 4096U) - 27315;
    return temp_centi;
}

/* ============================================================ */
/* PCM frame helpers                                            */
/* ============================================================ */

static int pcm_abs16(int v)
{
    if (v < 0) {
        if (v == (-32768)) return 32768;
        return -v;
    }
    return v;
}

static int pcm_read_sample(void)
{
    u32 raw = XGpio_DiscreteRead(&gpio_pcm, PCM_SAMPLE_CHANNEL);
    return (int)((int16_t)(raw & 0xFFFFU));
}

static int pcm_read_valid(void)
{
    u32 raw = XGpio_DiscreteRead(&gpio_pcm, PCM_VALID_CHANNEL);
    return (int)(raw & 0x1U);
}

static u32 pcm_isqrt_u64(unsigned long long x)
{
    unsigned long long op = x;
    unsigned long long res = 0;
    unsigned long long one = 1ULL << 62;

    while (one > op) one >>= 2;
    while (one != 0ULL) {
        if (op >= res + one) {
            op -= res + one;
            res = (res >> 1) + one;
        } else {
            res >>= 1;
        }
        one >>= 2;
    }
    return (u32)res;
}

static void pcm_compute_frame(void)
{
    int i;
    int sample;
    int prev_sample = 0;
    int have_prev = 0;
    int abs_sample;
    int raw_min = 32767;
    int raw_max = -32768;
    int peak = 0;
    int zcr = 0;
    long sum = 0;
    unsigned long long sumsq = 0ULL;
    unsigned long long crest_num;
    unsigned int idx;

    if (g_pcm_ring_count < PCM_FRAME_N) return;

    for (i = 0; i < PCM_FRAME_N; i++) {
        idx = (g_pcm_ring_head + (unsigned int)i) & (PCM_FRAME_N - 1U);
        sample = (int)g_pcm_ring[idx];

        if (sample < raw_min) raw_min = sample;
        if (sample > raw_max) raw_max = sample;

        sum += sample;
        sumsq += (unsigned long long)((long long)sample * (long long)sample);

        abs_sample = pcm_abs16(sample);
        if (abs_sample > peak) peak = abs_sample;

        if (have_prev) {
            if ((sample >= 0 && prev_sample < 0) ||
                (sample < 0 && prev_sample >= 0)) {
                zcr++;
            }
        } else {
            have_prev = 1;
        }
        prev_sample = sample;
    }

    g_pcm_frame_raw_min = raw_min;
    g_pcm_frame_raw_max = raw_max;
    g_pcm_frame_raw_mean = (int)(sum / PCM_FRAME_N);
    g_pcm_frame_rms2 = (int)(sumsq / PCM_FRAME_N);
    g_pcm_frame_peak = peak;
    g_pcm_frame_zcr = zcr;

    crest_num = (unsigned long long)((long long)peak * (long long)peak) * 256ULL;
    if (g_pcm_frame_rms2 > 0) {
        g_pcm_frame_crest2 = (int)(crest_num / (unsigned long long)g_pcm_frame_rms2);
    } else {
        g_pcm_frame_crest2 = 0;
    }

    g_pcm_frame_ready = 1;
    if (g_pcm_frame_count < 0x7FFFFFFF) g_pcm_frame_count++;
}

static void pcm_update(void)
{
    int sample;
    int ac_sample;
    int abs_sample;

    if (!pcm_ok) {
        g_pcm_sample = 0;
        g_pcm_valid = 0;
        g_pcm_abs = 0;
        g_pcm_samples_seen = 0;
        g_pcm_dc = 0;
        g_pcm_ac = 0;
        g_pcm_ring_head = 0;
        g_pcm_ring_count = 0;
        g_pcm_hop_fill = 0;
        g_pcm_frame_ready = 0;
        g_pcm_no_valid_calls = 0;
        g_pcm_stream_alive = 0;
        g_mic_ready = 0;
        return;
    }

    sample = pcm_read_sample();
    g_pcm_valid = pcm_read_valid();
    g_pcm_sample = sample;
    g_pcm_frame_ready = 0;

    if (!g_pcm_valid) {
        if (g_pcm_no_valid_calls < 0x7FFFFFFF) g_pcm_no_valid_calls++;
        if (g_pcm_no_valid_calls > PCM_VALID_TIMEOUT_CALLS) {
            g_pcm_stream_alive = 0;
            g_mic_ready = 0;
        }
        return;
    }

    g_pcm_no_valid_calls = 0;
    g_pcm_stream_alive = 1;

    if (g_pcm_samples_seen == 0) g_pcm_dc = sample;
    g_pcm_dc += (sample - g_pcm_dc) >> 6;

    ac_sample = sample - g_pcm_dc;
    g_pcm_ac = ac_sample;
    abs_sample = pcm_abs16(ac_sample);
    g_pcm_abs = abs_sample;

    if (g_pcm_valid_count < 0x7FFFFFFF) g_pcm_valid_count++;
    if (g_pcm_valid_count >= PCM_READY_VALID_SAMPLES) g_mic_ready = 1;
    if (g_pcm_samples_seen < 0x7FFFFFFF) g_pcm_samples_seen++;

    g_pcm_ring[g_pcm_ring_head] = (int16_t)ac_sample;
    g_pcm_ring_head = (g_pcm_ring_head + 1U) & (PCM_FRAME_N - 1U);
    if (g_pcm_ring_count < PCM_FRAME_N) g_pcm_ring_count++;

    if (g_pcm_ring_count == PCM_FRAME_N && g_pcm_frame_count == 0) {
        pcm_compute_frame();
        g_pcm_hop_fill = 0;
    } else if (g_pcm_ring_count == PCM_FRAME_N) {
        g_pcm_hop_fill++;
        if (g_pcm_hop_fill >= PCM_HOP_N) {
            pcm_compute_frame();
            g_pcm_hop_fill = 0;
        }
    }
}

/* ============================================================ */
/* Acoustic FSM                                                 */
/* ============================================================ */

static void acoustic_reset(void)
{
    int i;

    g_pcm_ring_head = 0;
    g_pcm_ring_count = 0;
    g_pcm_hop_fill = 0;
    g_pcm_frame_ready = 0;
    g_pcm_frame_count = 0;
    g_pcm_valid_count = 0;
    g_pcm_warmup = 0;
    g_pcm_frame_raw_min = 0;
    g_pcm_frame_raw_max = 0;
    g_pcm_frame_raw_mean = 0;
    g_pcm_frame_rms2 = 0;
    g_pcm_frame_peak = 0;
    g_pcm_frame_zcr = 0;
    g_pcm_frame_crest2 = 0;
    g_pcm_noise_floor2 = 0;
    g_pcm_peak_floor = 0;
    g_pcm_noise_floor = 0;
    g_pcm_active_energy = 0;
    g_pcm_active_peak = 0;
    g_pcm_prev_active_energy = 0;
    g_pcm_energy_delta = 0;
    g_pcm_quiet_learn_frames = 0;
    g_pcm_no_valid_calls = 0;
    g_pcm_stream_alive = 0;
    g_mic_ready = 0;

    g_aco_state = ACO_QUIET;
    g_aco_hold = 0;
    g_aco_refractory = 0;

    g_mic_diff = 0;
    g_mic_envelope = 0;
    g_mic_active = 0;
    g_mic_score = 0;
    g_acoustic_state = "QUIET";
    g_acoustic_event = "QUIET";
    g_acoustic_risk = "LOW";

    for (i = 0; i < PCM_FRAME_N; i++) g_pcm_ring[i] = 0;
}

static void acoustic_update(void)
{
    int clap_candidate;
    int rms;
    int active_energy;
    int active_peak;
    int ui_score;

    pcm_update();

    /* Mirror baseline/raw for UI even when PCM is fine. */
    g_mic_raw = g_pcm_sample;
    g_mic_baseline = g_pcm_dc;

    if (!pcm_ok) {
        acoustic_reset();
        return;
    }

    if (!g_mic_ready || !g_pcm_stream_alive || g_pcm_frame_count == 0) {
        g_acoustic_event = "QUIET";
        g_acoustic_risk = "LOW";
        g_acoustic_state = "QUIET";
        g_aco_state = ACO_QUIET;
        g_aco_hold = 0;
        if (g_mic_score > 0) {
            g_mic_score -= (g_mic_score >> 2) + 1;
            if (g_mic_score < 0) g_mic_score = 0;
        }
        g_pcm_active_energy = 0;
        g_pcm_active_peak = 0;
        g_pcm_energy_delta = 0;
        return;
    }

    /* Decrement refractory every loop call (not only on new frame). */
    if (g_aco_refractory > 0) g_aco_refractory--;

    if (!g_pcm_frame_ready) return;
    g_pcm_frame_ready = 0;

    /* Warmup: aggressive learn of noise floor. */
    if (g_pcm_warmup == 0) {
        g_pcm_noise_floor2 = g_pcm_frame_rms2;
        g_pcm_peak_floor = g_pcm_frame_peak;
    }
    if (g_pcm_warmup < PCM_WARMUP_FRAMES) {
        g_pcm_warmup++;
        g_pcm_noise_floor2 += (g_pcm_frame_rms2 - g_pcm_noise_floor2) >> 2;
        g_pcm_peak_floor   += (g_pcm_frame_peak  - g_pcm_peak_floor)   >> 2;
        if (g_pcm_noise_floor2 < 0) g_pcm_noise_floor2 = 0;
        if (g_pcm_peak_floor   < 0) g_pcm_peak_floor   = 0;
        g_pcm_noise_floor = (int)pcm_isqrt_u64((unsigned long long)g_pcm_noise_floor2);
        g_pcm_active_energy = 0;
        g_pcm_active_peak = 0;
        g_aco_state = ACO_QUIET;
        g_aco_hold = 0;
        g_acoustic_state = "QUIET";
        g_acoustic_event = "QUIET";
        g_acoustic_risk = "LOW";
        g_mic_score = 0;
        return;
    }

    /* ---- Asymmetric noise floor (frozen during clap hold) ---- */
    if (g_aco_hold > 0) {
        /* freeze noise floor while clap is held */
    } else if (g_pcm_frame_rms2 < g_pcm_noise_floor2) {
        g_pcm_noise_floor2 += (g_pcm_frame_rms2 - g_pcm_noise_floor2) >> NF_FAST_DOWN_SHIFT;
    } else {
        g_pcm_noise_floor2 += (g_pcm_frame_rms2 - g_pcm_noise_floor2) >> NF_SLOW_UP_SHIFT;
    }
    if (g_aco_hold == 0) {
        if (g_pcm_frame_peak < g_pcm_peak_floor) {
            g_pcm_peak_floor += (g_pcm_frame_peak - g_pcm_peak_floor) >> NF_FAST_DOWN_SHIFT;
        } else {
            g_pcm_peak_floor += (g_pcm_frame_peak - g_pcm_peak_floor) >> NF_SLOW_UP_SHIFT;
        }
    }
    if (g_pcm_noise_floor2 < 0) g_pcm_noise_floor2 = 0;
    if (g_pcm_peak_floor   < 0) g_pcm_peak_floor   = 0;

    rms = (int)pcm_isqrt_u64((unsigned long long)g_pcm_noise_floor2);
    g_pcm_noise_floor = rms;

    active_energy = (int)pcm_isqrt_u64((unsigned long long)
        ((g_pcm_frame_rms2 > g_pcm_noise_floor2)
            ? (g_pcm_frame_rms2 - g_pcm_noise_floor2) : 0));
    active_peak = g_pcm_frame_peak - g_pcm_peak_floor;
    if (active_peak < 0) active_peak = 0;

    g_pcm_active_energy = active_energy;
    g_pcm_active_peak = active_peak;

    g_pcm_energy_delta = iabs(active_energy - g_pcm_prev_active_energy);
    g_pcm_prev_active_energy = active_energy;

    /* Extra quiet learning: speed up floor when truly silent. */
    if (active_energy <= QUIET_ENERGY_TH &&
        active_peak <= (g_pcm_noise_floor + QUIET_PEAK_MARGIN)) {
        if (g_pcm_quiet_learn_frames < 255) g_pcm_quiet_learn_frames++;
        if (g_pcm_quiet_learn_frames >= 2 && g_aco_hold == 0) {
            g_pcm_noise_floor2 += (g_pcm_frame_rms2 - g_pcm_noise_floor2) >> NF_QUIET_LEARN_SHIFT;
            g_pcm_peak_floor   += (g_pcm_frame_peak  - g_pcm_peak_floor)   >> NF_QUIET_LEARN_SHIFT;
        }
    } else {
        g_pcm_quiet_learn_frames = 0;
    }

    /* ---- Clap / impact detector ---- */
    clap_candidate =
        (active_peak >= CLAP_PEAK_TH) &&
        (active_energy >= CLAP_ENERGY_TH) &&
        (g_pcm_energy_delta >= CLAP_JUMP_TH) &&
        (g_pcm_frame_crest2 >= CLAP_CREST2_TH);

    if (clap_candidate && g_aco_refractory == 0) {
        g_aco_state = ACO_CLAP;
        g_aco_hold = CLAP_HOLD_FRAMES;
        g_aco_refractory = CLAP_REFRACTORY_FRAMES;
    } else if (g_aco_hold > 0) {
        g_aco_hold--;
        if (g_aco_hold == 0) {
            g_aco_state = ACO_QUIET;
        }
    } else {
        g_aco_state = ACO_QUIET;
    }

    if (g_aco_state == ACO_CLAP) {
        g_acoustic_state = "CLAP / IMPACT";
        g_acoustic_event = "CLAP / IMPACT";
        g_acoustic_risk = "HIGH";
    } else {
        g_acoustic_state = "QUIET";
        g_acoustic_event = "QUIET";
        g_acoustic_risk = "LOW";
    }

    /* EMA smoothing for UI mirrors first, so the chart and score */
    /* both reflect the same smoothed signal.                     */
    /* Fast attack (1/2), slow release (1/8).                     */
    {
        static int env_ema = 0;
        static int peak_ema = 0;

        if (active_energy > env_ema)
            env_ema += (active_energy - env_ema) >> 1;
        else
            env_ema += (active_energy - env_ema) >> 3;

        if (active_peak > peak_ema)
            peak_ema += (active_peak - peak_ema) >> 1;
        else
            peak_ema += (active_peak - peak_ema) >> 3;

        g_mic_envelope = env_ema;
        g_mic_diff     = peak_ema;
        g_mic_active   = env_ema;
    }

    /* Score from SMOOTHED values, with lower gain to keep        */
    /* background noise from saturating the bar.                  */
    ui_score = g_mic_envelope * 10;
    if (g_mic_diff * 3 > ui_score) ui_score = g_mic_diff * 3;
    if (ui_score < 0) ui_score = 0;
    if (ui_score > UI_SCORE_MAX) ui_score = UI_SCORE_MAX;

    /* Asymmetric EMA on the score itself for extra stability.    */
    if (ui_score > g_mic_score)
        g_mic_score += (ui_score - g_mic_score) >> 1;
    else
        g_mic_score += (ui_score - g_mic_score) >> 3;

    /* Extra decay in QUIET so the bar settles toward zero.       */
    if (g_aco_state == ACO_QUIET && g_mic_score > 30) {
        g_mic_score -= (g_mic_score >> 4) + 1;
    }
    if (g_mic_score < 0) g_mic_score = 0;
}

/* ============================================================ */
/* Seven-segment display                                        */
/* ============================================================ */

static const u8 glyph_hex[16] = {
    (u8)~(SEG_CA | SEG_CB | SEG_CC | SEG_CD | SEG_CE | SEG_CF),
    (u8)~(SEG_CB | SEG_CC),
    (u8)~(SEG_CA | SEG_CB | SEG_CD | SEG_CE | SEG_CG),
    (u8)~(SEG_CA | SEG_CB | SEG_CC | SEG_CD | SEG_CG),
    (u8)~(SEG_CB | SEG_CC | SEG_CF | SEG_CG),
    (u8)~(SEG_CA | SEG_CC | SEG_CD | SEG_CF | SEG_CG),
    (u8)~(SEG_CA | SEG_CC | SEG_CD | SEG_CE | SEG_CF | SEG_CG),
    (u8)~(SEG_CA | SEG_CB | SEG_CC),
    (u8)~(SEG_CA | SEG_CB | SEG_CC | SEG_CD | SEG_CE | SEG_CF | SEG_CG),
    (u8)~(SEG_CA | SEG_CB | SEG_CC | SEG_CD | SEG_CF | SEG_CG),
    (u8)~(SEG_CA | SEG_CB | SEG_CC | SEG_CE | SEG_CF | SEG_CG),
    (u8)~(SEG_CC | SEG_CD | SEG_CE | SEG_CF | SEG_CG),
    (u8)~(SEG_CA | SEG_CD | SEG_CE | SEG_CF),
    (u8)~(SEG_CB | SEG_CC | SEG_CD | SEG_CE | SEG_CG),
    (u8)~(SEG_CA | SEG_CD | SEG_CE | SEG_CF | SEG_CG),
    (u8)~(SEG_CA | SEG_CE | SEG_CF | SEG_CG)
};

#define GLYPH_BLANK             SEG_OFF
#define GLYPH_A                 ((u8)~(SEG_CA | SEG_CB | SEG_CC | SEG_CE | SEG_CF | SEG_CG))
#define GLYPH_C                 ((u8)~(SEG_CA | SEG_CD | SEG_CE | SEG_CF))
#define GLYPH_L                 ((u8)~(SEG_CD | SEG_CE | SEG_CF))
#define GLYPH_O                 ((u8)~(SEG_CA | SEG_CB | SEG_CC | SEG_CD | SEG_CE | SEG_CF))
#define GLYPH_F                 ((u8)~(SEG_CA | SEG_CE | SEG_CF | SEG_CG))
#define GLYPH_T                 ((u8)~(SEG_CD | SEG_CE | SEG_CF | SEG_CG))
#define GLYPH_P                 ((u8)~(SEG_CA | SEG_CB | SEG_CE | SEG_CF | SEG_CG))
#define GLYPH_M                 ((u8)~(SEG_CA | SEG_CC | SEG_CE))
#define GLYPH_I                 ((u8)~(SEG_CB | SEG_CC))
#define GLYPH_N                 ((u8)~(SEG_CC | SEG_CE | SEG_CG))
#define GLYPH_D                 ((u8)~(SEG_CB | SEG_CC | SEG_CD | SEG_CE | SEG_CG))
#define GLYPH_R                 ((u8)~(SEG_CE | SEG_CG))
#define GLYPH_U                 ((u8)~(SEG_CB | SEG_CC | SEG_CD | SEG_CE | SEG_CF))
#define GLYPH_H                 ((u8)~(SEG_CB | SEG_CC | SEG_CE | SEG_CF | SEG_CG))
#define GLYPH_Y                 ((u8)~(SEG_CB | SEG_CC | SEG_CD | SEG_CF | SEG_CG))
#define GLYPH_E                 ((u8)~(SEG_CA | SEG_CD | SEG_CE | SEG_CF | SEG_CG))
#define GLYPH_S                 ((u8)~(SEG_CA | SEG_CC | SEG_CD | SEG_CF | SEG_CG))
#define GLYPH_B                 ((u8)~(SEG_CC | SEG_CD | SEG_CE | SEG_CF | SEG_CG))
#define GLYPH_DASH              ((u8)~(SEG_CG))

/* Decimal point bit (active-low like the others) */
#define DOT_MASK                ((u8)~(SEG_CDP))

static void sevenseg_frame_clear(void)
{
    int i;
    for (i = 0; i < 8; i++) g_display_frame[i] = GLYPH_BLANK;
}

static void sevenseg_all_off(void)
{
    XGpio_DiscreteWrite(&gpio_7seg_an, 1, AN_OFF);
    XGpio_DiscreteWrite(&gpio_7seg_seg, 1, SEG_OFF);
}

static void sevenseg_set_3digits(int pos, int value)
{
    int v = value;
    if (v < 0) v = -v;
    if (v > 999) v = 999;
    g_display_frame[pos    ] = glyph_hex[(v / 100) % 10];
    g_display_frame[pos + 1] = glyph_hex[(v / 10) % 10];
    g_display_frame[pos + 2] = glyph_hex[v % 10];
}

static void sevenseg_set_4digits(int pos, int value)
{
    int v = value;
    if (v < 0) v = -v;
    if (v > 9999) v = 9999;
    g_display_frame[pos    ] = glyph_hex[(v / 1000) % 10];
    g_display_frame[pos + 1] = glyph_hex[(v / 100) % 10];
    g_display_frame[pos + 2] = glyph_hex[(v / 10) % 10];
    g_display_frame[pos + 3] = glyph_hex[v % 10];
}

/* Write fixed-point "XX.X" starting at pos, value is centi-units (e.g. 2345 -> 23.4) */
static void sevenseg_set_temp(int pos, int centi)
{
    int abs_v = (centi < 0) ? -centi : centi;
    int tenths = abs_v / 10;          /* X.X => keep 1 decimal */
    int whole  = tenths / 10;
    int dec    = tenths % 10;

    if (whole > 99) whole = 99;
    g_display_frame[pos    ] = glyph_hex[(whole / 10) % 10];
    g_display_frame[pos + 1] = (u8)(glyph_hex[whole % 10] & DOT_MASK);  /* dot on */
    g_display_frame[pos + 2] = glyph_hex[dec];
    g_display_frame[pos + 3] = GLYPH_C;
}

/* Map current motion event to a 3-character mnemonic */
static void sevenseg_write_event_label(int pos)
{
    char c = g_event[0];
    u8 a, b, d;

    if      (c == 'F') { a = GLYPH_F; b = GLYPH_A; d = GLYPH_L; }   /* FAL = FALL */
    else if (c == 'I') { a = GLYPH_P; b = GLYPH_A; d = GLYPH_C; }   /* PAC = IMPACT */
    else if (c == 'V') { a = GLYPH_U; b = GLYPH_I; d = GLYPH_B; }   /* UIb = VIBR */
    else if (c == 'M') { a = GLYPH_M; b = GLYPH_O; d = GLYPH_U; }   /* MOU = MOVE */
    else               { a = GLYPH_N; b = GLYPH_O; d = GLYPH_R; }   /* NOR = NORM */

    g_display_frame[pos    ] = a;
    g_display_frame[pos + 1] = b;
    g_display_frame[pos + 2] = d;
}

static void sevenseg_set_off_frame(void)
{
    sevenseg_frame_clear();
    g_display_frame[2] = GLYPH_O;
    g_display_frame[3] = GLYPH_F;
    g_display_frame[4] = GLYPH_F;
}

static void sevenseg_update_page(void)
{
    if (!(g_switches & SW_DETECTOR_ENABLE)) {
        sevenseg_set_off_frame();
        return;
    }

    sevenseg_frame_clear();

    if (g_web_page == WEB_PAGE_HOME) {
        /* Layout: [H P] [E V T] [S S S]                          */
        /* HP marks "Home Page", followed by motion event mnemonic */
        /* and a 3-digit score on the right.                      */
        g_display_frame[0] = GLYPH_H;
        g_display_frame[1] = GLYPH_P;
        sevenseg_write_event_label(2);
        sevenseg_set_3digits(5, g_score);

    } else if (g_web_page == WEB_PAGE_MOTION) {
        /* Layout: M O T  [-] [S S S S]  */
        g_display_frame[0] = GLYPH_M;
        g_display_frame[1] = GLYPH_O;
        g_display_frame[2] = GLYPH_T;
        g_display_frame[3] = GLYPH_DASH;
        sevenseg_set_4digits(4, g_score);

    } else if (g_web_page == WEB_PAGE_ENVIRONMENT) {
        /* Layout: [XX.X C] [HH%] -> show temperature in left half */
        sevenseg_set_temp(0, g_temp_centi);
        /* Right half: humidity as 3-digit integer percent */
        sevenseg_set_3digits(5, g_hum_centi / 100);

    } else if (g_web_page == WEB_PAGE_ACOUSTIC) {
        /* Layout: [Q U I E] or [C L A P] then 4-digit score */
        if (g_aco_state == ACO_CLAP) {
            g_display_frame[0] = GLYPH_C;
            g_display_frame[1] = GLYPH_L;
            g_display_frame[2] = GLYPH_A;
            g_display_frame[3] = GLYPH_P;
        } else {
            /* 'q' glyph: a+b+c+f+g (lowercase-q shape with bottom open) */
            g_display_frame[0] = (u8)~(SEG_CA | SEG_CB | SEG_CC | SEG_CF | SEG_CG);
            g_display_frame[1] = GLYPH_U;
            g_display_frame[2] = GLYPH_I;
            g_display_frame[3] = GLYPH_E;
        }
        sevenseg_set_4digits(4, g_mic_score);

    } else { /* SYSTEM page */
        /* Layout: A L R  [-] [N N N N]  alarm count */
        g_display_frame[0] = GLYPH_A;
        g_display_frame[1] = GLYPH_L;
        g_display_frame[2] = GLYPH_R;
        g_display_frame[3] = GLYPH_DASH;
        sevenseg_set_4digits(4, g_alarm_count);
    }
}

static void sevenseg_scan_task(void)
{
    u8 physical_position;
    u8 an;

    physical_position = (u8)(7U - g_7seg_scan_pos);
    XGpio_DiscreteWrite(&gpio_7seg_an, 1, AN_OFF);
    XGpio_DiscreteWrite(&gpio_7seg_seg, 1, g_display_frame[g_7seg_scan_pos]);
    an = (u8)~(1U << physical_position);
    XGpio_DiscreteWrite(&gpio_7seg_an, 1, an);

    g_7seg_scan_pos++;
    if (g_7seg_scan_pos >= 8U) g_7seg_scan_pos = 0;
}

/* ============================================================ */
/* GPIO init                                                    */
/* ============================================================ */

static int gpio_init_all(void)
{
    int status;

    status = XGpio_Initialize(&gpio_leds, XPAR_AXI_GPIO_LEDS_DEVICE_ID);
    if (status != XST_SUCCESS) { xil_printf("GPIO LED FAIL\r\n"); return XST_FAILURE; }

    status = XGpio_Initialize(&gpio_switches, XPAR_AXI_GPIO_SWITCHES_DEVICE_ID);
    if (status != XST_SUCCESS) { xil_printf("GPIO SWITCH FAIL\r\n"); return XST_FAILURE; }

    status = XGpio_Initialize(&gpio_buttons, XPAR_AXI_GPIO_BUTTONS_DEVICE_ID);
    if (status != XST_SUCCESS) { xil_printf("GPIO BUTTON FAIL\r\n"); return XST_FAILURE; }

    status = XGpio_Initialize(&gpio_pcm, PCM_GPIO_DEVICE_ID);
    if (status != XST_SUCCESS) {
        pcm_ok = 0;
        xil_printf("GPIO PCM FAIL, continuing without PCM\r\n");
    } else {
        pcm_ok = 1;
        XGpio_SetDataDirection(&gpio_pcm, PCM_SAMPLE_CHANNEL, 0xFFFFFFFF);
        XGpio_SetDataDirection(&gpio_pcm, PCM_VALID_CHANNEL, 0xFFFFFFFF);
        xil_printf("GPIO PCM OK\r\n");
    }

    status = XGpio_Initialize(&gpio_7seg_an, SEVENSEG_AN_DEVICE_ID);
    if (status != XST_SUCCESS) { xil_printf("7SEG AN FAIL\r\n"); return XST_FAILURE; }

    status = XGpio_Initialize(&gpio_7seg_seg, SEVENSEG_SEG_DEVICE_ID);
    if (status != XST_SUCCESS) { xil_printf("7SEG SEG FAIL\r\n"); return XST_FAILURE; }

    XGpio_SetDataDirection(&gpio_leds, 1, 0x00000000);
    XGpio_SetDataDirection(&gpio_switches, 1, 0xFFFFFFFF);
    XGpio_SetDataDirection(&gpio_switches, 2, 0xFFFFFFFF);
    XGpio_SetDataDirection(&gpio_buttons, 1, 0xFFFFFFFF);
    XGpio_SetDataDirection(&gpio_7seg_an, 1, 0x00000000);
    XGpio_SetDataDirection(&gpio_7seg_seg, 1, 0x00000000);

    sevenseg_all_off();
    sevenseg_set_off_frame();

    xil_printf("GPIO OK\r\n");
    xil_printf("7SEG OK\r\n");
    return XST_SUCCESS;
}

/* ============================================================ */
/* ADXL362 SPI                                                  */
/* ============================================================ */

static void spi_xfer(u8 *tx, u8 *rx, int len)
{
    XSpi_SetSlaveSelect(&Spi, 0x01);
    XSpi_Transfer(&Spi, tx, rx, len);
}

static void spi_wr(u8 reg, u8 val)
{
    u8 tx[3] = { ADXL362_CMD_WRITE, reg, val };
    u8 rx[3] = { 0, 0, 0 };
    spi_xfer(tx, rx, 3);
}

static u8 spi_rd(u8 reg)
{
    u8 tx[3] = { ADXL362_CMD_READ, reg, 0 };
    u8 rx[3] = { 0, 0, 0 };
    spi_xfer(tx, rx, 3);
    return rx[2];
}

static void read_xyz(int *x, int *y, int *z)
{
    u8 tx[8] = { ADXL362_CMD_READ, ADXL362_REG_XDATA_L, 0, 0, 0, 0, 0, 0 };
    u8 rx[8] = { 0 };
    int xv, yv, zv;

    spi_xfer(tx, rx, 8);

    xv = (int)(((unsigned int)rx[3] << 8) | rx[2]);
    yv = (int)(((unsigned int)rx[5] << 8) | rx[4]);
    zv = (int)(((unsigned int)rx[7] << 8) | rx[6]);

    if (xv & 0x0800) xv |= 0xFFFFF000;
    if (yv & 0x0800) yv |= 0xFFFFF000;
    if (zv & 0x0800) zv |= 0xFFFFF000;

    *x = xv; *y = yv; *z = zv;
}

static int adxl_init(void)
{
    XSpi_Config *cfg;
    u8 id;

    cfg = XSpi_LookupConfig(SPI_DEVICE_ID);
    if (!cfg) { xil_printf("ADXL SPI CONFIG FAIL\r\n"); return 0; }

    if (XSpi_CfgInitialize(&Spi, cfg, cfg->BaseAddress) != XST_SUCCESS) {
        xil_printf("ADXL SPI INIT FAIL\r\n");
        return 0;
    }

    XSpi_SetOptions(&Spi, XSP_MASTER_OPTION | XSP_MANUAL_SSELECT_OPTION);
    XSpi_Start(&Spi);
    XSpi_IntrGlobalDisable(&Spi);
    XSpi_SetSlaveSelect(&Spi, 0x01);

    spi_wr(ADXL362_REG_SOFT_RST, ADXL362_SOFT_RST_KEY);
    usleep(500000);

    id = spi_rd(ADXL362_REG_DEVID_AD);
    if (id != ADXL362_EXPECTED_ID) {
        xil_printf("ADXL FAIL ID=0x%02X\r\n", id);
        return 0;
    }

    spi_wr(ADXL362_REG_FILT_CTL, ADXL362_100HZ);
    spi_wr(ADXL362_REG_PWR_CTL, ADXL362_MEASURE);
    usleep(100000);

    xil_printf("ADXL OK ID=0x%02X\r\n", id);
    return 1;
}

/* ============================================================ */
/* Motion FSM                                                   */
/* ============================================================ */

static void motion_filter_reset(int x, int y, int z)
{
    baseline_x = x; baseline_y = y; baseline_z = z;
    filt_x = x;     filt_y = y;     filt_z = z;
    prev_x = x;     prev_y = y;     prev_z = z;

    motion_ready = 1;
    motion_state = 0;
    vibration_state = 0;
    motion_on_frames = 0;
    motion_off_frames = 0;
    vib_acc = 0;
    still_frames = 0;
    event_hold_cycles = 0;
}

static int calibrate_baseline_window(int reject_if_moving)
{
    int x, y, z;
    long sx = 0, sy = 0, sz = 0;
    int min_x = 32767, min_y = 32767, min_z = 32767;
    int max_x = -32768, max_y = -32768, max_z = -32768;
    int span;
    int i;

    for (i = 0; i < BASELINE_SAMPLES; i++) {
        read_xyz(&x, &y, &z);

        sx += x; sy += y; sz += z;
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
        if (z < min_z) min_z = z;
        if (z > max_z) max_z = z;

        usleep(10000);
    }

    span = (max_x - min_x) + (max_y - min_y) + (max_z - min_z);
    if (reject_if_moving && span > CALIB_MAX_SPAN) {
        xil_printf("Calibration rejected: board not still (span=%d)\r\n", span);
        return 0;
    }

    baseline_x = (int)(sx / BASELINE_SAMPLES);
    baseline_y = (int)(sy / BASELINE_SAMPLES);
    baseline_z = (int)(sz / BASELINE_SAMPLES);

    motion_filter_reset(baseline_x, baseline_y, baseline_z);

    xil_printf("Baseline OK X=%d Y=%d Z=%d span=%d\r\n",
               baseline_x, baseline_y, baseline_z, span);
    return 1;
}

static int calibrate_baseline_quality(void)
{
    xil_printf("Baseline calibration: keep board still...\r\n");
    return calibrate_baseline_window(1);
}

static void calibrate_baseline(void)
{
    int attempt;

    xil_printf("Baseline calibration (boot)...\r\n");
    for (attempt = 0; attempt < CALIB_BOOT_TRIES; attempt++) {
        if (calibrate_baseline_window(1)) return;
        usleep(30000);
    }
    xil_printf("Boot calibration fallback (forced)\r\n");
    calibrate_baseline_window(0);
}

static void motion_features_update(int x, int y, int z,
                                   int *score_out,
                                   int *delta_out,
                                   int *orient_change_out)
{
    int dx, dy, dz;
    int jerk;
    int score;

    if (!motion_ready) motion_filter_reset(x, y, z);

    /* Fast EMA */
    filt_x += (x - filt_x) >> MOTION_FAST_SHIFT;
    filt_y += (y - filt_y) >> MOTION_FAST_SHIFT;
    filt_z += (z - filt_z) >> MOTION_FAST_SHIFT;

    /* Provisional score for baseline gating */
    dx = filt_x - baseline_x;
    dy = filt_y - baseline_y;
    dz = filt_z - baseline_z;
    score = iabs(dx) + iabs(dy) + iabs(dz);

    /* Jerk proxy (delta) */
    jerk = iabs(filt_x - prev_x) + iabs(filt_y - prev_y) + iabs(filt_z - prev_z);
    *delta_out = jerk;

    /* Still frame counter */
    if (jerk < MOTION_STILL_JERK) {
        if (still_frames < 30000) still_frames++;
    } else {
        still_frames = 0;
    }

    /* Baseline tracking ONLY when quiet */
    if (score < MOTION_REF_GATE && jerk < MOTION_STILL_JERK) {
        baseline_x += (filt_x - baseline_x) >> MOTION_REF_SHIFT;
        baseline_y += (filt_y - baseline_y) >> MOTION_REF_SHIFT;
        baseline_z += (filt_z - baseline_z) >> MOTION_REF_SHIFT;
    } else if (still_frames >= MOTION_STILL_FRAMES && jerk < MOTION_STILL_JERK) {
        baseline_x += (filt_x - baseline_x) >> MOTION_RECENTER_SHIFT;
        baseline_y += (filt_y - baseline_y) >> MOTION_RECENTER_SHIFT;
        baseline_z += (filt_z - baseline_z) >> MOTION_RECENTER_SHIFT;
    }

    /* Recompute score from updated baseline */
    dx = filt_x - baseline_x;
    dy = filt_y - baseline_y;
    dz = filt_z - baseline_z;
    *score_out = iabs(dx) + iabs(dy) + iabs(dz);

    *orient_change_out = (iabs(dz) > ORIENT_CHANGE_TH) ? 1 : 0;

    prev_x = filt_x; prev_y = filt_y; prev_z = filt_z;
}

static const char *classify_event_robust(int score, int delta, int orient_change)
{
    /* IMPACT / FALL: sharp delta wins immediately (transient). */
    if (delta >= IMPACT_DELTA) {
        motion_state = 1;
        motion_on_frames = 0;
        motion_off_frames = 0;

        if (delta >= FALL_DELTA && orient_change && score >= FALL_SCORE) {
            vibration_state = 0;
            vib_acc = 0;
            return "FALL CANDIDATE";
        }
        return "IMPACT";
    }

    /* Vibration accumulator: repeated moderate shakes. */
    if (delta >= VIB_FRAME_DELTA) {
        vib_acc += VIB_ACC_INC;
        if (vib_acc > VIB_ACC_MAX) vib_acc = VIB_ACC_MAX;
    } else if (delta <= VIB_QUIET_DELTA) {
        vib_acc -= VIB_ACC_DEC;
        if (vib_acc < 0) vib_acc = 0;
    }

    if (!vibration_state && vib_acc >= VIB_ACC_ON) {
        vibration_state = 1;
    } else if (vibration_state && vib_acc <= VIB_ACC_OFF) {
        vibration_state = 0;
    }

    /* MOVING / NORMAL FSM with hysteresis. */
    if (!motion_state) {
        if (delta >= MOTION_ON_MG ||
            (score >= MOTION_ON_MG && still_frames < MOTION_STILL_FRAMES)) {
            if (motion_on_frames < 255) motion_on_frames++;
            if (motion_on_frames >= MOTION_ON_FRAMES) {
                motion_state = 1;
                motion_off_frames = 0;
            }
        } else {
            motion_on_frames = 0;
        }
    } else {
        if (score <= MOTION_OFF_MG && delta <= MOTION_OFF_MG) {
            if (motion_off_frames < 255) motion_off_frames++;
            if (motion_off_frames >= MOTION_OFF_FRAMES) {
                motion_state = 0;
                vibration_state = 0;
                motion_on_frames = 0;
                vib_acc = 0;
            }
        } else {
            motion_off_frames = 0;
        }
    }

    if (motion_state) {
        if (vibration_state) return "VIBRATION";
        return "MOVING";
    }

    /* Hard guard against drift while on table. */
    if (still_frames >= MOTION_STILL_FRAMES &&
        delta < MOTION_OFF_MG && score < MOTION_ON_MG) {
        vib_acc = 0;
        vibration_state = 0;
    }

    return "NORMAL";
}

/* ============================================================ */
/* BME280 I2C driver                                            */
/* ============================================================ */

static u16 bme_u16(u8 lsb, u8 msb) { return (u16)(((u16)msb << 8) | lsb); }
static s16 bme_s16(u8 lsb, u8 msb) { return (s16)bme_u16(lsb, msb); }

static int bme280_write_reg(u8 reg, u8 val)
{
    u8 buf[2] = { reg, val };
    int sent = XIic_Send(BME280_IIC_BASEADDR, (u8)bme280_addr, buf, 2, XIIC_STOP);
    return (sent == 2) ? 1 : 0;
}

static int bme280_read_regs_at(u8 addr, u8 reg, u8 *buf, int len)
{
    int sent;
    int received;

    sent = XIic_Send(BME280_IIC_BASEADDR, addr, &reg, 1, XIIC_REPEATED_START);
    if (sent != 1) return 0;

    received = XIic_Recv(BME280_IIC_BASEADDR, addr, buf, len, XIIC_STOP);
    return (received == len) ? 1 : 0;
}

static int bme280_read_regs(u8 reg, u8 *buf, int len)
{
    if (!bme280_ok) return 0;
    return bme280_read_regs_at((u8)bme280_addr, reg, buf, len);
}

static int bme280_probe_addr(u8 addr, u8 *id)
{
    return bme280_read_regs_at(addr, BME280_REG_ID, id, 1);
}

static int bme280_read_calibration(void)
{
    u8 c1[26];
    u8 c2[7];

    if (!bme280_read_regs(0x88, c1, 26)) {
        xil_printf("BME280 calib block 1 FAIL\r\n");
        return 0;
    }
    if (!bme280_read_regs(0xE1, c2, 7)) {
        xil_printf("BME280 calib block 2 FAIL\r\n");
        return 0;
    }

    bme_cal.dig_T1 = bme_u16(c1[0], c1[1]);
    bme_cal.dig_T2 = bme_s16(c1[2], c1[3]);
    bme_cal.dig_T3 = bme_s16(c1[4], c1[5]);
    bme_cal.dig_P1 = bme_u16(c1[6], c1[7]);
    bme_cal.dig_P2 = bme_s16(c1[8], c1[9]);
    bme_cal.dig_P3 = bme_s16(c1[10], c1[11]);
    bme_cal.dig_P4 = bme_s16(c1[12], c1[13]);
    bme_cal.dig_P5 = bme_s16(c1[14], c1[15]);
    bme_cal.dig_P6 = bme_s16(c1[16], c1[17]);
    bme_cal.dig_P7 = bme_s16(c1[18], c1[19]);
    bme_cal.dig_P8 = bme_s16(c1[20], c1[21]);
    bme_cal.dig_P9 = bme_s16(c1[22], c1[23]);
    bme_cal.dig_H1 = c1[25];
    bme_cal.dig_H2 = bme_s16(c2[0], c2[1]);
    bme_cal.dig_H3 = c2[2];
    bme_cal.dig_H4 = (s16)(((s16)c2[3] << 4) | (c2[4] & 0x0F));
    bme_cal.dig_H5 = (s16)(((s16)c2[5] << 4) | (c2[4] >> 4));
    bme_cal.dig_H6 = (s8)c2[6];

    xil_printf("BME280 calibration OK\r\n");
    return 1;
}

static int bme280_compensate_temp(s32 adc_T)
{
    s32 var1, var2, temp;
    var1 = ((((adc_T >> 3) - ((s32)bme_cal.dig_T1 << 1))) * ((s32)bme_cal.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((s32)bme_cal.dig_T1)) * ((adc_T >> 4) - ((s32)bme_cal.dig_T1))) >> 12) * ((s32)bme_cal.dig_T3)) >> 14;
    bme_cal.t_fine = var1 + var2;
    temp = (bme_cal.t_fine * 5 + 128) >> 8;
    return (int)temp;
}

static unsigned int bme280_compensate_press(s32 adc_P)
{
    s64 var1, var2, p;

    var1 = ((s64)bme_cal.t_fine) - 128000;
    var2 = var1 * var1 * (s64)bme_cal.dig_P6;
    var2 = var2 + ((var1 * (s64)bme_cal.dig_P5) << 17);
    var2 = var2 + (((s64)bme_cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (s64)bme_cal.dig_P3) >> 8) + ((var1 * (s64)bme_cal.dig_P2) << 12);
    var1 = (((((s64)1) << 47) + var1)) * ((s64)bme_cal.dig_P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((s64)bme_cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((s64)bme_cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((s64)bme_cal.dig_P7) << 4);
    return (unsigned int)(p / 256);
}

static int bme280_compensate_hum(s32 adc_H)
{
    s32 v_x1;
    unsigned int h_x1024;

    v_x1 = bme_cal.t_fine - 76800;
    v_x1 = (((((adc_H << 14) - (((s32)bme_cal.dig_H4) << 20) - (((s32)bme_cal.dig_H5) * v_x1)) + 16384) >> 15) *
            (((((((v_x1 * ((s32)bme_cal.dig_H6)) >> 10) * (((v_x1 * ((s32)bme_cal.dig_H3)) >> 11) + 32768)) >> 10) +
               2097152) * ((s32)bme_cal.dig_H2) + 8192) >> 14));
    v_x1 = v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((s32)bme_cal.dig_H1)) >> 4);
    if (v_x1 < 0) v_x1 = 0;
    if (v_x1 > 419430400) v_x1 = 419430400;
    h_x1024 = (unsigned int)(v_x1 >> 12);
    return (int)((h_x1024 * 100U) / 1024U);
}

static int bme280_read_measurement(void)
{
    u8 d[8];
    s32 adc_P, adc_T, adc_H;

    if (!bme280_ok) return 0;
    if (!bme280_read_regs(BME280_REG_DATA, d, 8)) return 0;

    adc_P = (((s32)d[0] << 12) | ((s32)d[1] << 4) | ((s32)d[2] >> 4));
    adc_T = (((s32)d[3] << 12) | ((s32)d[4] << 4) | ((s32)d[5] >> 4));
    adc_H = (((s32)d[6] << 8)  |  ((s32)d[7]));

    g_temp_centi = bme280_compensate_temp(adc_T);
    g_press_pa   = bme280_compensate_press(adc_P);
    g_hum_centi  = bme280_compensate_hum(adc_H);
    return 1;
}

static int bme280_init(void)
{
    u8 id;

    xil_printf("BME280 I2C validation...\r\n");
    bme280_ok = 0;
    bme280_addr = 0;
    bme280_id = 0;

    if (bme280_probe_addr(BME280_ADDR_76, &id)) {
        xil_printf("BME280 probe 0x76 ID=0x%02X\r\n", id);
        if (id == BME280_EXPECTED_ID) {
            bme280_addr = BME280_ADDR_76;
            bme280_id = id;
            bme280_ok = 1;
        }
    }
    if (!bme280_ok && bme280_probe_addr(BME280_ADDR_77, &id)) {
        xil_printf("BME280 probe 0x77 ID=0x%02X\r\n", id);
        if (id == BME280_EXPECTED_ID) {
            bme280_addr = BME280_ADDR_77;
            bme280_id = id;
            bme280_ok = 1;
        }
    }

    if (!bme280_ok) {
        xil_printf("BME280 NOT DETECTED\r\n");
        return 0;
    }

    xil_printf("BME280 OK ADDR=0x%02X ID=0x%02X\r\n", bme280_addr, bme280_id);
    bme280_write_reg(BME280_REG_RESET, BME280_RESET_KEY);
    usleep(300000);

    if (!bme280_read_calibration()) { bme280_ok = 0; return 0; }

    bme280_write_reg(BME280_REG_CTRL_HUM, 0x01);
    bme280_write_reg(BME280_REG_CONFIG, 0xA0);
    bme280_write_reg(BME280_REG_CTRL_MEAS, 0x27);
    usleep(200000);

    if (!bme280_read_measurement()) {
        xil_printf("BME280 first measurement FAIL\r\n");
        bme280_ok = 0;
        return 0;
    }

    xil_printf("BME280 init OK\r\n");
    return 1;
}

/* ============================================================ */
/* OLED compact numeric dashboard                               */
/* Safe version for PmodOLEDrgb on JB.                           */
/* No OLEDrgb_begin, no PutString, no PutChar.                   */
/* ============================================================ */

#define OLED_CMD_CLEARWINDOW             0x25
#define OLED_CMD_DEACTIVESCROLLING       0x2E
#define OLED_CMD_SETCONTRASTA            0x81
#define OLED_CMD_SETCONTRASTB            0x82
#define OLED_CMD_SETCONTRASTC            0x83
#define OLED_CMD_MASTERCURRENTCONTROL    0x87
#define OLED_CMD_SETPRECHARGESPEEDA      0x8A
#define OLED_CMD_SETPRECHARGESPEEDB      0x8B
#define OLED_CMD_SETPRECHARGESPEEDC      0x8C
#define OLED_CMD_SETREMAP                0xA0
#define OLED_CMD_SETDISPLAYSTARTLINE     0xA1
#define OLED_CMD_SETDISPLAYOFFSET        0xA2
#define OLED_CMD_NORMALDISPLAY           0xA4
#define OLED_CMD_SETMULTIPLEXRATIO       0xA8
#define OLED_CMD_SETMASTERCONFIGURE      0xAD
#define OLED_CMD_DISPLAYOFF              0xAE
#define OLED_CMD_DISPLAYON               0xAF
#define OLED_CMD_POWERSAVEMODE           0xB0
#define OLED_CMD_PHASEPERIODADJUSTMENT   0xB1
#define OLED_CMD_DISPLAYCLOCKDIV         0xB3
#define OLED_CMD_SETPRECHARGEVOLTAGE     0xBB
#define OLED_CMD_SETVVOLTAGE             0xBE

/* Called from the idle HTTP wait loop, roughly once per millisecond. */
#define OLED_UPDATE_TICKS                500U

static u16 oled_black;
static u16 oled_green;
static u16 oled_yellow;
static u16 oled_red;
static u16 oled_cyan;
static u16 oled_gray;

static int oled_clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static int oled_contains(const char *text, const char *needle)
{
    const char *a;
    const char *b;

    if (text == NULL || needle == NULL) return 0;
    if (*needle == '\0') return 1;

    while (*text) {
        a = text;
        b = needle;

        while (*a && *b && *a == *b) {
            a++;
            b++;
        }
        if (*b == '\0') return 1;
        text++;
    }

    return 0;
}

static void oled_cmd1(u8 c0)
{
    u8 cmd[1];
    cmd[0] = c0;
    OLEDrgb_WriteSPI(&oled, cmd, 1, NULL, 0);
}

static void oled_cmd2(u8 c0, u8 c1)
{
    u8 cmd[2];
    cmd[0] = c0;
    cmd[1] = c1;
    OLEDrgb_WriteSPI(&oled, cmd, 2, NULL, 0);
}

static void oled_clear_window_fast(void)
{
    u8 cmd[5];

    cmd[0] = OLED_CMD_CLEARWINDOW;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = 95;
    cmd[4] = 63;

    OLEDrgb_WriteSPI(&oled, cmd, 5, NULL, 0);
}

static void oled_make_colors(void)
{
    oled_black  = OLEDrgb_BuildRGB(0, 0, 0);
    oled_green  = OLEDrgb_BuildRGB(0, 220, 80);
    oled_yellow = OLEDrgb_BuildRGB(255, 180, 0);
    oled_red    = OLEDrgb_BuildRGB(255, 0, 0);
    oled_cyan   = OLEDrgb_BuildRGB(0, 200, 255);
    oled_gray   = OLEDrgb_BuildRGB(70, 70, 70);
}

static const char *oled_event_short_name(void)
{
    if (!(g_switches & SW_DETECTOR_ENABLE)) return "OFF";

    if (oled_contains(g_event, "CLAP")) return "CLAP";
    if (oled_contains(g_event, "IMPACT")) return "IMPACT";
    if (oled_contains(g_event, "FALL")) return "IMPACT";
    if (oled_contains(g_event, "VIBRATION")) return "VIB";
    if (oled_contains(g_event, "MOVING")) return "MOVING";
    if (oled_contains(g_event, "HOT")) return "HOT";
    if (oled_contains(g_event, "WARM")) return "HOT";
    if (oled_contains(g_event, "THERMAL")) return "HOT";
    if (oled_contains(g_event, "HUMIDITY")) return "HOT";
    if (oled_contains(g_event, "OFFLINE")) return "OFF";
    if (oled_contains(g_event, "DISABLED")) return "OFF";

    return "NORMAL";
}

static const char *oled_risk_short_name(void)
{
    if (!(g_switches & SW_DETECTOR_ENABLE)) return "OFF";
    if (g_risk == NULL) return "LOW";

    if (g_risk[0] == 'C') return "CRIT";
    if (g_risk[0] == 'H') return "HIGH";
    if (g_risk[0] == 'M') return "MED";

    return "LOW";
}

static u16 oled_color_from_status(const char *event, const char *risk)
{
    if (!(g_switches & SW_DETECTOR_ENABLE)) return oled_gray;

    if (event != NULL) {
        if (oled_contains(event, "CLAP")) return oled_red;
        if (oled_contains(event, "IMPACT")) return oled_red;
        if (oled_contains(event, "FALL")) return oled_red;
        if (oled_contains(event, "HOT")) return oled_red;
        if (oled_contains(event, "VIBRATION")) return oled_yellow;
        if (oled_contains(event, "WARM")) return oled_yellow;
        if (oled_contains(event, "THERMAL")) return oled_yellow;
        if (oled_contains(event, "HUMIDITY")) return oled_yellow;
        if (oled_contains(event, "MOVING")) return oled_cyan;
    }

    if (risk != NULL) {
        if (risk[0] == 'C') return oled_red;
        if (risk[0] == 'H') return oled_red;
        if (risk[0] == 'M') return oled_yellow;
    }

    return oled_green;
}

static int oled_score_to_percent(int score)
{
    score = oled_clamp_int(score, 0, 999);
    return (score * 100) / 999;
}

static int oled_score_to_999(int score)
{
    return oled_clamp_int(score, 0, 999);
}

/* 5x7 pixel font. This avoids unstable OLED text functions. */
static void oled_get_glyph(char c, u8 g[5])
{
    g[0] = 0; g[1] = 0; g[2] = 0; g[3] = 0; g[4] = 0;

    switch (c) {
    case 'A': g[0]=0x7E; g[1]=0x11; g[2]=0x11; g[3]=0x11; g[4]=0x7E; break;
    case 'B': g[0]=0x7F; g[1]=0x49; g[2]=0x49; g[3]=0x49; g[4]=0x36; break;
    case 'C': g[0]=0x3E; g[1]=0x41; g[2]=0x41; g[3]=0x41; g[4]=0x22; break;
    case 'D': g[0]=0x7F; g[1]=0x41; g[2]=0x41; g[3]=0x22; g[4]=0x1C; break;
    case 'E': g[0]=0x7F; g[1]=0x49; g[2]=0x49; g[3]=0x49; g[4]=0x41; break;
    case 'F': g[0]=0x7F; g[1]=0x09; g[2]=0x09; g[3]=0x09; g[4]=0x01; break;
    case 'G': g[0]=0x3E; g[1]=0x41; g[2]=0x49; g[3]=0x49; g[4]=0x7A; break;
    case 'H': g[0]=0x7F; g[1]=0x08; g[2]=0x08; g[3]=0x08; g[4]=0x7F; break;
    case 'I': g[0]=0x00; g[1]=0x41; g[2]=0x7F; g[3]=0x41; g[4]=0x00; break;
    case 'L': g[0]=0x7F; g[1]=0x40; g[2]=0x40; g[3]=0x40; g[4]=0x40; break;
    case 'M': g[0]=0x7F; g[1]=0x02; g[2]=0x0C; g[3]=0x02; g[4]=0x7F; break;
    case 'N': g[0]=0x7F; g[1]=0x04; g[2]=0x08; g[3]=0x10; g[4]=0x7F; break;
    case 'O': g[0]=0x3E; g[1]=0x41; g[2]=0x41; g[3]=0x41; g[4]=0x3E; break;
    case 'P': g[0]=0x7F; g[1]=0x09; g[2]=0x09; g[3]=0x09; g[4]=0x06; break;
    case 'R': g[0]=0x7F; g[1]=0x09; g[2]=0x19; g[3]=0x29; g[4]=0x46; break;
    case 'S': g[0]=0x46; g[1]=0x49; g[2]=0x49; g[3]=0x49; g[4]=0x31; break;
    case 'T': g[0]=0x01; g[1]=0x01; g[2]=0x7F; g[3]=0x01; g[4]=0x01; break;
    case 'U': g[0]=0x3F; g[1]=0x40; g[2]=0x40; g[3]=0x40; g[4]=0x3F; break;
    case 'V': g[0]=0x1F; g[1]=0x20; g[2]=0x40; g[3]=0x20; g[4]=0x1F; break;
    case 'W': g[0]=0x7F; g[1]=0x20; g[2]=0x18; g[3]=0x20; g[4]=0x7F; break;
    case 'Y': g[0]=0x07; g[1]=0x08; g[2]=0x70; g[3]=0x08; g[4]=0x07; break;
    case '0': g[0]=0x3E; g[1]=0x51; g[2]=0x49; g[3]=0x45; g[4]=0x3E; break;
    case '1': g[0]=0x00; g[1]=0x42; g[2]=0x7F; g[3]=0x40; g[4]=0x00; break;
    case '2': g[0]=0x42; g[1]=0x61; g[2]=0x51; g[3]=0x49; g[4]=0x46; break;
    case '3': g[0]=0x21; g[1]=0x41; g[2]=0x45; g[3]=0x4B; g[4]=0x31; break;
    case '4': g[0]=0x18; g[1]=0x14; g[2]=0x12; g[3]=0x7F; g[4]=0x10; break;
    case '5': g[0]=0x27; g[1]=0x45; g[2]=0x45; g[3]=0x45; g[4]=0x39; break;
    case '6': g[0]=0x3C; g[1]=0x4A; g[2]=0x49; g[3]=0x49; g[4]=0x30; break;
    case '7': g[0]=0x01; g[1]=0x71; g[2]=0x09; g[3]=0x05; g[4]=0x03; break;
    case '8': g[0]=0x36; g[1]=0x49; g[2]=0x49; g[3]=0x49; g[4]=0x36; break;
    case '9': g[0]=0x06; g[1]=0x49; g[2]=0x49; g[3]=0x29; g[4]=0x1E; break;
    case ':': g[0]=0x00; g[1]=0x36; g[2]=0x36; g[3]=0x00; g[4]=0x00; break;
    default: break;
    }
}

static void oled_draw_char(int x, int y, char c, u16 color)
{
    u8 glyph[5];
    int col;
    int row;

    if (!oled_hw_ready || !oled_display_ready) return;
    if (x > 90 || y > 56) return;

    /* Clear the 5x7 cell before redrawing to avoid old pixels. */
    OLEDrgb_DrawRectangle(&oled, x, y, x + 5, y + 7, oled_black, 1, oled_black);

    if (c == ' ') return;

    oled_get_glyph(c, glyph);

    for (col = 0; col < 5; col++) {
        for (row = 0; row < 7; row++) {
            if (glyph[col] & (1U << row)) {
                OLEDrgb_DrawPixel(&oled, x + col, y + row, color);
            }
        }
    }
}

static void oled_draw_text(int x, int y, const char *text, u16 color)
{
    if (text == NULL) return;

    while (*text && x < 92) {
        oled_draw_char(x, y, *text, color);
        x += 6;
        text++;
    }
}


static void oled_print_short_value(int x, int y, char label, int value, u16 color)
{
    char text[6];

    value = oled_score_to_999(value);

    text[0] = label;
    text[1] = ':';
    text[2] = (char)('0' + ((value / 100) % 10));
    text[3] = (char)('0' + ((value / 10) % 10));
    text[4] = (char)('0' + (value % 10));
    text[5] = '\0';

    oled_draw_text(x, y, text, color);
}

static int oled_dashboard_init(void)
{
    int status;

    oled_hw_ready = 0;
    oled_display_ready = 0;

    xil_printf("OLED dashboard init...\r\n");

    oled.GPIO_addr = XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR;
    XSpi_OLEDrgb.BaseAddress = XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR;

    status = OLEDrgb_SPIInit(&oled.OLEDSpi);
    if (status != XST_SUCCESS) {
        xil_printf("OLED SPI INIT FAIL, continuing without OLED\r\n");
        return 0;
    }

    OLEDrgb_HostInit(&oled);

    Xil_Out32(oled.GPIO_addr, 0xA);
    usleep(20000);
    Xil_Out32(oled.GPIO_addr, 0x8);
    usleep(1000);
    Xil_Out32(oled.GPIO_addr, 0xA);
    usleep(2000);

    oled_cmd2(0xFD, 0x12);
    oled_cmd1(OLED_CMD_DISPLAYOFF);

    oled_cmd2(OLED_CMD_SETREMAP, 0x72);
    oled_cmd2(OLED_CMD_SETDISPLAYSTARTLINE, 0x00);
    oled_cmd2(OLED_CMD_SETDISPLAYOFFSET, 0x00);
    oled_cmd1(OLED_CMD_NORMALDISPLAY);
    oled_cmd2(OLED_CMD_SETMULTIPLEXRATIO, 0x3F);
    oled_cmd2(OLED_CMD_SETMASTERCONFIGURE, 0x8E);
    oled_cmd2(OLED_CMD_POWERSAVEMODE, 0x0B);
    oled_cmd2(OLED_CMD_PHASEPERIODADJUSTMENT, 0x31);
    oled_cmd2(OLED_CMD_DISPLAYCLOCKDIV, 0xF0);
    oled_cmd2(OLED_CMD_SETPRECHARGESPEEDA, 0x64);
    oled_cmd2(OLED_CMD_SETPRECHARGESPEEDB, 0x78);
    oled_cmd2(OLED_CMD_SETPRECHARGESPEEDC, 0x64);
    oled_cmd2(OLED_CMD_SETPRECHARGEVOLTAGE, 0x3A);
    oled_cmd2(OLED_CMD_SETVVOLTAGE, 0x3E);
    oled_cmd2(OLED_CMD_MASTERCURRENTCONTROL, 0x06);
    oled_cmd2(OLED_CMD_SETCONTRASTA, 0x91);
    oled_cmd2(OLED_CMD_SETCONTRASTB, 0x50);
    oled_cmd2(OLED_CMD_SETCONTRASTC, 0x7D);
    oled_cmd1(OLED_CMD_DEACTIVESCROLLING);

    oled_clear_window_fast();

    Xil_Out32(oled.GPIO_addr, 0xE);
    usleep(25000);
    oled_cmd1(OLED_CMD_DISPLAYON);
    usleep(100000);

    oled_make_colors();

    oled_hw_ready = 1;
    oled_display_ready = 1;

    /* Clean static startup screen. */
    OLEDrgb_Clear(&oled);
    oled_draw_text(8, 5, "FMEAD", oled_cyan);
    oled_draw_text(8, 22, "SYSTEM OFF", oled_gray);
    oled_draw_text(8, 39, "SW0 ON TO RUN", oled_gray);

    xil_printf("OLED DASHBOARD OK\r\n");
    return 1;
}

static void oled_dashboard_update(void)
{
    static unsigned int tick = 0;
    int enabled;
    const char *event_text;
    const char *risk_text;
    u16 status_color;
    u16 event_color;
    u16 acoustic_color;
    u16 env_color;
    char line2[17];
    int i;
    int pos;

    if (!oled_hw_ready || !oled_display_ready) return;

    tick++;
    if (tick < OLED_UPDATE_TICKS) return;
    tick = 0;

    enabled = (g_switches & SW_DETECTOR_ENABLE) ? 1 : 0;

    /*
     * Full clear is intentional here.
     * The previous line-only clear left ghost text such as MEDGH and RUN.
     * This update runs slowly and only from the idle HTTP wait loop.
     */
    OLEDrgb_Clear(&oled);

    if (!enabled) {
        oled_draw_text(8, 5, "FMEAD", oled_cyan);
        oled_draw_text(8, 22, "SYSTEM OFF", oled_gray);
        oled_draw_text(8, 39, "SW0 ON TO RUN", oled_gray);
        return;
    }

    event_text = oled_event_short_name();
    risk_text = oled_risk_short_name();

    status_color = oled_color_from_status(g_event, g_risk);
    event_color = oled_color_from_status(event_text, g_risk);
    acoustic_color = (g_aco_state == ACO_CLAP) ? oled_red : oled_green;
    env_color = oled_color_from_status(g_env_event, g_env_risk);

    pos = 0;
    for (i = 0; event_text[i] && pos < 16; i++) {
        line2[pos++] = event_text[i];
    }
    if (pos < 16) {
        line2[pos++] = ' ';
    }
    for (i = 0; risk_text[i] && pos < 16; i++) {
        line2[pos++] = risk_text[i];
    }
    line2[pos] = '\0';

    oled_draw_text(8, 3, "FMEAD", oled_cyan);
    oled_draw_text(8, 17, line2, event_color);

    /* Cleaner two-column numeric layout. No bars, no RUN text. */
    oled_print_short_value(8, 35, 'M', g_score, status_color);
    oled_print_short_value(54, 35, 'A', g_mic_score, acoustic_color);
    oled_print_short_value(8, 50, 'E', g_env_score, env_color);

    (void)oled_score_to_percent(g_score);
}

/* ============================================================ */
/* Detector / fused state                                       */
/* ============================================================ */

static const char *risk_from_event(const char *event)
{
    if (event[0] == 'F') return "CRITICAL";
    if (event[0] == 'I') return "HIGH";
    if (event[0] == 'V') return "MEDIUM";
    if (event[0] == 'M') return "LOW";
    return "LOW";
}

static int event_is_anomaly(const char *event)
{
    if (event[0] == 'F' || event[0] == 'I' || event[0] == 'V') return 1;
    return 0;
}

static int risk_severity(const char *risk)
{
    if (risk[0] == 'C') return 3;
    if (risk[0] == 'H') return 2;
    if (risk[0] == 'M') return 1;
    return 0;
}

static int risk_is_anomaly(const char *risk)
{
    return (risk_severity(risk) > 0) ? 1 : 0;
}

static void update_environment_status(void)
{
    int delta_temp = iabs(g_fpga_temp_centi - g_temp_centi);
    int score = 0;

    g_env_event = "NORMAL";
    g_env_risk = "LOW";

    if (!bme280_ok && !xadc_ok) {
        g_env_event = "SENSORS OFFLINE";
        g_env_risk = "MEDIUM";
    } else if (xadc_ok && g_fpga_temp_centi >= ENV_FPGA_HIGH_C) {
        g_env_event = "FPGA HOT"; g_env_risk = "HIGH";
    } else if (bme280_ok && g_temp_centi >= ENV_AMBIENT_HIGH_C) {
        g_env_event = "AMBIENT HOT"; g_env_risk = "HIGH";
    } else if (bme280_ok && g_hum_centi >= ENV_HUM_HIGH_CENTI) {
        g_env_event = "HUMIDITY HIGH"; g_env_risk = "MEDIUM";
    } else if (bme280_ok && g_hum_centi > 0 && g_hum_centi <= ENV_HUM_LOW_CENTI) {
        g_env_event = "HUMIDITY LOW"; g_env_risk = "MEDIUM";
    } else if (xadc_ok && bme280_ok && delta_temp >= ENV_DELTA_WARN_C) {
        g_env_event = "THERMAL GAP"; g_env_risk = "MEDIUM";
    } else if (xadc_ok && g_fpga_temp_centi >= ENV_FPGA_WARN_C) {
        g_env_event = "FPGA WARM"; g_env_risk = "LOW";
    } else if (bme280_ok && g_temp_centi >= ENV_AMBIENT_WARN_C) {
        g_env_event = "AMBIENT WARM"; g_env_risk = "LOW";
    }

    if (xadc_ok)  score += (g_fpga_temp_centi > 3000) ? (g_fpga_temp_centi - 3000) / 4 : 0;
    if (bme280_ok) score += (g_temp_centi > 2600) ? (g_temp_centi - 2600) / 3 : 0;
    if (bme280_ok && g_hum_centi >= ENV_HUM_HIGH_CENTI) score += (g_hum_centi - ENV_HUM_HIGH_CENTI) / 3;
    if (bme280_ok && g_hum_centi > 0 && g_hum_centi <= ENV_HUM_LOW_CENTI) score += (ENV_HUM_LOW_CENTI - g_hum_centi) / 3;
    if (xadc_ok && bme280_ok && delta_temp >= ENV_DELTA_WARN_C) score += (delta_temp - ENV_DELTA_WARN_C) / 2;

    if (score < 0) score = 0;
    if (score > 999) score = 999;
    g_env_score = score;
}

static void update_overall_status(void)
{
    int motion_sev = risk_severity(g_motion_risk);
    int env_sev = risk_severity(g_env_risk);
    int acoustic_sev = risk_severity(g_acoustic_risk);

    if (g_motion_event[0] == 'D' && g_env_event[0] == 'D' && g_acoustic_event[0] == 'D') {
        g_event = "DISABLED";
        g_risk = "LOW";
        return;
    }

    g_event = g_motion_event;
    g_risk = g_motion_risk;

    if (env_sev > motion_sev) {
        g_event = g_env_event; g_risk = g_env_risk;
        motion_sev = env_sev;
    }
    if (acoustic_sev > motion_sev) {
        g_event = g_acoustic_event; g_risk = g_acoustic_risk;
        motion_sev = acoustic_sev;
    }

    if (motion_sev == 0) {
        if (g_motion_event[0] == 'M') {
            g_event = g_motion_event; g_risk = g_motion_risk;
        } else if (g_aco_state == ACO_CLAP) {
            g_event = g_acoustic_event; g_risk = g_acoustic_risk;
        } else {
            g_event = g_env_event; g_risk = g_env_risk;
        }
    }
}

static u32 build_leds(int anomaly_active)
{
    static int led_value_ema = 0;
    static int prev_bars = 0;
    u32 leds = 0;
    int bars = 0;
    int value = 0;
    int i;
    int page_anomaly = anomaly_active;

    if (!(g_switches & SW_DETECTOR_ENABLE)) {
        led_value_ema = 0;
        prev_bars = 0;
        return 0;
    }
    leds |= 0x0001;
    if (!(g_switches & SW_LED_BAR_ENABLE)) return leds;
    leds |= 0x0002;

    if (g_web_page == WEB_PAGE_ACOUSTIC) {
        value = g_mic_score;
        page_anomaly = (g_aco_state == ACO_CLAP);
    } else if (g_web_page == WEB_PAGE_ENVIRONMENT) {
        value = g_env_score;
        page_anomaly = risk_is_anomaly(g_env_risk);
    } else if (g_web_page == WEB_PAGE_MOTION) {
        value = g_score;
        page_anomaly = event_is_anomaly(g_motion_event);
    } else {
        value = g_score;
        page_anomaly = anomaly_active;
    }

    if (value < 0) value = 0;
    if (value > 999) value = 999;

    /* EMA smoothing: rises in ~3 frames, decays in ~10 frames.   */
    /* Asymmetric on purpose: fast attack so events are visible,  */
    /* slow release so the bar doesn't flicker.                   */
    if (value > led_value_ema) {
        led_value_ema += (value - led_value_ema) >> 1;     /* fast attack */
    } else {
        led_value_ema += (value - led_value_ema) >> 3;     /* slow release */
    }

    bars = (led_value_ema * 14) / 1000;
    if (bars > 14) bars = 14;
    if (bars < 0) bars = 0;

    /* Hysteresis: bar only drops if change is > 1 level, avoids  */
    /* the lowest LED of the active group from blinking.          */
    if (bars < prev_bars && (prev_bars - bars) == 1) bars = prev_bars;
    prev_bars = bars;

    for (i = 0; i < bars; i++) leds |= (1U << (2 + i));
    if (page_anomaly) leds |= 0x8000;

    return leds;
}

static void detector_update(void)
{
    int detector_enabled;
    int raw_x, raw_y, raw_z;
    int orient_change;
    int anomaly_active = 0;
    u32 leds;
    int center_pressed;
    int right_pressed;
    static const char *last_event = "NORMAL";
    static int prev_detector_enabled = 0;

    g_switches = (int)XGpio_DiscreteRead(&gpio_switches, 1);
    g_buttons  = (int)XGpio_DiscreteRead(&gpio_buttons, 1);

    center_pressed = (g_buttons & BTN_CENTER_MASK) && !(prev_buttons & BTN_CENTER_MASK);
    right_pressed  = (g_buttons & BTN_RIGHT_MASK)  && !(prev_buttons & BTN_RIGHT_MASK);
    prev_buttons = g_buttons;

    acoustic_update();

    if (xadc_ok) g_fpga_temp_centi = xadc_read_fpga_temp_centi();

    detector_enabled = (g_switches & SW_DETECTOR_ENABLE) ? 1 : 0;
    if (detector_enabled && !prev_detector_enabled && pcm_ok) acoustic_reset();
    prev_detector_enabled = detector_enabled;

    if (center_pressed && adxl_ok) {
        if (calibrate_baseline_quality()) {
            g_cooldown = 0;
        } else {
            xil_printf("BTNC: hold the board still and press again\r\n");
        }
    }

    if (right_pressed) {
        g_alarm_count = 0;
        g_max_score = 0;
        g_cooldown = 0;
        xil_printf("Alarm counter reset\r\n");
    }

    if (adxl_ok) {
        read_xyz(&raw_x, &raw_y, &raw_z);
    } else {
        raw_x = 0; raw_y = 0; raw_z = 980;
    }

    motion_features_update(raw_x, raw_y, raw_z, &g_score, &g_delta, &orient_change);
    g_x = filt_x; g_y = filt_y; g_z = filt_z;

    if (!detector_enabled || !adxl_ok) {
        g_motion_event = "DISABLED";
        g_motion_risk = "LOW";
        anomaly_active = 0;
        event_hold_cycles = 0;
    } else {
        const char *raw_event = classify_event_robust(g_score, g_delta, orient_change);

        if (raw_event[0] == 'N') {
            if (event_hold_cycles > 0) {
                event_hold_cycles--;
                g_motion_event = last_event;
                g_motion_risk = risk_from_event(last_event);
                anomaly_active = event_is_anomaly(last_event);
            } else {
                g_motion_event = "NORMAL";
                g_motion_risk = "LOW";
                anomaly_active = 0;
            }
        } else {
            g_motion_event = raw_event;
            g_motion_risk = risk_from_event(raw_event);
            anomaly_active = event_is_anomaly(raw_event);
            event_hold_cycles = EVENT_HOLD_CYCLES;
            last_event = raw_event;
        }
    }

    if (!detector_enabled) {
        g_acoustic_event = "DISABLED";
        g_acoustic_risk = "LOW";
        if (pcm_ok) acoustic_reset();
        g_env_event = "DISABLED";
        g_env_risk = "LOW";
    } else {
        update_environment_status();
    }

    update_overall_status();

    if (g_score > g_max_score) g_max_score = g_score;
    anomaly_active = event_is_anomaly(g_motion_event) ||
                     risk_is_anomaly(g_env_risk) ||
                     (g_aco_state == ACO_CLAP);

    if (anomaly_active && g_cooldown == 0) {
        g_alarm_count++;
        g_cooldown = COOLDOWN_CYCLES;
    }
    if (g_cooldown > 0) g_cooldown--;

    leds = build_leds(anomaly_active);
    XGpio_DiscreteWrite(&gpio_leds, 1, leds);
}

/* ============================================================ */
/* Local output service (keeps 7-seg scanning during UART waits)*/
/* ============================================================ */

static void service_local_outputs(void)
{
    sevenseg_update_page();
    sevenseg_scan_task();
}

static void service_delay_ms(unsigned int ms)
{
    unsigned int i;
    for (i = 0; i < ms; i++) {
        service_local_outputs();
        usleep(1000);
    }
}

/* ============================================================ */
/* ESP32 UART                                                   */
/* ============================================================ */

static void esp_gpio_init(void)
{
    Xil_Out32(ESP_GPIO_BASEADDR + GPIO_CH1_TRI, 0x03);
    Xil_Out32(ESP_GPIO_BASEADDR + GPIO_CH2_TRI, 0x09);
    Xil_Out32(ESP_GPIO_BASEADDR + GPIO_CH2_DATA, 0x00);
    usleep(200000);
    Xil_Out32(ESP_GPIO_BASEADDR + GPIO_CH2_DATA, 0x02);
    sleep(3);
}

static void esp_tx(const char *s)
{
    while (*s) {
        while (XUartLite_IsTransmitFull(ESP_UART_BASEADDR)) service_local_outputs();
        XUartLite_SendByte(ESP_UART_BASEADDR, (u8)*s);
        service_local_outputs();
        s++;
    }
}

static void esp_txbuf(u8 *b, unsigned int n)
{
    unsigned int i;
    for (i = 0; i < n; i++) {
        while (XUartLite_IsTransmitFull(ESP_UART_BASEADDR)) service_local_outputs();
        XUartLite_SendByte(ESP_UART_BASEADDR, b[i]);
        if ((i & 0x1FU) == 0U) service_local_outputs();
    }
}

static int esp_rx(u8 *c)
{
    if (!XUartLite_IsReceiveEmpty(ESP_UART_BASEADDR)) {
        *c = XUartLite_RecvByte(ESP_UART_BASEADDR);
        return 1;
    }
    return 0;
}

static void esp_flush(void)
{
    u8 c;
    int quiet = 0;
    while (quiet < 50) {
        if (esp_rx(&c)) {
            quiet = 0;
        } else {
            service_delay_ms(1);
            quiet++;
        }
    }
}

static int match_token(const char *pat, char c, unsigned int *st)
{
    if (c == pat[*st]) {
        (*st)++;
        if (!pat[*st]) { *st = 0; return 1; }
    } else {
        *st = (c == pat[0]) ? 1U : 0U;
    }
    return 0;
}

static int esp_wait(const char *tok, unsigned int ms)
{
    unsigned int elapsed = 0;
    unsigned int state = 0;
    u8 c;

    while (elapsed < ms) {
        if (esp_rx(&c)) {
#if UART_VERBOSE
            xil_printf("%c", c);
#endif
            if (match_token(tok, (char)c, &state)) return 1;
        } else {
            service_delay_ms(1);
            elapsed++;
        }
    }
    return 0;
}

static int esp_cmd(const char *cmd, const char *expected, unsigned int timeout_ms)
{
    esp_flush();
    esp_tx(cmd);
    if (esp_wait(expected, timeout_ms)) return 1;
    xil_printf("ESP CMD FAIL: %s", cmd);
    return 0;
}

static int esp_init(void)
{
    int i;

    xil_printf("ESP32 reset...\r\n");
    esp_gpio_init();
    esp_flush();

    xil_printf("ESP32 AT check...\r\n");
    for (i = 0; i < 5; i++) {
        if (esp_cmd("AT\r\n", "OK", 3000)) {
            xil_printf("ESP32 AT OK\r\n");
            break;
        }
        sleep(1);
    }
    if (i == 5) { xil_printf("ESP32 AT FAIL\r\n"); return 0; }

    esp_cmd("ATE0\r\n", "OK", 2000);

    /* Stop any previous server state before creating the AP again.
     * This is safe if no server is active; failures are ignored.
     */
    esp_tx("AT+CIPSERVER=0\r\n");
    esp_wait("OK", 2000);
    esp_flush();

    if (!esp_cmd("AT+CWMODE=2\r\n", "OK", 5000)) {
        xil_printf("ESP32 WiFi mode FAIL\r\n");
        return 0;
    }
    if (!esp_cmd("AT+CWSAP=\"FMEAD_FPGA\",\"12345678\",5,3\r\n", "OK", 8000)) {
        xil_printf("ESP32 AP FAIL\r\n");
        return 0;
    }
    xil_printf("ESP32 AP OK\r\n");

    if (!esp_cmd("AT+CIPMUX=1\r\n", "OK", 3000)) { xil_printf("ESP32 CIPMUX FAIL\r\n"); return 0; }
    esp_cmd("AT+CIPSERVERMAXCONN=1\r\n", "OK", 3000);

    if (!esp_cmd("AT+CIPSERVER=1,80\r\n", "OK", 5000)) {
        xil_printf("ESP32 HTTP SERVER FAIL\r\n");
        return 0;
    }

    xil_printf("ESP32 HTTP SERVER OK\r\n");
    xil_printf("SSID: FMEAD_FPGA\r\n");
    xil_printf("PASS: 12345678\r\n");
    xil_printf("URL : http://192.168.4.1/\r\n");
    return 1;
}

/* ============================================================ */
/* Web dashboard HTML builder                                   */
/* ============================================================ */

static int build_index_page(void)
{
    char *p = page;

    p = as(p, "HTTP/1.1 200 OK\r\nContent-Type:text/html\r\nCache-Control:no-store\r\nConnection:close\r\n\r\n");

    p = as(p, "<!doctype html><html><head><meta charset=utf-8>"
              "<meta name=viewport content=\"width=device-width,initial-scale=1\">"
              "<title>FMEAD-FPGA</title><style>"
              "*{box-sizing:border-box;margin:0;padding:0}"
              ":root{--bg:#070b12;--p1:#0f1521;--p2:#161e2d;--p3:#1d263a;--bd:#243049;--bd2:#324362;"
              "--tx:#e6edf7;--mu:#8693ad;--mu2:#a8b3c8;--cy:#3ec6ff;--gn:#22d97f;--am:#ffb02e;"
              "--rd:#ff4d6d;--vt:#9d6bff;--or:#ff8a47;--m:ui-monospace,Consolas,monospace}"
              ""
              "body{background:var(--bg);color:var(--tx);font-family:-apple-system,Segoe UI,Arial,sans-serif;font-size:14px;line-height:1.45}"
              ".app,.m,.kp,.sb,.es,.lb,.lg,.tb,.bg,.swc,.rw,.ds,.bs,.nt,.ap,.ft,.en,.bv .x{font-variant-numeric:tabular-nums}"
              ".app{max-width:1280px;margin:auto;padding:14px}"
              ".app.st{opacity:.55}"
              ".hd{background:var(--p1);border:1px solid var(--bd);border-radius:12px;padding:14px 18px;margin-bottom:12px}"
              ".ht{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:12px;margin-bottom:12px}"
              ".bn{display:flex;align-items:center;gap:12px}"
              ".bm{width:38px;height:38px;border-radius:9px;background:var(--cy);display:flex;align-items:center;justify-content:center;font-weight:800;font-size:13px;font-family:var(--m);color:#000}"
              ".bt{font-weight:700;font-size:16px}"
              ".bs{color:var(--mu);font-size:11px;font-family:var(--m);margin-top:2px}"
              ".kp{display:flex;gap:14px;font-family:var(--m);font-size:11px;color:var(--mu);flex-wrap:wrap;align-items:center}"
              ".kp b{color:var(--tx);margin-left:5px}"
              ".lk{display:flex;align-items:center;gap:8px;padding:6px 13px;border-radius:999px;background:var(--p2);border:1px solid var(--bd2);font-family:var(--m);font-size:11px}"
              ".dt{width:8px;height:8px;border-radius:50%;background:var(--gn);color:var(--gn)}"
              ".lk.w .dt{background:var(--am);color:var(--am)}"
              ".lk.b .dt{background:var(--rd);color:var(--rd)}"
              ".tbs{display:flex;gap:6px;flex-wrap:wrap}"
              ".tb{background:transparent;color:var(--mu);border:1px solid var(--bd);padding:9px 16px;border-radius:8px;font-family:var(--m);font-size:11px;cursor:pointer;text-transform:uppercase}"
              ".tb:hover{color:var(--mu2);border-color:var(--bd2)}"
              ".tb.on{background:var(--p3);color:var(--cy);border-color:var(--cy)}"
              ".sb{background:var(--p1);border:1px solid var(--bd);border-radius:10px;padding:10px 16px;margin-bottom:12px;display:flex;flex-wrap:wrap;gap:18px;font-family:var(--m);font-size:11px;align-items:center}"
              ".sg{display:flex;align-items:center;gap:8px}"
              ".sg .l{color:var(--mu);text-transform:uppercase;font-size:10px}"
              ".v{color:var(--tx);font-weight:700}"
              ".v.lo{color:var(--gn)}.v.md{color:var(--am)}.v.hi{color:var(--or)}.v.cr{color:var(--rd)}.v.of{color:var(--mu)}"
              ".sp{width:1px;height:14px;background:var(--bd2)}"
              ".pg{display:none}.pg.on{display:block}"
              ".r{display:grid;gap:12px;margin-bottom:12px}"
              ".r2{grid-template-columns:1fr 1fr}.r3{grid-template-columns:repeat(3,1fr)}"
              ".r4{grid-template-columns:repeat(4,1fr)}.rh{grid-template-columns:1.6fr 1fr}"
              ".c{background:var(--p1);border:1px solid var(--bd);border-radius:12px;padding:16px}"
              ".c h3{font-size:11px;font-weight:700;text-transform:uppercase;color:var(--mu);margin-bottom:13px}"
              ".lb{font-family:var(--m);font-size:10px;text-transform:uppercase;color:var(--mu)}"
              ".eh{padding:22px}.eh.cr{border-color:rgba(255,77,109,.4)}.eh.hi{border-color:rgba(255,138,71,.35)}.eh.md{border-color:rgba(255,176,46,.3)}"
              ".en{font-size:42px;font-weight:800;line-height:1;margin-top:10px}"
              ".en.lo{color:var(--cy)}.en.md{color:var(--am)}.en.hi{color:var(--or)}.en.cr{color:var(--rd)}.en.of{color:var(--mu)}"
              ".er{display:flex;align-items:center;gap:12px;margin-top:16px;flex-wrap:wrap}"
              ".bg{padding:6px 13px;border-radius:6px;font-family:var(--m);font-size:10px;font-weight:700;border:1px solid}"
              ".bg.lo{background:#1a3a26;border-color:#2c5d3f;color:var(--gn)}"
              ".bg.md{background:#3a2c12;border-color:#5d4220;color:var(--am)}"
              ".bg.hi{background:#3a2418;border-color:#5d3a25;color:var(--or)}"
              ".bg.cr{background:#3a1820;border-color:#5d2535;color:var(--rd)}"
              ".es{font-family:var(--m);font-size:11px;color:var(--mu)}.es b{color:var(--tx)}"
              ".mt{margin-top:16px}"
              ".mh{display:flex;justify-content:space-between;font-family:var(--m);font-size:10px;color:var(--mu);text-transform:uppercase;margin-bottom:6px}"
              ".mbr{height:10px;background:#060a10;border:1px solid var(--bd);border-radius:5px;overflow:hidden}"
              ".mf{height:100%;width:0%}"
              ".mf.m{background:var(--cy)}"
              ".mf.s{background:var(--gn)}"
              ".bv{display:flex;align-items:baseline;gap:8px;margin-top:8px}"
              ".bv .x{font-family:var(--m);font-size:30px;font-weight:800}"
              ".bv .x.cy{color:var(--cy)}.bv .x.gn{color:var(--gn)}.bv .x.vt{color:var(--vt)}.bv .x.or{color:var(--or)}"
              ".bv .u{color:var(--mu);font-size:12px;font-family:var(--m)}"
              ".ds{font-size:10px;color:var(--mu);margin-top:6px;font-family:var(--m)}"
              ".sc{border-top:2px solid var(--bd2)}"
              ".sc.x{border-top-color:var(--cy)}.sc.y{border-top-color:var(--gn)}.sc.z{border-top-color:var(--am)}"
              ".sc.o{border-top-color:var(--vt)}.sc.t{border-top-color:var(--cy)}.sc.p{border-top-color:var(--vt)}"
              ".sc.h{border-top-color:var(--gn)}.sc.f{border-top-color:var(--or)}.sc.q{border-top-color:var(--vt)}"
              ".rw{display:flex;justify-content:space-between;align-items:center;padding:9px 13px;background:var(--p2);border:1px solid var(--bd);border-radius:7px;font-family:var(--m);font-size:11px;margin-bottom:6px}"
              ".rw .n{color:var(--mu2);display:flex;align-items:center;gap:9px}"
              ".ld{width:7px;height:7px;border-radius:50%}"
              ".ld.ok{background:var(--gn);color:var(--gn)}"
              ".ld.wn{background:var(--am);color:var(--am)}"
              ".ld.bd{background:var(--rd);color:var(--rd)}"
              ".ld.of{background:#3a4458;color:transparent}"
              ".ok{color:var(--gn)}.wn{color:var(--am)}.bd{color:var(--rd)}.of{color:var(--mu)}"
              ".lg{display:flex;gap:12px;font-family:var(--m);font-size:10px;color:var(--mu);margin-bottom:8px}"
              ".lg span{display:flex;align-items:center;gap:5px}"
              ".lc{width:10px;height:2px}"
              ".cw{height:240px;position:relative;background:#0a1018;border:1px solid var(--bd);border-radius:8px;padding:8px}.cw canvas{width:100%;height:100%;display:block}"
              ".ap{display:inline-block;padding:5px 14px;border-radius:14px;font-family:var(--m);font-size:11px;font-weight:700;border:1px solid}"
              ".ap.q{background:#1a3a26;color:var(--gn);border-color:#2c5d3f}"
              ".ap.p{background:#3a1820;color:var(--rd);border-color:#5d2535}"
              ".nt{padding:10px 14px;background:#152a3d;border:1px solid #1f3e5c;border-radius:8px;color:var(--cy);font-size:11px;font-family:var(--m);margin-bottom:12px}"
              ".dis{padding:50px 30px;text-align:center;color:var(--mu);background:var(--p1);border:1px dashed var(--bd2);border-radius:12px;margin-bottom:12px}"
              ".dis .lb{color:var(--rd);font-size:11px;margin-bottom:8px}"
              ".dis .i{font-size:22px;font-weight:700;color:var(--tx);margin-bottom:6px}"
              ".dis .d{font-size:12px;color:var(--mu2)}"
              ".sw{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}"
              ".swc{background:var(--p2);border:1px solid var(--bd);border-radius:7px;padding:9px 12px;font-family:var(--m);font-size:10px;display:flex;justify-content:space-between;align-items:center}"
              ".swc.o{border-color:#2c5d3f;background:#0e1f17}"
              ".swc.r{opacity:.5}"
              ".swc .n{color:var(--mu2)}"
              ".swc .s{font-weight:700;color:var(--mu)}"
              ".swc.o .s{color:var(--gn)}"
              ".ft{text-align:center;margin-top:18px;font-family:var(--m);font-size:10px;color:var(--mu)}"
              "body.off .liv{display:none !important}body.off .ofs{display:block !important}.ofs{display:none}"
              "@media(max-width:980px){.r4{grid-template-columns:repeat(2,1fr)}.r3,.r2,.rh{grid-template-columns:1fr}.sw{grid-template-columns:repeat(2,1fr)}.en{font-size:32px}.bv .x{font-size:24px}}"
              "</style></head><body>");

    p = as(p, "<div class=app id=app>"
              "<div class=hd><div class=ht><div class=bn><div class=bm>FE</div>"
              "<div><div class=bt>FMEAD-FPGA</div><div class=bs>Edge Anomaly Detector</div></div></div>"
              "<div class=kp><span>UP<b id=ucnt>0</b></span><span>LAT<b id=ulat>0ms</b></span>"
              "<span>T<b id=uupt>00:00</b></span>"
              "<div class=lk id=lnk><span class=dt></span><span id=lst>ONLINE</span></div></div></div>"
              "<div class=tbs>"
              "<button class=\"tb on\" data-p=home>Home</button>"
              "<button class=tb data-p=motion>Motion</button>"
              "<button class=tb data-p=environment>Environment</button>"
              "<button class=tb data-p=acoustic>Acoustic</button>"
              "<button class=tb data-p=system>System</button>"
              "</div></div>"
              "<div class=sb>"
              "<div class=sg><span class=l>SYS</span><span class=\"v lo\" id=Ss>--</span></div><div class=sp></div>"
              "<div class=sg><span class=l>EVENT</span><span class=\"v lo\" id=Se>--</span></div><div class=sp></div>"
              "<div class=sg><span class=l>RISK</span><span class=\"v lo\" id=Sr>--</span></div><div class=sp></div>"
              "<div class=sg><span class=l>SW0</span><span class=\"v lo\" id=S0>--</span></div><div class=sp></div>"
              "<div class=sg><span class=l>SW1</span><span class=\"v of\" id=S1>--</span></div><div class=sp></div>"
              "<div class=sg><span class=l>7SEG</span><span class=\"v lo\" id=Sp>HOME</span></div></div>");

    /* HOME PAGE */
    p = as(p, "<div class=\"pg on\" id=P_home>"
              "<div class=ofs><div class=dis><div class=lb>SYSTEM DISABLED</div>"
              "<div class=i>Turn SW0 ON to start</div>"
              "<div class=d>Sensors paused. LEDs and 7-segment are off.</div></div></div>"
              "<div class=liv>"
              "<div class=\"r rh\">"
              "<div class=\"c eh\" id=Hh><div class=lb>Detected event</div>"
              "<div class=\"en lo\" id=He>NORMAL</div>"
              "<div class=er><div class=\"bg lo\" id=Hr>LOW RISK</div>"
              "<div class=es>ALARMS <b id=Ha>0</b> | PEAK <b id=Hm>0</b> | ORIENT <b id=Ho>--</b></div></div>"
              "<div class=mt><div class=mh><span>Motion intensity</span><span id=Hp>0%</span></div>"
              "<div class=mbr><div class=\"mf m\" id=Hb></div></div></div></div>"
              "<div class=c><h3>Modules</h3>"
              "<div class=rw><span class=n><span class=\"ld ok\" id=L1></span>Motion ADXL362</span><span class=\"v ok\" id=V1>READY</span></div>"
              "<div class=rw><span class=n><span class=\"ld ok\" id=L2></span>Environment BME+XADC</span><span class=\"v ok\" id=V2>READY</span></div>"
              "<div class=rw><span class=n><span class=\"ld ok\" id=L3></span>Acoustic PDM mic</span><span class=\"v ok\" id=V3>READY</span></div>"
              "<div class=rw><span class=n><span class=\"ld ok\"></span>ESP32 WiFi AP</span><span class=\"v ok\">ONLINE</span></div>"
              "</div></div>"
              "<div class=\"r r4\">"
              "<div class=\"c sc x\"><div class=lb>Motion score</div><div class=bv><span class=\"x cy\" id=Hs>0</span><span class=u>/999</span></div><div class=ds>ADXL362</div></div>"
              "<div class=\"c sc t\"><div class=lb>Ambient</div><div class=bv><span class=\"x cy\" id=Hbt>--</span><span class=u>C</span></div><div class=ds>BME280</div></div>"
              "<div class=\"c sc f\"><div class=lb>FPGA die</div><div class=bv><span class=\"x or\" id=Hft>--</span><span class=u>C</span></div><div class=ds>XADC</div></div>"
              "<div class=\"c sc q\"><div class=lb>Acoustic</div><div class=bv style=margin-top:6px><span class=\"ap q\" id=Hac>QUIET</span></div><div class=ds id=Hms>env=0</div></div>"
              "</div></div></div>");

    /* MOTION PAGE */
    p = as(p, "<div class=pg id=P_motion>"
              "<div class=ofs><div class=dis><div class=lb>MOTION DISABLED</div>"
              "<div class=i>SW0 is OFF</div><div class=d>Motion sensing paused.</div></div></div>"
              "<div class=liv>"
              "<div class=\"r r4\">"
              "<div class=\"c sc x\"><div class=lb>X axis</div><div class=bv><span class=\"x cy\" id=mX>0</span><span class=u>raw</span></div><div class=ds>ADXL</div></div>"
              "<div class=\"c sc y\"><div class=lb>Y axis</div><div class=bv><span class=\"x gn\" id=mY>0</span><span class=u>raw</span></div><div class=ds>ADXL</div></div>"
              "<div class=\"c sc z\"><div class=lb>Z axis</div><div class=bv><span class=\"x or\" id=mZ>0</span><span class=u>raw</span></div><div class=ds>ADXL</div></div>"
              "<div class=\"c sc o\"><div class=lb>Orientation</div><div class=bv style=margin-top:8px><span class=\"x vt\" id=mO style=font-size:18px>--</span></div><div class=ds>Posture</div></div>"
              "</div>"
              "<div class=\"r r3\">"
              "<div class=c><div class=lb>Score</div><div class=bv><span class=\"x cy\" id=mS>0</span><span class=u>/999</span></div><div class=mt><div class=mbr><div class=\"mf m\" id=mB></div></div></div></div>"
              "<div class=c><div class=lb>Delta</div><div class=bv><span class=\"x cy\" id=mD>0</span></div><div class=ds style=margin-top:14px>Sample change</div></div>"
              "<div class=c><div class=lb>Event</div><div class=bv><span class=\"x cy\" id=mE style=font-size:22px>NORMAL</span></div><div class=ds style=margin-top:8px>A <b style=color:var(--tx) id=mA>0</b> | P <b style=color:var(--tx) id=mM>0</b></div></div>"
              "</div>"
              "<div class=c><h3>Live trace X / Y / Z</h3>"
              "<div class=lg><span><span class=lc style=background:#3ec6ff></span>X</span>"
              "<span><span class=lc style=background:#22d97f></span>Y</span>"
              "<span><span class=lc style=background:#ffb02e></span>Z</span></div>"
              "<div class=cw><canvas id=c1></canvas></div></div>"
              "</div></div>");

    /* ENVIRONMENT PAGE */
    p = as(p, "<div class=pg id=P_environment>"
              "<div class=ofs><div class=dis><div class=lb>ENVIRONMENT DISABLED</div>"
              "<div class=i>SW0 is OFF</div><div class=d>BME280 and XADC paused.</div></div></div>"
              "<div class=liv>"
              "<div class=\"r r4\">"
              "<div class=\"c sc t\"><div class=lb>Ambient temp</div><div class=bv><span class=\"x cy\" id=eT>--</span><span class=u>C</span></div><div class=ds>BME280</div></div>"
              "<div class=\"c sc p\"><div class=lb>Pressure</div><div class=bv><span class=\"x vt\" id=eP>--</span><span class=u>hPa</span></div><div class=ds>Local</div></div>"
              "<div class=\"c sc h\"><div class=lb>Humidity</div><div class=bv><span class=\"x gn\" id=eH>--</span><span class=u>%</span></div><div class=ds>Relative</div></div>"
              "<div class=\"c sc f\"><div class=lb>FPGA die</div><div class=bv><span class=\"x or\" id=eF>--</span><span class=u>C</span></div><div class=ds>XADC</div></div>"
              "</div>"
              "<div class=\"r r2\">"
              "<div class=c><h3>Temperature trends</h3>"
              "<div class=lg><span><span class=lc style=background:#3ec6ff></span>Ambient</span>"
              "<span><span class=lc style=background:#ff8a47></span>FPGA</span></div>"
              "<div class=cw><canvas id=c2></canvas></div></div>"
              "<div class=c><h3>Sensor health</h3>"
              "<div class=rw><span class=n><span class=\"ld ok\" id=eLb></span>BME280</span><span class=\"v ok\" id=eVb>OK</span></div>"
              "<div class=rw><span class=n><span class=\"ld ok\" id=eLx></span>XADC</span><span class=\"v ok\" id=eVx>OK</span></div>"
              "<div class=rw><span class=n><span class=\"ld ok\"></span>Env event</span><span class=v id=eBa>--</span></div>"
              "<div class=rw><span class=n><span class=\"ld ok\"></span>Risk</span><span class=v id=eTd>--</span></div>"
              "</div></div>"
              "</div></div>");

    /* ACOUSTIC PAGE */
    p = as(p, "<div class=pg id=P_acoustic>"
              "<div class=ofs><div class=dis><div class=lb>ACOUSTIC DISABLED</div>"
              "<div class=i>SW0 is OFF</div><div class=d>Mic pipeline paused.</div></div></div>"
              "<div class=liv>"
              "<div class=nt>Acoustic event detection only: QUIET or CLAP / IMPACT. Not speech recognition.</div>"
              "<div class=\"r rh\">"
              "<div class=\"c eh\"><div class=lb>Acoustic state</div>"
              "<div class=\"en lo\" id=aE>QUIET</div>"
              "<div class=er><div class=\"ap q\" id=aP>QUIET</div>"
              "<div class=es>ENV <b id=aMe>0</b> | SC <b id=aMs>0</b>/999 | BASE <b id=aMb>0</b></div></div>"
              "<div class=mt><div class=mh><span>Sound intensity</span><span id=aPc>0%</span></div>"
              "<div class=mbr><div class=\"mf s\" id=aB></div></div></div></div>"
              "<div class=c><h3>Mic raw values</h3>"
              "<div class=rw><span class=n>MIC_RAW</span><span class=v id=aMr>0</span></div>"
              "<div class=rw><span class=n>MIC_DIFF</span><span class=v id=aMd>0</span></div>"
              "<div class=rw><span class=n>ENVELOPE</span><span class=v id=aMe2>0</span></div>"
              "<div class=rw><span class=n>BASELINE</span><span class=v id=aMb2>0</span></div>"
              "<div class=rw><span class=n>STATE</span><span class=v id=aSt>QUIET</span></div>"
              "</div></div>"
              "<div class=c><h3>Acoustic trace</h3>"
              "<div class=lg><span><span class=lc style=background:#3ec6ff></span>Envelope</span>"
              "<span><span class=lc style=background:#9d6bff></span>Score</span>"
              "<span><span class=lc style=background:#ffb02e></span>Peak</span></div>"
              "<div class=cw><canvas id=c3></canvas></div></div>"
              "</div></div>");

    /* SYSTEM PAGE */
    p = as(p, "<div class=pg id=P_system>"
              "<div class=\"r r2\">"
              "<div class=c><h3>Switches</h3><div class=sw id=swg></div>"
              "<div class=ds style=margin-top:10px;line-height:1.7>SW0 board ON/OFF. SW1 LED bar. SW2-15 reserved. Tabs control 7-seg.</div></div>"
              "<div class=c><h3>Hardware</h3>"
              "<div class=rw><span class=n><span class=\"ld ok\" id=hLa></span>ADXL362</span><span class=\"v ok\" id=hVa>READY</span></div>"
              "<div class=rw><span class=n><span class=\"ld ok\" id=hLb></span>BME280</span><span class=\"v ok\" id=hVb>READY</span></div>"
              "<div class=rw><span class=n><span class=\"ld ok\" id=hLx></span>XADC</span><span class=\"v ok\" id=hVx>READY</span></div>"
              "<div class=rw><span class=n><span class=\"ld ok\" id=hLm></span>PDM Mic</span><span class=\"v ok\" id=hVm>READY</span></div>"
              "<div class=rw><span class=n><span class=\"ld wn\" id=hLo></span>OLED</span><span class=\"v wn\" id=hVo>PENDING</span></div>"
              "<div class=rw><span class=n><span class=\"ld ok\"></span>ESP32 AP</span><span class=\"v ok\">ONLINE</span></div>"
              "</div></div>"
              "<div class=\"r r2\">"
              "<div class=c><h3>Debug</h3>"
              "<div class=rw><span class=n>System</span><span class=v id=dS>--</span></div>"
              "<div class=rw><span class=n>Page</span><span class=v id=dP>--</span></div>"
              "<div class=rw><span class=n>Event | Risk</span><span class=v id=dE>--</span></div>"
              "<div class=rw><span class=n>Orient</span><span class=v id=dO>--</span></div>"
              "<div class=rw><span class=n>SW raw</span><span class=v id=dW>--</span></div>"
              "<div class=rw><span class=n>Cooldown</span><span class=v id=dC>--</span></div>"
              "<div class=rw><span class=n>Alarms | Peak</span><span class=v id=dM>--</span></div>"
              "</div>"
              "<div class=c><h3>Project</h3>"
              "<div style=font-family:var(--m);font-size:11px;color:var(--mu2);line-height:1.9>"
              "<b>FMEAD-FPGA</b> Edge Anomaly Detector<br>"
              "Nexys 4 DDR | Artix-7 | MicroBlaze<br>"
              "ADXL362 + BME280 + XADC + PDM mic<br>"
              "ESP32 AP mode | /data JSON<br>"
              "</div></div></div></div>"
              "<div class=ft>FMEAD-FPGA | ESP32 | MicroBlaze edge telemetry</div></div>");

    /* JS */
    p = as(p, "<script>(function(){"
              "var H1=[],H2=[],H3=[],MAX=60,T0=Date.now(),U=0,L=Date.now(),STL=15000,cur='home',busy=0,fails=0,pendingPg='',ptmr=0;"
              "function $(i){return document.getElementById(i)}"
              "function tx(i,v){var e=$(i);if(e&&e.textContent!==String(v))e.textContent=v}"
              "function cl(i,c){var e=$(i);if(e)e.className=c}"
              "function n(v,d){if(v==null)return d;var x=+v;return isFinite(x)?x:d}"
              "function s(v,d){return v==null?d:String(v)}"
              "function rk(r){return r=='CRITICAL'?'cr':r=='HIGH'?'hi':r=='MEDIUM'?'md':'lo'}"
              "function ac(st){return st=='CLAP / IMPACT'?'p':'q'}"
              "function pc(v){var p=Math.round(v*100/999);return p<0?0:p>100?100:p}"
              "function fT(sec){var m=Math.floor(sec/60),s=sec%60;return(m<10?'0':'')+m+':'+(s<10?'0':'')+s}"
              "function bSw(){var h='';for(var i=0;i<16;i++){var rsv=i>1,nm=i==0?'Board':i==1?'LED bar':'Reserved';"
              "h+='<div class=\"swc'+(rsv?' r':'')+'\" id=w'+i+'><div><div class=n>SW'+i+'</div>'+"
              "'<div style=color:var(--mu);font-size:9px;margin-top:2px>'+nm+'</div></div><span class=s>--</span></div>'}"
              "$('swg').innerHTML=h}"
              "function uSw(sw){for(var i=0;i<16;i++){var on=(sw>>i)&1,c=$('w'+i),rsv=i>1;if(!c)continue;"
              "c.classList.toggle('o',!!on&&!rsv);c.querySelector('.s').textContent=rsv?'RSV':(on?'ON':'OFF')}}"
              "var tbs=document.querySelectorAll('.tb');for(var i=0;i<tbs.length;i++){"
              "tbs[i].addEventListener('click',function(e){var pg=e.currentTarget.getAttribute('data-p');"
              "swT(pg);queuePg(pg)})}"
              "function done(){busy=0;if(pendingPg)setTimeout(sendPg,120)}"
              "function sendPg(){if(busy||!pendingPg)return;var pg=pendingPg;pendingPg='';busy=1;"
              "fetch('/p/'+pg,{cache:'no-store'}).then(function(){done()},function(){done()})}"
              "function queuePg(pg){pendingPg=pg;clearTimeout(ptmr);ptmr=setTimeout(sendPg,180)}"
              "function swT(pg){cur=pg;var ts=document.querySelectorAll('.tb');"
              "for(var i=0;i<ts.length;i++)ts[i].classList.toggle('on',ts[i].getAttribute('data-p')===pg);"
              "var ps=document.querySelectorAll('.pg');"
              "for(var j=0;j<ps.length;j++)ps[j].classList.toggle('on',ps[j].id==='P_'+pg);"
              "tx('Sp',pg.toUpperCase());setTimeout(dr,30)}"
              "function gr(c,w,h,lo,hi){"
              "c.strokeStyle='#1a2233';c.lineWidth=1;"
              "for(var i=1;i<5;i++){var y=i*h/5;c.beginPath();c.moveTo(0,y);c.lineTo(w,y);c.stroke()}"
              "for(var j=1;j<6;j++){var x=j*w/6;c.beginPath();c.moveTo(x,0);c.lineTo(x,h);c.stroke()}"
              "if(lo<0&&hi>0){var r=hi-lo,y0=h-((0-lo)/r)*h*.9-h*.05;"
              "c.strokeStyle='#3a4660';c.lineWidth=1.5;"
              "c.beginPath();c.moveTo(0,y0);c.lineTo(w,y0);c.stroke()}}"
              "function ln(c,d,k,lo,hi,w,h,col,fillCol){if(d.length<2)return;"
              "var r=hi-lo||1,lx=0,ly=0,pts=[];"
              "for(var i=0;i<d.length;i++){var x=i*w/(MAX-1),v=n(d[i][k],0),"
              "y=h-((v-lo)/r)*h*.9-h*.05;pts.push([x,y]);lx=x;ly=y}"
              "if(fillCol){c.fillStyle=fillCol;c.beginPath();"
              "c.moveTo(pts[0][0],h);for(var i=0;i<pts.length;i++)c.lineTo(pts[i][0],pts[i][1]);"
              "c.lineTo(pts[pts.length-1][0],h);c.closePath();c.fill()}"
              "c.strokeStyle=col;c.lineWidth=2;c.lineJoin='round';c.lineCap='round';"
              "c.beginPath();c.moveTo(pts[0][0],pts[0][1]);"
              "for(var i=1;i<pts.length;i++)c.lineTo(pts[i][0],pts[i][1]);c.stroke();"
              "c.fillStyle=col;c.beginPath();c.arc(lx,ly,3.5,0,6.283);c.fill();"
              "c.fillStyle='#0a1018';c.beginPath();c.arc(lx,ly,1.5,0,6.283);c.fill()}"
              "function lb(c,t,x,y){c.fillStyle='#6a7a90';c.font='10px monospace';c.fillText(t,x,y)}"
              "function hx(col,a){var r=parseInt(col.substr(1,2),16),g=parseInt(col.substr(3,2),16),"
              "b=parseInt(col.substr(5,2),16);return 'rgba('+r+','+g+','+b+','+a+')'}"
              "function dC(id,d,ks,cs,mn,mx){var c=$(id);if(!c||d.length<2)return;"
              "var x=c.getContext('2d'),w=c.clientWidth,h=c.clientHeight;c.width=w;c.height=h;"
              "x.clearRect(0,0,w,h);var lo=mn,hi=mx;"
              "if(lo==null||hi==null){var v=[];d.forEach(function(o){ks.forEach(function(k){v.push(n(o[k],0))})});"
              "lo=Math.min.apply(null,v);hi=Math.max.apply(null,v);"
              "if(lo===hi){lo--;hi++}var pad=(hi-lo)*0.1;lo-=pad;hi+=pad}"
              "gr(x,w,h,lo,hi);"
              "ks.forEach(function(k,i){ln(x,d,k,lo,hi,w,h,cs[i],hx(cs[i],.08))});"
              "lb(x,String(Math.round(hi)),4,12);lb(x,String(Math.round(lo)),4,h-4);"
              "if(d.length>0){var last=d[d.length-1];ks.forEach(function(k,i){"
              "var v=n(last[k],0);x.fillStyle=cs[i];x.font='bold 10px monospace';"
              "x.fillText(String(Math.round(v)),w-32,14+i*12)})}}"
              "function dr(){if(cur=='motion')dC('c1',H1,['x','y','z'],['#3ec6ff','#22d97f','#ffb02e'],-1200,1200);"
              "if(cur=='environment')dC('c2',H2,['bt','ft'],['#3ec6ff','#ff8a47']);"
              "if(cur=='acoustic')dC('c3',H3,['me','ms','md'],['#3ec6ff','#9d6bff','#ffb02e'])}"
              "function up(j){"
              "H1.push({x:n(j.x,0),y:n(j.y,0),z:n(j.z,0)});if(H1.length>MAX)H1.shift();"
              "H2.push({bt:parseFloat(j.bt)||0,ft:parseFloat(j.ftc)||0});if(H2.length>MAX)H2.shift();"
              "H3.push({me:n(j.me,0),ms:n(j.ms,0),md:n(j.md,0)});if(H3.length>MAX)H3.shift();"
              "var sw=n(j.sw,0),sw0=sw&1,sw1=(sw>>1)&1,off=!sw0;document.body.classList.toggle('off',off);"
              "var home_ev=s(j.ev,'NORMAL'),home_r=s(j.rk,'LOW'),"
              "mev=s(j.mev,'NORMAL'),mrk=s(j.mrk,'LOW'),"
              "eev=s(j.eev,'NORMAL'),erk=s(j.erk,'LOW'),"
              "sc=n(j.sc,0),mx=n(j.mx,0),al=n(j.al,0),dt=n(j.dt,0),cd=n(j.cd,0),or_=s(j.or,'--');"
              "var bt=j.bt!=null?j.bt:'--',bp=j.bp!=null?j.bp:'--',bh=j.bh!=null?j.bh:'--',ft=j.ftc!=null?j.ftc:'--';"
              "var aok=n(j.aok,1),bok=n(j.bok,1),xok=n(j.xok,1),mok=n(j.mok,1),ol=n(j.ol,0);"
              "var mr=n(j.mr,0),md=n(j.md,0),me=n(j.me,0),ms=n(j.ms,0),mb=n(j.mb,0),"
              "as_=s(j.as,'QUIET'),aev=s(j.aev,as_),ark=s(j.ark,'LOW');"
              "var ev=cur=='motion'?mev:cur=='environment'?eev:cur=='acoustic'?aev:home_ev,"
              "r=cur=='motion'?mrk:cur=='environment'?erk:cur=='acoustic'?ark:home_r,"
              "rc=rk(r),home_rc=rk(home_r);"
              "tx('Ss',off?'OFF':'ENABLED');cl('Ss','v '+(off?'cr':'lo'));"
              "tx('Se',off?'--':ev);cl('Se','v '+(off?'of':rc));"
              "tx('Sr',off?'--':r);cl('Sr','v '+(off?'of':rc));"
              "tx('S0',sw0?'ON':'OFF');cl('S0','v '+(sw0?'lo':'cr'));"
              "tx('S1',sw1?'ON':'OFF');cl('S1','v '+(sw1?'lo':'of'));"
              "if(!off){"
              "tx('He',home_ev);cl('He','en '+home_rc);cl('Hh','c eh '+home_rc);"
              "tx('Hr',home_r+' RISK');cl('Hr','bg '+home_rc);"
              "tx('Ha',al);tx('Hm',mx);tx('Ho',or_);"
              "var p=pc(sc);$('Hb').style.width=p+'%';tx('Hp',p+'%');"
              "tx('V1',aok?'READY':'OFF');cl('V1','v '+(aok?'ok':'bd'));cl('L1','ld '+(aok?'ok':'bd'));"
              "tx('V2',bok&&xok?'READY':'PART');cl('V2','v '+(bok&&xok?'ok':'wn'));cl('L2','ld '+(bok&&xok?'ok':'wn'));"
              "tx('V3',mok?'READY':'OFF');cl('V3','v '+(mok?'ok':'bd'));cl('L3','ld '+(mok?'ok':'bd'));"
              "tx('Hs',sc);tx('Hbt',bt);tx('Hft',ft);"
              "tx('Hac',as_);cl('Hac','ap '+ac(as_));tx('Hms','env='+me);"
              "tx('mX',n(j.x,0));tx('mY',n(j.y,0));tx('mZ',n(j.z,0));tx('mO',or_);"
              "tx('mS',sc);tx('mD',dt);tx('mE',mev);tx('mA',al);tx('mM',mx);$('mB').style.width=pc(sc)+'%';"
              "tx('eT',bt);tx('eP',bp);tx('eH',bh);tx('eF',ft);"
              "tx('eVb',bok?'OK':'FAIL');cl('eLb','ld '+(bok?'ok':'bd'));cl('eVb','v '+(bok?'ok':'bd'));"
              "tx('eVx',xok?'OK':'FAIL');cl('eLx','ld '+(xok?'ok':'bd'));cl('eVx','v '+(xok?'ok':'bd'));"
              "tx('eBa',eev);cl('eBa','v '+rk(erk));tx('eTd',erk);cl('eTd','v '+rk(erk));"
              "tx('aE',aev);var c=ac(as_),ae=rk(ark);cl('aE','en '+ae);"
              "tx('aP',as_);cl('aP','ap '+c);"
              "tx('aMe',me);tx('aMs',ms);tx('aMb',mb);tx('aMr',mr);tx('aMd',md);tx('aMe2',me);tx('aMb2',mb);tx('aSt',as_);"
              "var ap=pc(ms);$('aB').style.width=ap+'%';tx('aPc',ap+'%');}"
              "uSw(sw);"
              "tx('hVa',aok?'READY':'OFF');cl('hLa','ld '+(aok?'ok':'bd'));cl('hVa','v '+(aok?'ok':'bd'));"
              "tx('hVb',bok?'READY':'OFF');cl('hLb','ld '+(bok?'ok':'bd'));cl('hVb','v '+(bok?'ok':'bd'));"
              "tx('hVx',xok?'READY':'OFF');cl('hLx','ld '+(xok?'ok':'bd'));cl('hVx','v '+(xok?'ok':'bd'));"
              "tx('hVm',mok?'READY':'OFF');cl('hLm','ld '+(mok?'ok':'bd'));cl('hVm','v '+(mok?'ok':'bd'));"
              "tx('hVo',ol?'READY':'PENDING');cl('hLo','ld '+(ol?'ok':'wn'));cl('hVo','v '+(ol?'ok':'wn'));"
              "tx('dS',off?'OFF':'ENABLED');tx('dP',cur.toUpperCase());"
              "tx('dE',ev+' | '+r);tx('dO',or_);tx('dW','0x'+sw.toString(16).toUpperCase());"
              "tx('dC',cd);tx('dM',al+' | '+mx);if(!off)dr()}"
              "function sL(c,t){cl('lnk','lk '+c);tx('lst',t)}"
              "function tk(){$('app').classList.toggle('st',Date.now()-L>STL);"
              "tx('uupt',fT(Math.floor((Date.now()-T0)/1000)))}"
              "function pl(){if(busy)return;busy=1;"
              "var t=Date.now(),ct=null,opt={cache:'no-store'};"
              "if(typeof AbortController!=='undefined'){ct=new AbortController();opt.signal=ct.signal;"
              "setTimeout(function(){try{ct.abort()}catch(e){}},8000)}"
              "fetch('/data',opt).then(function(r){if(!r.ok)throw 0;return r.json()})"
              ".then(function(j){fails=0;L=Date.now();U++;tx('ucnt',U);tx('ulat',(Date.now()-t)+'ms');sL('','ONLINE');up(j);done()})"
              ".catch(function(){fails++;var a=Date.now()-L;"
              "sL(a>STL?'b':'w',a>STL?'LOST':(fails>2?'UNSTABLE':'ONLINE'));done()})}"
              "bSw();setInterval(tk,500);setInterval(pl,2500);setTimeout(pl,300);})();</script></body></html>");

    return page_len(p);
}

static int build_cmd_response(void)
{
    char *p = page;
    p = as(p,
        "HTTP/1.1 204 No Content\r\n"
        "Cache-Control:no-store\r\n"
        "Connection:close\r\n\r\n");
    return page_len(p);
}

/* ============================================================ */
/* JSON data route                                              */
/* ============================================================ */

static int build_data_json(void)
{
    char *p = page;
    const char *orient = orientation_name(g_x, g_y, g_z);
    const char *page_event = g_event;
    const char *page_risk = g_risk;

    if (bme280_ok) bme280_read_measurement();
    update_environment_status();
    update_overall_status();

    if (g_web_page == WEB_PAGE_MOTION) {
        page_event = g_motion_event; page_risk = g_motion_risk;
    } else if (g_web_page == WEB_PAGE_ENVIRONMENT) {
        page_event = g_env_event; page_risk = g_env_risk;
    } else if (g_web_page == WEB_PAGE_ACOUSTIC) {
        page_event = g_acoustic_event; page_risk = g_acoustic_risk;
    }

    p = as(p,
        "HTTP/1.1 200 OK\r\n"
        "Content-Type:application/json\r\n"
        "Cache-Control:no-store\r\n"
        "Connection:close\r\n\r\n"
        "{\"x\":");

    p = ai(p, g_x);
    p = as(p, ",\"y\":");   p = ai(p, g_y);
    p = as(p, ",\"z\":");   p = ai(p, g_z);
    p = as(p, ",\"sc\":");  p = ai(p, g_score);
    p = as(p, ",\"dt\":");  p = ai(p, g_delta);
    p = as(p, ",\"ev\":\"");  p = as(p, page_event);
    p = as(p, "\",\"rk\":\""); p = as(p, page_risk);
    p = as(p, "\",\"mev\":\""); p = as(p, g_motion_event);
    p = as(p, "\",\"mrk\":\""); p = as(p, g_motion_risk);
    p = as(p, "\",\"eev\":\""); p = as(p, g_env_event);
    p = as(p, "\",\"erk\":\""); p = as(p, g_env_risk);
    p = as(p, "\",\"aev\":\""); p = as(p, g_acoustic_event);
    p = as(p, "\",\"ark\":\""); p = as(p, g_acoustic_risk);
    p = as(p, "\",\"or\":\"");  p = as(p, orient);
    p = as(p, "\",\"al\":");    p = ai(p, g_alarm_count);
    p = as(p, ",\"mx\":");      p = ai(p, g_max_score);
    p = as(p, ",\"cd\":");      p = ai(p, g_cooldown);
    p = as(p, ",\"sw\":");      p = au(p, (unsigned int)g_switches);
    p = as(p, ",\"ol\":");      p = as(p, oled_hw_ready ? "1" : "0");
    p = as(p, ",\"aok\":");     p = as(p, adxl_ok ? "1" : "0");
    p = as(p, ",\"bok\":");     p = as(p, bme280_ok ? "1" : "0");
    p = as(p, ",\"ba\":");      p = au(p, (unsigned int)bme280_addr);
    p = as(p, ",\"bt\":");      p = af2(p, g_temp_centi);
    p = as(p, ",\"bp\":");      p = au(p, g_press_pa / 100U);
    p = as(p, ",\"bh\":");      p = af2(p, g_hum_centi);
    p = as(p, ",\"ftc\":");     p = af2(p, g_fpga_temp_centi);
    p = as(p, ",\"xok\":");     p = as(p, xadc_ok ? "1" : "0");
    p = as(p, ",\"pok\":");     p = as(p, pcm_ok ? "1" : "0");
    p = as(p, ",\"mok\":");     p = as(p, (pcm_ok && g_mic_ready) ? "1" : "0");

    /* Acoustic UI mirrors */
    p = as(p, ",\"mr\":");  p = ai(p, g_mic_raw);
    p = as(p, ",\"md\":");  p = ai(p, g_mic_diff);
    p = as(p, ",\"me\":");  p = ai(p, g_mic_envelope);
    p = as(p, ",\"ms\":");  p = ai(p, g_mic_score);
    p = as(p, ",\"mb\":");  p = ai(p, g_mic_baseline);
    p = as(p, ",\"as\":\""); p = as(p, g_acoustic_state); p = as(p, "\"");
    p = as(p, "}");

    return page_len(p);
}

/* ============================================================ */
/* HTTP receive (parse +IPD, GET path)                          */
/* ============================================================ */

static int wait_get(int *cid, int *request_type)
{
    unsigned int ipd_state = 0;
    unsigned int get_state = 0;
    int wait_id = 0;
    int got_id = 0;
    int id = 0;
    int after_get = 0;
    int path_len = 0;
    char path[32];
    u8 c;

    *request_type = 0;

    while (1) {
        detector_update();
        service_local_outputs();
        oled_dashboard_update();

        if (esp_rx(&c)) {
            if (match_token("+IPD,", (char)c, &ipd_state)) {
                xil_printf("HTTP request received\r\n");
                wait_id = 1;
                got_id = 0;
                get_state = 0;
                after_get = 0;
                path_len = 0;
                path[0] = '\0';
                *request_type = 0;
                continue;
            }

            if (wait_id) {
                if (c >= '0' && c <= '9') {
                    id = c - '0';
                    got_id = 1;
                    wait_id = 0;
                }
                continue;
            }

            if (got_id && !after_get) {
                if (match_token("GET ", (char)c, &get_state)) {
                    after_get = 1;
                    path_len = 0;
                    path[0] = '\0';
                    continue;
                }
            }

            if (after_get) {
                if (c == ' ') {
                    path[path_len] = '\0';
                    *cid = id;

                    if (path[0] == '/' && path[1] == '\0') {
                        *request_type = 0;
                    } else if (path[0] == '/' && path[1] == 'd' && path[2] == 'a' &&
                        path[3] == 't' && path[4] == 'a') {
                        *request_type = 1;
                    } else if (path[0] == '/' && path[1] == 'p' && path[2] == '/') {
                        *request_type = 2;
                        if      (path[3] == 'h') g_web_page = WEB_PAGE_HOME;
                        else if (path[3] == 'm') g_web_page = WEB_PAGE_MOTION;
                        else if (path[3] == 'e') g_web_page = WEB_PAGE_ENVIRONMENT;
                        else if (path[3] == 'a') g_web_page = WEB_PAGE_ACOUSTIC;
                        else if (path[3] == 's') g_web_page = WEB_PAGE_SYSTEM;
                        xil_printf("7SEG page set from web: %d\r\n", g_web_page);
                    } else if (path[0] == '/' && path[1] == 'f' && path[2] == 'a' &&
                               path[3] == 'v') {
                        *request_type = 3;
                    } else {
                        *request_type = 3;
                    }
                    return 1;
                }

                if (path_len < ((int)sizeof(path) - 1)) {
                    path[path_len++] = (char)c;
                    path[path_len] = '\0';
                }
            }
        } else {
            service_delay_ms(1);
        }
    }
}

/* ============================================================ */
/* HTTP send                                                    */
/* ============================================================ */

static int send_chunk(int cid, const char *buf, int len)
{
    char cmd[40];
    char *p = cmd;

    /* Build AT+CIPSEND command into a private local buffer       */
    /* so the page response buffer remains untouched.             */
    *p++ = 'A'; *p++ = 'T'; *p++ = '+'; *p++ = 'C'; *p++ = 'I';
    *p++ = 'P'; *p++ = 'S'; *p++ = 'E'; *p++ = 'N'; *p++ = 'D';
    *p++ = '=';
    *p++ = (char)('0' + cid);
    *p++ = ',';
    {
        char t[12]; int i = 0; unsigned int v = (unsigned int)len;
        if (!v) { *p++ = '0'; }
        else {
            while (v) { t[i++] = (char)('0' + (v % 10U)); v /= 10U; }
            while (i > 0) *p++ = t[--i];
        }
    }
    *p++ = '\r'; *p++ = '\n'; *p = '\0';

    esp_tx(cmd);
    if (!esp_wait(">", 3000)) {
        xil_printf("HTTP SEND PROMPT FAIL\r\n");
        return 0;
    }

    esp_txbuf((u8 *)buf, (unsigned int)len);
    if (!esp_wait("SEND OK", 7000)) {
        xil_printf("HTTP SEND FAIL\r\n");
        return 0;
    }

    service_delay_ms(25);
    return 1;
}

static int send_response(int cid, const char *buf, int len)
{
    int sent = 0;
    int chunk;

    while (sent < len) {
        chunk = len - sent;
        if (chunk > ESP_SEND_CHUNK) chunk = ESP_SEND_CHUNK;
        if (!send_chunk(cid, buf + sent, chunk)) return 0;
        sent += chunk;
        service_local_outputs();
    }
    return 1;
}

static void close_conn(int cid)
{
    char cmd[24];
    char *p = cmd;
    const char *prefix = "AT+CIPCLOSE=";

    while (*prefix) *p++ = *prefix++;
    *p++ = (char)('0' + cid);
    *p++ = '\r'; *p++ = '\n'; *p = '\0';

    esp_tx(cmd);
    esp_wait("OK", 1500);
}

static void handle(int cid, int request_type)
{
    int len;

    if (request_type != 2 && request_type != 3) {
        detector_update();
    }

    if (request_type == 1) {
        len = build_data_json();
        xil_printf("Serving /data (%d bytes)\r\n", len);
    } else if (request_type == 2) {
        len = build_cmd_response();
        xil_printf("Serving page command\r\n");
    } else if (request_type == 3) {
        len = build_cmd_response();
        xil_printf("Serving small ignore response\r\n");
    } else {
        len = build_index_page();
        xil_printf("Serving / (%d bytes)\r\n", len);
    }

    service_delay_ms(20);

    if (send_response(cid, page, len)) {
        xil_printf("Response OK\r\n");
    } else {
        xil_printf("Response FAIL\r\n");
    }

    service_delay_ms(50);
    close_conn(cid);
}

/* ============================================================ */
/* Main                                                         */
/* ============================================================ */

int main(void)
{
    int cid;
    int request_type;

    xil_printf("\r\n");
    xil_printf("============================================\r\n");
    xil_printf(" FMEAD-FPGA: Multi-Modal Edge Anomaly Detector\r\n");
    xil_printf(" Nexys 4 DDR  |  MicroBlaze bare-metal\r\n");
    xil_printf(" Author: Amilton Koxi\r\n");
    xil_printf(" University of Debrecen\r\n");
    xil_printf("============================================\r\n");

    if (gpio_init_all() != XST_SUCCESS) {
        xil_printf("SYSTEM FAIL: GPIO\r\n");
        while (1) { }
    }

    oled_dashboard_init();

    adxl_ok = adxl_init();
    if (adxl_ok) calibrate_baseline();
    else xil_printf("SYSTEM WARN: ADXL OFFLINE\r\n");

    bme280_init();

    xadc_ok = xadc_init();
    if (!xadc_ok) xil_printf("SYSTEM WARN: XADC OFFLINE\r\n");

    xil_printf("Controls: SW0 board ON/OFF | SW1 LED bar\r\n");
    xil_printf("Buttons: BTNC recalibrate | BTNR reset alarms\r\n");
    if (pcm_ok) xil_printf("PCM GPIO: OK\r\n");
    else xil_printf("PCM GPIO: OFFLINE\r\n");
    xil_printf("PCM GPIO base: 0x%08X\r\n", XPAR_AXI_GPIO_PCM_BASEADDR);
    xil_printf("HTTP page buffer: %d bytes\r\n", PAGE_BUF_SIZE);

    if (!esp_init()) {
        xil_printf("SYSTEM FAIL: ESP32\r\n");
        while (1) {
            detector_update();
            service_delay_ms(1000);
        }
    }

    xil_printf("SYSTEM READY\r\n");

    while (1) {
        if (wait_get(&cid, &request_type)) handle(cid, request_type);
    }

    return 0;
}
