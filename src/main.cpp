#include "mbed.h"
#include "arm_math.h"
using namespace std::chrono_literals;

/*************************************
 *  B-L475E-IOT01A | LSM6DSL I2C2    *
 *  Tremor / Dyskinesia Detector     *
 *  Enhanced LED Feedback Version    *
 *************************************/

/*********** ───── CONFIG BLOCK ───── ***********/
#define DEBUG_RAW_EVERY    50   // Print raw sample every N ticks (0 = off)
#define DEBUG_FFT_SUMMARY   1   // 1: print one-line summary per channel
#define DEBUG_THRESH_MSG    1   // 1: show decision variables each window

static float ACC_T_TH    = 0.10f;  // 保持加速度计震颤阈值
static float ACC_D_TH    = 0.10f;  // 保持加速度计运动障碍阈值
static float GYR_T_TH    = 10.0f;  // 保持陀螺仪震颤阈值
static float GYR_D_TH    = 10.0f;  // 保持陀螺仪运动障碍阈值
static float PEAK_TO_RMS = 1.5f;   // 保持峰值/RMS比值阈值

// 添加稳定性判断参数
static const int STABLE_WINDOWS = 1;  // 保持只需要一个窗口
static int stable_tremor = 0;         // 震颤稳定计数器
static int stable_dyskinesia = 0;     // 运动障碍稳定计数器

// 添加基线校准参数
static const int CALIBRATION_WINDOWS = 5;   // 减少校准窗口数
static float baseline_acc[3] = {0};        // 加速度计基线
static float baseline_gyr[3] = {0};        // 陀螺仪基线
static bool is_calibrated = false;         // 校准状态

/*********** Debug serial ***********/
UnbufferedSerial pc(USBTX, USBRX, 115200);

/*********** I²C2 wiring (PB11 SDA, PB10 SCL) ***********/
I2C i2c(PB_11, PB_10);
int LSM_ADDR;

/*********** LSM6DSL registers ***********/
constexpr uint8_t WHO_AM_I = 0x0F;
constexpr uint8_t CTRL1_XL  = 0x10;
constexpr uint8_t CTRL2_G   = 0x11;
constexpr uint8_t CTRL3_C   = 0x12;
constexpr uint8_t OUT_G_L   = 0x22;
constexpr uint8_t OUT_XL_L  = 0x28;

/*********** Algorithm parameters ***********/
constexpr uint32_t Fs    = 104;
constexpr uint32_t WIN_S = 1;    // 减少窗口大小从 3 秒到 1 秒
constexpr size_t   N     = Fs * WIN_S;
constexpr size_t   FFTN  = 256;  // 减少 FFT 点数以提高处理速度

/*********** LED outputs ***********/
PwmOut led_tremor(PA_5);      // LD2 (绿色) - 震颤指示
PwmOut led_dyskinesia(PC_9);  // LD4 (蓝色) - 运动障碍指示
DigitalOut led_status(PB_14);  // LD3 (红色) - 系统状态指示
DigitalOut led_power(PA_8);    // LD5 (红色) - 电源/错误指示

/*********** Data buffers ***********/
static float ax[FFTN], ay[FFTN], az[FFTN];
static float gx[FFTN], gy[FFTN], gz[FFTN];

/*********** Sampling ticker ***********/
static volatile bool tick_flag = false;
Ticker tick;
void isr() { tick_flag = true; }

/*********** I²C helpers ***********/
inline void w8(uint8_t reg, uint8_t val) {
    char b[2] = { (char)reg, (char)val };
    i2c.write(LSM_ADDR, b, 2);
}

inline uint8_t r8(uint8_t reg) {
    uint8_t v;
    i2c.write(LSM_ADDR, (char *)&reg, 1, true);
    i2c.read(LSM_ADDR, (char *)&v, 1);
    return v;
}

inline int rN(uint8_t reg, uint8_t *dst) {
    i2c.write(LSM_ADDR, (char *)&reg, 1, true);
    return i2c.read(LSM_ADDR, (char *)dst, 6);
}

/*********** Logging helper ***********/
void logf(const char *fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    pc.write(buf, n);
}

int main() {
    // Boot message
    pc.write("Boot\r\n", 6);

    // I2C setup
    i2c.frequency(400000);

    // Auto-detect sensor address
    for (int addr : {0x6A, 0x6B}) {
        LSM_ADDR = addr << 1;
        if (r8(WHO_AM_I) == 0x6A) {
            logf("Found LSM6DSL at 0x%02X\r\n", addr);
            break;
        }
    }

    // Sensor configuration
    w8(CTRL1_XL, 0x40); // 104 Hz, ±2g
    w8(CTRL2_G,  0x40); // 104 Hz, 245 dps
    w8(CTRL3_C,  0x44); // IF_INC=1, BDU=1
    logf("CTRL1=%02X CTRL2=%02X CTRL3=%02X\r\n",
         r8(CTRL1_XL), r8(CTRL2_G), r8(CTRL3_C));

    // LED setup
    led_tremor.period_ms(1);
    led_dyskinesia.period_ms(1);
    led_status = 0;  // 使用数字输出
    led_power = 0;   // 使用数字输出

    // 初始 LED 状态 - 全部关闭
    led_tremor.write(0);
    led_dyskinesia.write(0);
    led_status = 0;
    led_power = 0;

    // FFT initialization
    arm_rfft_fast_instance_f32 fft;
    arm_rfft_fast_init_f32(&fft, FFTN);
    float fbuf[FFTN], mag[FFTN / 2];
    int i3 = roundf(3.0f * FFTN / Fs);
    int i5 = roundf(5.0f * FFTN / Fs);
    int i7 = roundf(7.0f * FFTN / Fs);
    logf("Freq bins: i3=%d i5=%d i7=%d\r\n", i3, i5, i7);

    // Attach ticker
    tick.attach(&isr, 9600us);

    // 添加校准过程
    logf("Starting calibration...\r\n");
    for (int i = 0; i < CALIBRATION_WINDOWS; i++) {
        size_t idx = 0;
        while (idx < N) {
            if (!tick_flag) continue;
            tick_flag = false;

            uint8_t data[6];
            
            // 收集加速度计数据
            if (rN(OUT_XL_L, data) != 0) continue;
            baseline_acc[0] += (data[0] | (data[1] << 8)) * 0.000061f;
            baseline_acc[1] += (data[2] | (data[3] << 8)) * 0.000061f;
            baseline_acc[2] += (data[4] | (data[5] << 8)) * 0.000061f;

            // 收集陀螺仪数据
            if (rN(OUT_G_L, data) != 0) continue;
            baseline_gyr[0] += (data[0] | (data[1] << 8)) * 0.00875f;
            baseline_gyr[1] += (data[2] | (data[3] << 8)) * 0.00875f;
            baseline_gyr[2] += (data[4] | (data[5] << 8)) * 0.00875f;

            ++idx;
        }
        ThisThread::sleep_for(100ms);
    }

    // 计算平均值作为基线
    for (int i = 0; i < 3; i++) {
        baseline_acc[i] /= (CALIBRATION_WINDOWS * N);
        baseline_gyr[i] /= (CALIBRATION_WINDOWS * N);
    }
    is_calibrated = true;
    logf("Calibration complete. Baselines: ACC[%.3f, %.3f, %.3f] GYR[%.3f, %.3f, %.3f]\r\n",
         baseline_acc[0], baseline_acc[1], baseline_acc[2],
         baseline_gyr[0], baseline_gyr[1], baseline_gyr[2]);

    uint32_t windowCount = 0;
    while (true) {
        logf("--- Window %lu ---\r\n", ++windowCount);

        // Collect N samples
        size_t idx = 0;
        while (idx < N) {
            if (!tick_flag) continue;
            tick_flag = false;

            uint8_t data[6];

            // Accel
            if (rN(OUT_XL_L, data) != 0) continue;
            int16_t axr = data[0] | (data[1] << 8);
            int16_t ayr = data[2] | (data[3] << 8);
            int16_t azr = data[4] | (data[5] << 8);

            // Gyro
            if (rN(OUT_G_L, data) != 0) continue;
            int16_t gxr = data[0] | (data[1] << 8);
            int16_t gyr = data[2] | (data[3] << 8);
            int16_t gzr = data[4] | (data[5] << 8);

            // Debug raw
            if (DEBUG_RAW_EVERY && (idx % DEBUG_RAW_EVERY == 0)) {
                logf("RAW %d %d %d %d %d %d\r\n",
                     axr, ayr, azr, gxr, gyr, gzr);
            }

            // Scale and remove baseline
            ax[idx] = axr * 0.000061f - baseline_acc[0];
            ay[idx] = ayr * 0.000061f - baseline_acc[1];
            az[idx] = azr * 0.000061f - baseline_acc[2];
            gx[idx] = gxr * 0.00875f - baseline_gyr[0];
            gy[idx] = gyr * 0.00875f - baseline_gyr[1];
            gz[idx] = gzr * 0.00875f - baseline_gyr[2];

            ++idx;
        }

        // Zero-pad
        for (size_t i = N; i < FFTN; i++) {
            ax[i] = ay[i] = az[i] = gx[i] = gy[i] = gz[i] = 0;
        }

        // Analysis
        bool trem = false, dysk = false;
        float levelT = 0, levelD = 0;

        auto analyze = [&](float *d, float tth, float dth,
                           float scale, const char *tag) {
            // FFT
            arm_rfft_fast_f32(&fft, d, fbuf, 0);
            arm_cmplx_mag_f32(fbuf, mag, FFTN / 2);

            // Compute RMS
            float rms = 0;
            for (int k = 1; k < i7 + 3; k++) {
                rms += mag[k] * mag[k];
            }
            rms = sqrtf(rms / (i7 + 2));

            // Find peaks
            float p35 = 0, p57 = 0;
            int k35 = i3, k57 = i5;
            for (int k = i3; k <= i5; k++) {
                if (mag[k] > p35) { p35 = mag[k]; k35 = k; }
            }
            for (int k = i5; k <= i7; k++) {
                if (mag[k] > p57) { p57 = mag[k]; k57 = k; }
            }

            float f35 = k35 * (float)Fs / FFTN;
            float f57 = k57 * (float)Fs / FFTN;

            // Debug summary
            if (DEBUG_FFT_SUMMARY) {
                logf("%s 3-5 %.3f@%.1fHz 5-7 %.3f@%.1fHz rms %.3f\r\n",
                     tag, (double)p35, (double)f35,
                     (double)p57, (double)f57,
                     (double)rms);
            }

            // 改进的阈值逻辑
            if (p35 >= tth && p35 / rms > PEAK_TO_RMS && rms > tth * 0.5f) {
                trem = true;
                levelT = fmaxf(levelT, p35 / scale);
            }
            if (p57 >= dth && p57 / rms > PEAK_TO_RMS && rms > dth * 0.5f) {
                dysk = true;
                levelD = fmaxf(levelD, p57 / scale);
            }
        };

        analyze(ax, ACC_T_TH, ACC_D_TH, 0.5f, "AX");
        analyze(ay, ACC_T_TH, ACC_D_TH, 0.5f, "AY");
        analyze(az, ACC_T_TH, ACC_D_TH, 0.5f, "AZ");
        analyze(gx, GYR_T_TH, GYR_D_TH, 100.f, "GX");
        analyze(gy, GYR_T_TH, GYR_D_TH, 100.f, "GY");
        analyze(gz, GYR_T_TH, GYR_D_TH, 100.f, "GZ");

        levelT = fminf(levelT, 1.0f);
        levelD = fminf(levelD, 1.0f);

        if (DEBUG_THRESH_MSG) {
            logf("Decision T=%d(%.2f) D=%d(%.2f)\r\n",
                 trem, levelT, dysk, levelD);
        }

        // LED feedback
        bool show_tremor = false;
        bool show_dyskinesia = false;

        if (dysk && (!trem || levelD >= levelT)) {
            stable_dyskinesia++;
            stable_tremor = 0;
            if (stable_dyskinesia >= STABLE_WINDOWS) {
                show_dyskinesia = true;
            }
        } else if (trem) {
            stable_tremor++;
            stable_dyskinesia = 0;
            if (stable_tremor >= STABLE_WINDOWS) {
                show_tremor = true;
            }
        } else {
            stable_tremor = 0;
            stable_dyskinesia = 0;
        }

        // 重置所有 LED 状态
        led_tremor.write(0);
        led_dyskinesia.write(0);
        led_status = 0;
        led_power = 0;

        // 只在检测到运动时更新 LED
        if (show_dyskinesia) {
            // 运动障碍指示
            led_dyskinesia.period_ms(200);  // 5Hz
            led_dyskinesia.write(levelD);   // 亮度与强度成正比
        } else if (show_tremor) {
            // 震颤指示
            led_tremor.period_ms(500);      // 2Hz
            led_tremor.write(levelT);       // 亮度与强度成正比
        }

        // 更新状态 LED - 只在检测到运动时亮起
        if (show_tremor || show_dyskinesia) {
            led_status = 1;  // 使用数字输出
        } else {
            led_status = 0;
        }

        // 保持电源 LED 关闭
        led_power = 0;

        // 添加调试信息
        if (DEBUG_THRESH_MSG) {
            logf("Motion detected - Tremor: %d(%.2f) Dyskinesia: %d(%.2f)\r\n",
                 trem, levelT, dysk, levelD);
        }
    }
}
