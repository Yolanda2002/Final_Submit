#include "mbed.h"
#include "arm_math.h"
using namespace std::chrono_literals;

/*************************************
 *  B-L475E-IOT01A | LSM6DSL I2C2    *
 *  Tremor / Dyskinesia Detector     *
 *  Enhanced LED Feedback Version    *
 *************************************/

/*********** ───── 配置参数 ───── ***********/
#define DEBUG_RAW_EVERY    50   // 每50个采样点打印一次原始数据（0表示关闭）
#define DEBUG_FFT_SUMMARY   1   // 1: 每个通道打印一行FFT分析摘要
#define DEBUG_THRESH_MSG    1   // 1: 显示每个窗口的决策变量

// 检测阈值设置
static float ACC_T_TH    = 0.10f;  // 加速度计震颤检测阈值
static float ACC_D_TH    = 0.10f;  // 加速度计运动障碍检测阈值
static float GYR_T_TH    = 10.0f;  // 陀螺仪震颤检测阈值
static float GYR_D_TH    = 10.0f;  // 陀螺仪运动障碍检测阈值
static float PEAK_TO_RMS = 1.5f;   // 峰值与RMS比值阈值，用于判断信号质量

// 稳定性判断参数
static const int STABLE_WINDOWS = 1;  // 需要连续检测到症状的窗口数
static int stable_tremor = 0;         // 震颤稳定计数器
static int stable_dyskinesia = 0;     // 运动障碍稳定计数器

// 基线校准参数
static const int CALIBRATION_WINDOWS = 5;   // 校准窗口数
static float baseline_acc[3] = {0};        // 加速度计基线值
static float baseline_gyr[3] = {0};        // 陀螺仪基线值
static bool is_calibrated = false;         // 校准状态标志

/*********** 调试串口设置 ***********/
UnbufferedSerial pc(USBTX, USBRX, 115200);  // 创建串口对象，波特率115200

/*********** I²C2 接线定义 (PB11 SDA, PB10 SCL) ***********/
I2C i2c(PB_11, PB_10);  // 创建I2C对象
int LSM_ADDR;           // LSM6DSL传感器地址

/*********** LSM6DSL 寄存器地址定义 ***********/
constexpr uint8_t WHO_AM_I = 0x0F;  // 器件ID寄存器
constexpr uint8_t CTRL1_XL  = 0x10;  // 加速度计控制寄存器
constexpr uint8_t CTRL2_G   = 0x11;  // 陀螺仪控制寄存器
constexpr uint8_t CTRL3_C   = 0x12;  // 通用控制寄存器
constexpr uint8_t OUT_G_L   = 0x22;  // 陀螺仪数据输出寄存器（低字节）
constexpr uint8_t OUT_XL_L  = 0x28;  // 加速度计数据输出寄存器（低字节）

/*********** 算法参数设置 ***********/
constexpr uint32_t Fs    = 104;     // 采样频率（Hz）
constexpr uint32_t WIN_S = 1;       // 窗口大小（秒）
constexpr size_t   N     = Fs * WIN_S;  // 每个窗口的采样点数
constexpr size_t   FFTN  = 256;     // FFT点数

/*********** LED输出定义 ***********/
PwmOut led_tremor(PA_5);      // LD2 (绿色) - 震颤指示LED
PwmOut led_dyskinesia(PC_9);  // LD4 (蓝色) - 运动障碍指示LED
DigitalOut led_status(PB_14);  // LD3 (红色) - 系统状态指示LED
DigitalOut led_power(PA_8);    // LD5 (红色) - 电源/错误指示LED

/*********** 数据缓冲区 ***********/
static float ax[FFTN], ay[FFTN], az[FFTN];  // 加速度计数据缓冲区
static float gx[FFTN], gy[FFTN], gz[FFTN];  // 陀螺仪数据缓冲区

/*********** 采样定时器设置 ***********/
static volatile bool tick_flag = false;  // 采样标志
Ticker tick;                            // 定时器对象
void isr() { tick_flag = true; }        // 定时器中断服务函数

/*********** I²C 辅助函数 ***********/
// 写入一个字节到指定寄存器
inline void w8(uint8_t reg, uint8_t val) {
    char b[2] = { (char)reg, (char)val };
    i2c.write(LSM_ADDR, b, 2);
}

// 从指定寄存器读取一个字节
inline uint8_t r8(uint8_t reg) {
    uint8_t v;
    i2c.write(LSM_ADDR, (char *)&reg, 1, true);
    i2c.read(LSM_ADDR, (char *)&v, 1);
    return v;
}

// 从指定寄存器读取多个字节
inline int rN(uint8_t reg, uint8_t *dst) {
    i2c.write(LSM_ADDR, (char *)&reg, 1, true);
    return i2c.read(LSM_ADDR, (char *)dst, 6);
}

/*********** 日志辅助函数 ***********/
void logf(const char *fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    pc.write(buf, n);
}

// 主函数
int main() {
    // 启动消息
    pc.write("Boot\r\n", 6);

    // I2C初始化
    i2c.frequency(400000);  // 设置I2C频率为400kHz

    // 自动检测传感器地址
    for (int addr : {0x6A, 0x6B}) {
        LSM_ADDR = addr << 1;
        if (r8(WHO_AM_I) == 0x6A) {
            logf("Found LSM6DSL at 0x%02X\r\n", addr);
            break;
        }
    }

    // 传感器配置
    w8(CTRL1_XL, 0x40);  // 设置加速度计：104Hz采样率，±2g量程
    w8(CTRL2_G,  0x40);  // 设置陀螺仪：104Hz采样率，245dps量程
    w8(CTRL3_C,  0x44);  // 设置通用控制：IF_INC=1, BDU=1
    logf("CTRL1=%02X CTRL2=%02X CTRL3=%02X\r\n",
         r8(CTRL1_XL), r8(CTRL2_G), r8(CTRL3_C));

    // LED初始化
    led_tremor.period_ms(1);
    led_dyskinesia.period_ms(1);
    led_status = 0;  // 使用数字输出
    led_power = 0;   // 使用数字输出

    // 初始LED状态 - 全部关闭
    led_tremor.write(0);
    led_dyskinesia.write(0);
    led_status = 0;
    led_power = 0;

    // FFT初始化
    arm_rfft_fast_instance_f32 fft;
    arm_rfft_fast_init_f32(&fft, FFTN);
    float fbuf[FFTN], mag[FFTN / 2];
    int i3 = roundf(3.0f * FFTN / Fs);  // 3Hz对应的FFT bin
    int i5 = roundf(5.0f * FFTN / Fs);  // 5Hz对应的FFT bin
    int i7 = roundf(7.0f * FFTN / Fs);  // 7Hz对应的FFT bin
    logf("Freq bins: i3=%d i5=%d i7=%d\r\n", i3, i5, i7);

    // 启动采样定时器
    tick.attach(&isr, 9600us);  // 设置采样间隔为9600微秒

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

    // 计算基线平均值
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

        // 收集N个采样点
        size_t idx = 0;
        while (idx < N) {
            if (!tick_flag) continue;
            tick_flag = false;

            uint8_t data[6];

            // 读取加速度计数据
            if (rN(OUT_XL_L, data) != 0) continue;
            int16_t axr = data[0] | (data[1] << 8);
            int16_t ayr = data[2] | (data[3] << 8);
            int16_t azr = data[4] | (data[5] << 8);

            // 读取陀螺仪数据
            if (rN(OUT_G_L, data) != 0) continue;
            int16_t gxr = data[0] | (data[1] << 8);
            int16_t gyr = data[2] | (data[3] << 8);
            int16_t gzr = data[4] | (data[5] << 8);

            // 调试输出原始数据
            if (DEBUG_RAW_EVERY && (idx % DEBUG_RAW_EVERY == 0)) {
                logf("RAW %d %d %d %d %d %d\r\n",
                     axr, ayr, azr, gxr, gyr, gzr);
            }

            // 数据缩放和基线校正
            ax[idx] = axr * 0.000061f - baseline_acc[0];
            ay[idx] = ayr * 0.000061f - baseline_acc[1];
            az[idx] = azr * 0.000061f - baseline_acc[2];
            gx[idx] = gxr * 0.00875f - baseline_gyr[0];
            gy[idx] = gyr * 0.00875f - baseline_gyr[1];
            gz[idx] = gzr * 0.00875f - baseline_gyr[2];

            ++idx;
        }

        // 零填充
        for (size_t i = N; i < FFTN; i++) {
            ax[i] = ay[i] = az[i] = gx[i] = gy[i] = gz[i] = 0;
        }

        // 信号分析
        bool trem = false, dysk = false;
        float levelT = 0, levelD = 0;

        // 信号分析函数
        auto analyze = [&](float *d, float tth, float dth,
                           float scale, const char *tag) {
            // 执行FFT
            arm_rfft_fast_f32(&fft, d, fbuf, 0);
            arm_cmplx_mag_f32(fbuf, mag, FFTN / 2);

            // 计算RMS值
            float rms = 0;
            for (int k = 1; k < i7 + 3; k++) {
                rms += mag[k] * mag[k];
            }
            rms = sqrtf(rms / (i7 + 2));

            // 寻找峰值
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

            // 调试输出FFT分析结果
            if (DEBUG_FFT_SUMMARY) {
                logf("%s 3-5 %.3f@%.1fHz 5-7 %.3f@%.1fHz rms %.3f\r\n",
                     tag, (double)p35, (double)f35,
                     (double)p57, (double)f57,
                     (double)rms);
            }

            // 阈值判断逻辑
            if (p35 >= tth && p35 / rms > PEAK_TO_RMS && rms > tth * 0.3f) {
                trem = true;
                levelT = fmaxf(levelT, p35 / scale);
            }
            if (p57 >= dth && p57 / rms > PEAK_TO_RMS && rms > dth * 0.3f) {
                dysk = true;
                levelD = fmaxf(levelD, p57 / scale);
            }
        };

        // 分析所有通道
        analyze(ax, ACC_T_TH, ACC_D_TH, 0.5f, "AX");
        analyze(ay, ACC_T_TH, ACC_D_TH, 0.5f, "AY");
        analyze(az, ACC_T_TH, ACC_D_TH, 0.5f, "AZ");
        analyze(gx, GYR_T_TH, GYR_D_TH, 100.f, "GX");
        analyze(gy, GYR_T_TH, GYR_D_TH, 100.f, "GY");
        analyze(gz, GYR_T_TH, GYR_D_TH, 100.f, "GZ");

        // 限制信号强度在0-1之间
        levelT = fminf(levelT, 1.0f);
        levelD = fminf(levelD, 1.0f);

        // 调试输出决策变量
        if (DEBUG_THRESH_MSG) {
            logf("Decision T=%d(%.2f) D=%d(%.2f)\r\n",
                 trem, levelT, dysk, levelD);
        }

        // LED反馈控制
        bool show_tremor = false;
        bool show_dyskinesia = false;

        // 判断是否显示症状
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

        // 重置所有LED状态
        led_tremor.write(0);
        led_dyskinesia.write(0);
        led_status = 0;
        led_power = 0;

        // 根据检测结果控制LED
        if (show_dyskinesia) {
            // 运动障碍指示
            led_dyskinesia.period_ms(200);  // 5Hz闪烁
            led_dyskinesia.write(levelD);   // 亮度与强度成正比
        } else if (show_tremor) {
            // 震颤指示
            led_tremor.period_ms(500);      // 2Hz闪烁
            led_tremor.write(levelT);       // 亮度与强度成正比
        }

        // 更新状态LED
        if (show_tremor || show_dyskinesia) {
            led_status = 1;  // 检测到运动时亮起
        }

        // 保持电源LED关闭
        led_power = 0;

        // 输出调试信息
        if (DEBUG_THRESH_MSG) {
            logf("Motion detected - Tremor: %d(%.2f) Dyskinesia: %d(%.2f)\r\n",
                 trem, levelT, dysk, levelD);
        }
    }
}
