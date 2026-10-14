#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <esp_timer.h>

// ---------- DRV5053 线性霍尔 IC -> GPIO1 = ADC1_CH0 ----------
// DRV5053 输出与供电无关: 固定 0.2~1.8V, 零点(QVO) 1.0V
// 选用 ADC1 通道: ESP32-S3 的 ADC2 与 WiFi 存在硬件冲突, WiFi 工作时 ADC2 不可用
#define HALL_PIN            1

#define SAMPLE_HZ           500
#define SAMPLE_PERIOD_US    (1000000UL / SAMPLE_HZ)
#define OVERSAMPLE_N        8       // 每拍过采样次数, 随机噪声按 sqrt(N) 压制 (无 RC 时的算法补偿)
#define EMA_ALPHA           0.3f    // 一阶低通系数 (0~1), 越小越平滑、响应越慢

// ---------- 板载 WS2812 RGB LED (GPIO21) ----------
#define LED_PIN             21
#define NUM_PIXELS          1
#define LED_GREEN           strip.Color(0, 255, 0)
#define LED_OFF             0
#define BLINK_ON_MS         50      // 点亮时长 50ms
#define BLINK_PERIOD_MS     1000    // 闪烁周期 1s

// 采样数据结构 (采样线程->打印线程)
typedef struct
{
    uint32_t t_ms;
    uint16_t raw_mv;
    uint16_t filt_mv;
} sample_t;

static QueueHandle_t sampleQueue = nullptr;

// 硬件定时器回调 (esp_timer 任务上下文): 采样 + 滤波 + 入队
// 采样与打印解耦, 保证严格的 500Hz 采样节拍, 不受串口打印速度影响
static float vFilt = 1.0f;   // 初值取 5053 零点电压, 避免启动瞬态
static bool  filtInit = false;

static void onSampleTimer(void *arg)
{
    (void)arg;

    // 过采样: 每拍连读 N 次取平均, 压制随机噪声 (替代模拟 RC 的算法补偿)
    uint32_t sumMv = 0;
    for (int i = 0; i < OVERSAMPLE_N; i++)
        sumMv += analogReadMilliVolts(HALL_PIN);
    uint32_t rawMv = (uint32_t)(sumMv / OVERSAMPLE_N);

    // 一阶低通 (EMA)
    float vRaw = rawMv / 1000.0f;
    if (!filtInit) { vFilt = vRaw; filtInit = true; }
    vFilt += EMA_ALPHA * (vRaw - vFilt);

    sample_t s = {
        .t_ms   = millis(),
        .raw_mv = (uint16_t)rawMv,
        .filt_mv = (uint16_t)(vFilt * 1000.0f),
    };
    // 队列满时丢弃最旧数据, 绝不让采样节拍受阻
    if (xQueueSend(sampleQueue, &s, 0) != pdTRUE)
    {
        sample_t drop;
        xQueueReceive(sampleQueue, &drop, 0);
        xQueueSend(sampleQueue, &s, 0);
    }
}

// 打印线程: 固定在核心 0, 从队列取数输出 CSV
static void hallPrintTask(void *param)
{
    (void)param;
    sample_t s;
    for (;;)
    {
        if (xQueueReceive(sampleQueue, &s, portMAX_DELAY) == pdTRUE)
            Serial.printf("%lu,%u.%03u,%u.%03u\n",
                          (unsigned long)s.t_ms,
                          s.raw_mv / 1000, s.raw_mv % 1000,
                          s.filt_mv / 1000, s.filt_mv % 1000);
    }
}

Adafruit_NeoPixel strip(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("t_ms,v_raw_V,v_filt_V");

    // --- RGB LED ---
    strip.begin();
    strip.setBrightness(64);
    strip.clear();
    strip.show();

    // --- ADC ---
    analogReadResolution(12);
    // 6dB 衰减: 满量程约 2.2V, 恰好覆盖 5053 的 0.2~1.8V 输出,
    // 信号摆幅分辨率利用率 ~82%, 优于 11dB 档的 ~58%
    analogSetPinAttenuation(HALL_PIN, ADC_6db);

    // 采样队列 + 打印线程 (256 条 = 0.5s 缓冲, 打印偶发延迟不丢采样)
    sampleQueue = xQueueCreate(256, sizeof(sample_t));
    xTaskCreatePinnedToCore(
        hallPrintTask,      // 线程函数
        "hall_print",       // 名称
        4096,               // 栈深度 (字)
        nullptr,            // 参数
        2,                  // 优先级
        nullptr,            // 句柄
        0);                 // 核心 0 (Arduino loop 跑在核心 1)

    // --- 硬件定时器: 严格 500Hz ---
    const esp_timer_create_args_t timerArgs = {
        .callback = &onSampleTimer,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "adc_sample",
        .skip_unhandled_events = false,
    };
    esp_timer_handle_t sampleTimer = nullptr;
    esp_timer_create(&timerArgs, &sampleTimer);
    esp_timer_start_periodic(sampleTimer, SAMPLE_PERIOD_US);
}

void loop()
{
    // 核心 1: RGB 绿灯每秒闪一次 (50ms), 与 ADC 采样完全并行
    strip.setPixelColor(0, LED_GREEN);
    strip.show();
    delay(BLINK_ON_MS);

    strip.setPixelColor(0, LED_OFF);
    strip.show();
    delay(BLINK_PERIOD_MS - BLINK_ON_MS);
}
