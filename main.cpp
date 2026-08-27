/*
 * ESP32-S3 N16R8 简易方波示波器
 *
 * 自测接线：GPIO1（方波输出）——跳线——GPIO4（ADC 输入）
 *
 * 测量流程：
 * 1. 在固定时间内扫描输入，得到最小值和最大值。
 * 2. 取最小值与最大值的中点作为半压阈值。
 * 3. 统计输入从低于阈值变为高于阈值的次数。
 * 4. 用第一个上升沿到最后一个上升沿之间的周期数计算频率。
 * 5. 根据测得频率计算周期，每周期采样 20 次，共采样 100 次。
 * 6. 通过串口输出 5 个周期的原始采样值和 ASCII 波形。
 * 7. 在 OLED 上画出 5 个周期。
 */

 //以后可以把其他周期性、幅度在 0～3.3V 以内、并且与 ESP32 共地的波形接到 GPIO4，然后输入：

#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// 屏幕参数
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1       // 无复位引脚
#define OLED_I2C_ADDR 0x3C     // I2C 地址

// OLED 对象，后面的初始化和波形绘制函数都会使用它。
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 自测时的接线：GPIO1 输出方波，GPIO4 读取方波。
// GPIO4 是 ESP32-S3 的 ADC 输入脚，能把输入电压转换成 0~4095 的数字量。
static const int PIN_OUT = 1;
static const int PIN_IN = 4;

// LEDC 是 ESP32 内置的 PWM 外设，用它产生稳定的硬件方波。
static const int LEDC_CHANNEL = 0; // 使用 LEDC 的第 0 个通道
static const int LEDC_BITS = 12;   // 12 位占空比分辨率，数值范围 0~4095

// 每周期 20 个点，连续采样 100 个点，也就是 5 个周期。oled只有128分辨率，采样频率高于128就会被压缩了。
// 如果每周期 10 个点，连续采样 50 个点，方波看着不够陡峭。
static const int SAMPLE_COUNT = 100;
static const int SAMPLES_PER_PERIOD = 20;

// 12 位 ADC 的最大原始值是 4095。
// 3.3V 只是近似参考值，实际电压需要根据芯片校准结果修正。
static const int ADC_MAX = 4095;
static const float ADC_VOLTAGE = 3.3f;

// ESP32 启动后默认输出 200Hz 方波。
static float outputFrequency = 200.0f;

// 保存 50 个 ADC 采样值，后面会用这些数据打印数值和波形图。
static int samples[SAMPLE_COUNT];

struct FrequencyResult {
    bool valid;
    float frequency;
    int minimum;
    int maximum;
    int threshold;
    int risingEdges;
};

// 配置 GPIO1 输出方波。
//
// 方波有两个重要参数：
// - 频率：每秒重复多少次，例如 200Hz 表示每秒重复 200 次；
// - 占空比：一个周期内高电平所占的时间比例。
//
// 这里使用 50% 占空比，因此高电平和低电平时间大致相等。
static void configureSquareWave() {
    // 设置 LEDC 频率和分辨率。
    // ledcSetup 返回硬件实际采用的频率，实际值可能与目标值略有差异。
    uint32_t actualFrequency = ledcSetup(LEDC_CHANNEL, outputFrequency, LEDC_BITS);

    // 把 LEDC 通道连接到 GPIO1。
    ledcAttachPin(PIN_OUT, LEDC_CHANNEL);

    // 占空比分辨率为 12 位时，4095 的一半约为 2048，即约 50% 占空比。
    ledcWrite(LEDC_CHANNEL, 2048);

    Serial.printf("方波输出：GPIO%d，目标 %.1f Hz，实际 %lu Hz，占空比 50%%\n",
                  PIN_OUT, outputFrequency, (unsigned long)actualFrequency);
}

// 在一段时间内不断读取 ADC，找出输入信号的最低值和最高值。
//
// 例如方波输入可能读到：
// - 低电平约为 0~几十；
// - 高电平约为 4095 附近；
// 因此可以用 minimum 和 maximum 的中点作为判断高低电平的阈值。
static void findMinMax(uint32_t durationMs, int &minimum, int &maximum) {
    // 初始值要设置成相反方向的极值，方便第一次采样时更新。
    minimum = ADC_MAX;
    maximum = 0;
    uint32_t start = millis();

    while (millis() - start < durationMs) {
        int value = analogRead(PIN_IN);
        minimum = min(minimum, value);
        maximum = max(maximum, value);
    }
}

// 测量输入方波的频率。
//
// 频率的定义是：
//     频率 = 1秒内重复的次数
//
// 程序通过检测“低电平变成高电平”的上升沿来判断一次周期开始。
// 如果检测到第一个上升沿和最后一个上升沿：
//     周期数 = 上升沿次数 - 1
//     总时间 = 最后一个上升沿时间 - 第一个上升沿时间
//     频率 = 周期数 / 总时间
static FrequencyResult measureFrequency() {
    FrequencyResult result = {false, 0.0f, 0, 0, 0, 0};

    // 先采样 120ms，得到信号实际的最低值和最高值。
    findMinMax(120, result.minimum, result.maximum);

    // 取最低值和最高值的中间位置作为半压阈值。
    result.threshold = (result.minimum + result.maximum) / 2;

    int amplitude = result.maximum - result.minimum;
    if (amplitude < 100) {
        // 高低电平差太小，通常表示没有接线或输入没有有效波形。
        return result;
    }

    // 增加一点迟滞，避免 ADC 噪声在阈值附近抖动，导致一次上升沿被重复计算。
    // 例如真实阈值是 2000：必须高于 highThreshold 才算变高，
    // 必须低于 lowThreshold 才算重新变低。
    int hysteresis = max(25, amplitude / 10);
    int highThreshold = result.threshold + hysteresis;
    int lowThreshold = result.threshold - hysteresis;

    // 记录当前输入是否处于高电平状态。
    bool high = analogRead(PIN_IN) > result.threshold;
    uint32_t firstEdge = 0;
    uint32_t lastEdge = 0;

    // 在 300ms 测量窗口内持续检测上升沿。
    uint32_t start = millis();
    while (millis() - start < 300) {
        int value = analogRead(PIN_IN);

        // 只有当前处于低电平、并且采样值超过高阈值时，才算检测到上升沿。
        if (!high && value > highThreshold) {
            high = true;
            uint32_t edgeTime = micros();

            if (result.risingEdges == 0) {
                firstEdge = edgeTime;
            }
            lastEdge = edgeTime;
            result.risingEdges++;
        }
        // 只有电压降到低阈值以下，才允许下一次上升沿被检测。
        else if (high && value < lowThreshold) {
            high = false;
        }
    }

    // 至少需要两个上升沿，才能知道它们之间经过了一个完整周期。
    if (result.risingEdges >= 2 && lastEdge > firstEdge) {
        uint32_t elapsedUs = lastEdge - firstEdge;
        int periodCount = result.risingEdges - 1;

        // micros() 的单位是微秒，所以先乘 1000000 换算成 Hz。
        result.frequency = (float)periodCount * 1000000.0f /
                           (float)elapsedUs;
        result.valid = true;
    }
    return result;
}

// 按测得频率采集 50 个波形点。
//
// 采样间隔的计算方法：
//     周期 = 1000000 / 频率（单位：微秒）
//     每个点的间隔 = 周期 / 10
//
// 采样前先等待一个上升沿，称为“触发”。这样每次采样都尽量从相同
// 的波形位置开始，打印出来的 5 个周期不会随机左右移动。
static bool captureWaveform(float frequency, int threshold, uint32_t &sampleIntervalUs) {
    float periodUs = 1000000.0f / frequency;
    sampleIntervalUs = max((uint32_t)1, (uint32_t)lroundf(
        periodUs / SAMPLES_PER_PERIOD));

    // 等待输入从低电平变成高电平，作为第 0 个采样点的参考位置。
    bool high = analogRead(PIN_IN) > threshold;
    uint32_t triggerStart = millis();
    while (millis() - triggerStart < 200) {
        int value = analogRead(PIN_IN);
        if (!high && value > threshold) {
            break;
        }
        high = value > threshold;
    }

    // 使用绝对目标时间进行采样，而不是每次 delay 一段时间。
    // 这样可以避免每次 ADC 读取花费的时间不断累积，减少采样点漂移。
    uint32_t start = micros();
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        uint32_t target = start + (uint32_t)i * sampleIntervalUs;

        // 等到目标时刻再读取 ADC。
        while ((int32_t)(micros() - target) < 0) {
            // 等待到预定采样时刻
        }
        samples[i] = analogRead(PIN_IN);
    }
    return true;
}

// 通过串口打印测量结果和 ASCII 波形。
//
// 每一个采样值都对应波形上的一个横向位置：
// - 第 0~9 个点：第 1 个周期；
// - 第 10~19 个点：第 2 个周期；
// - 依此类推，共 5 个周期。
//
// ASCII 图的纵向位置由采样值大小决定，采样值越大，星号越靠上。
static void printWaveform(const FrequencyResult &frequency,
                          uint32_t sampleIntervalUs) {
    // 这里使用实际采样到的 50 个点重新计算显示范围。
    int minimum = ADC_MAX;
    int maximum = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        minimum = min(minimum, samples[i]);
        maximum = max(maximum, samples[i]);
    }

    // 如果输入信号变化很小，也保留一个最小显示范围，避免除以 0。
    int range = max(20, maximum - minimum);
    Serial.println();
    Serial.println("========== 测量结果 ==========");
    Serial.printf("输入频率：%.2f Hz，上升沿：%d 次\n",
                  frequency.frequency, frequency.risingEdges);
    Serial.printf("输入周期：%.2f us\n", 1000000.0f / frequency.frequency);
    Serial.printf("采样间隔：%lu us（每周期 10 点）\n",
                  (unsigned long)sampleIntervalUs);
    Serial.println("采样数量：50 点（5 个周期）");
    Serial.printf("半压阈值：%d（%.2f V）\n", frequency.threshold,
                  frequency.threshold * ADC_VOLTAGE / ADC_MAX);
    Serial.printf("采样范围：%d（%.2f V）到 %d（%.2f V）\n",
                  minimum, minimum * ADC_VOLTAGE / ADC_MAX,
                  maximum, maximum * ADC_VOLTAGE / ADC_MAX);

    Serial.println("\n原始采样值：");
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        Serial.printf("%5d", samples[i]);
        if ((i + 1) % SAMPLES_PER_PERIOD == 0) {
            Serial.printf("  <- 第 %d 个周期\n", (i + 1) / SAMPLES_PER_PERIOD);
        }
    }

    // 将纵轴分成 16 行，行数越多，波形显示越细。
    const int rows = 16;
    Serial.println("\nASCII 波形（5 个周期）：");
    for (int row = rows; row >= 0; row--) {
        float voltage = (minimum + range * (float)row / rows) *
                        ADC_VOLTAGE / ADC_MAX;
        Serial.printf("%4.2fV |", voltage);
        for (int i = 0; i < SAMPLE_COUNT; i++) {
            int level = lroundf((samples[i] - minimum) * rows / (float)range);
            Serial.print(level == row ? '*' : ' ');
        }
        Serial.println('|');
    }
    Serial.println("      +--------------------------------------------------+");
    Serial.println("       1周期       2周期       3周期       4周期       5周期");
    Serial.println("==============================\n");
}

// 把刚刚采集的 50 个点画到 OLED 上。
//
// OLED 分辨率为 128x64：
// - 上方 12 行显示频率和电压范围；
// - 下方区域显示波形；
// - 50 个采样点均匀分布在屏幕宽度内；
// - 每 10 个点对应一个周期，所以横向显示 5 个周期。
static void drawOLEDWaveform(const FrequencyResult &frequency) {
    int minimum = ADC_MAX;
    int maximum = 0;

    // 找到本次 50 个采样点的显示范围。
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        minimum = min(minimum, samples[i]);
        maximum = max(maximum, samples[i]);
    }

    int range = max(20, maximum - minimum);

    // OLED 的绘图区：x=0~127，y=18~63。
    // 上方两行分别显示频率和 ADC 数值、电压范围。
    const int plotTop = 18;
    const int plotBottom = SCREEN_HEIGHT - 1;
    const int plotWidth = SCREEN_WIDTH - 1;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // 第一行显示测得频率。128 像素宽度放不下太多中文，因此这里用英文缩写。
    display.setCursor(0, 0);
    display.printf("F:%.1fHz", frequency.frequency);

    // 第二行显示 ADC 最小值和最大值，电压值只写在最大 ADC 值后面。
    // 例如：ADC:0-4095(3.30V)
    display.setCursor(0, 8);
    display.printf("ADC:%d-%d(%.2fV)", minimum, maximum,
                   maximum * ADC_VOLTAGE / ADC_MAX);

    // 画出水平中线，作为波形的参考位置。
    int middle = plotBottom - (frequency.threshold - minimum) *
                 (plotBottom - plotTop) / range;
    middle = constrain(middle, plotTop, plotBottom);
    display.drawFastHLine(0, middle, SCREEN_WIDTH, SSD1306_WHITE);

    // 绘制 50 个采样点之间的连线。
    // 连线比单独画点更容易观察方波的高、低电平变化。
    for (int i = 0; i < SAMPLE_COUNT - 1; i++) {
        int x1 = i * plotWidth / (SAMPLE_COUNT - 1);
        int x2 = (i + 1) * plotWidth / (SAMPLE_COUNT - 1);

        int y1 = plotBottom - (samples[i] - minimum) *
                 (plotBottom - plotTop) / range;
        int y2 = plotBottom - (samples[i + 1] - minimum) *
                 (plotBottom - plotTop) / range;

        y1 = constrain(y1, plotTop, plotBottom);
        y2 = constrain(y2, plotTop, plotBottom);
        display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
    }

    // 每 10 个采样点画一条较短的周期分隔线。
    // 这些线只画在底部，不遮挡主要波形。
    for (int period = 1; period < 5; period++) {
        int x = (period * SAMPLES_PER_PERIOD) * plotWidth /
                (SAMPLE_COUNT - 1);
        display.drawFastVLine(x, plotBottom - 3, 4, SSD1306_WHITE);
    }

    // 将图像一次性发送到 OLED。
    display.display();
}

// 执行一次完整测量：先测频率，再根据频率采样并显示波形。
static void measureAndDisplay() {
    Serial.println("\n正在测量频率...");
    FrequencyResult result = measureFrequency();
    if (!result.valid) {
        Serial.printf("测量失败：信号幅度不足或没有信号（min=%d，max=%d）\n",
                      result.minimum, result.maximum);
        Serial.printf("请确认 GPIO%d 已连接到 GPIO%d。\n", PIN_OUT, PIN_IN);
        return;
    }

    Serial.printf("测得频率：%.2f Hz\n", result.frequency);
    Serial.println("正在按每周期 10 点采集 50 个点...");
    uint32_t sampleIntervalUs;
    captureWaveform(result.frequency, result.threshold, sampleIntervalUs);
    printWaveform(result, sampleIntervalUs);
    drawOLEDWaveform(result);
}

static void printHelp() {
    Serial.println("\n========== ESP32-S3 简易示波器 ==========");
    Serial.printf("方波输出：GPIO%d\n", PIN_OUT);
    Serial.printf("ADC 输入：GPIO%d\n", PIN_IN);
    Serial.println("接线：GPIO1 —— 跳线 —— GPIO4");
    Serial.println("命令：");
    Serial.println("  q       重新输出方波");
    Serial.println("  f200    设置输出频率为 200 Hz");
    Serial.println("  m       测量频率并显示 5 个周期");
    Serial.println("  ?       显示帮助");
    Serial.println("==========================================\n");
}


// 初始化函数
void initOLED() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
        Serial.println("SSD1306 初始化失败！");
        while (true);
    }
    display.clearDisplay();
}

// 显示内容函数
void showOLED(){
    display.clearDisplay(); // 每次显示前先清屏，防止文字重叠
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 25);
    display.println("ESP32 Oscilloscope!");
    display.display();
}

void setup() {
    // 启动 USB 串口，串口监视器也必须设置为 115200 波特率。
    Serial.begin(115200);
    
    Wire.begin(8, 9);  // 1. 显式初始化 I2C 总线，指定 ESP32-S3 的引脚,SDA 接 GPIO8，SCL 接 GPIO9
    initOLED();       // 初始化一次
    showOLED(); //示内容

    // 给 USB CDC 一点枚举时间，避免刚启动时串口信息来不及显示。
    delay(1500);

    // 设置 ADC 使用 12 位分辨率，返回值范围是 0~4095。
    analogReadResolution(12);

    // 设置 ADC 输入量程。ADC_11db 可以测量接近 3.3V 的输入，
    // 但 GPIO 输入电压绝对不能超过 ESP32-S3 的允许范围。
    analogSetPinAttenuation(PIN_IN, ADC_11db);
    pinMode(PIN_IN, INPUT);

    // 上电后立即开始输出默认的 200Hz、50% 占空比方波。
    configureSquareWave();
    printHelp();

}

void loop() {
    // 没有串口命令时不做测量
    if (!Serial.available()) {
        return;
    }

    // 读取一整行命令，并删除前后的空格和换行符。
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() == 0) {
        return;
    }

    // q：恢复当前频率的方波输出。
    if (command[0] == 'q') {
        configureSquareWave();
    }
    // f数字：修改输出频率，例如 f500 表示输出 500Hz。
    else if (command[0] == 'f') {
        float frequency = command.substring(1).toFloat();
        if (frequency >= 10.0f && frequency <= 5000.0f) {
            outputFrequency = frequency;
            configureSquareWave();
        } else {
            Serial.println("频率范围必须是 10 到 5000 Hz。");
        }
    }
    // m：执行一次“测频率 + 采样 50 点 + 显示波形”。
    else if (command[0] == 'm') {
        measureAndDisplay();
    }
    // ?：打印命令帮助。
    else if (command[0] == '?') {
        printHelp();
    } else {
        Serial.println("未知命令，请输入 ? 查看帮助。");
    }
}
