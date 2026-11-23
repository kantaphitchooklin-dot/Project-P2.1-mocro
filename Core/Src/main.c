#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* Peripheral handles */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* TCS34725 */
#define TCS34725_ADDR        (0x29 << 1)
#define TCS34725_COMMAND     0x80
#define TCS34725_ENABLE      0x00
#define TCS34725_ATIME       0x01
#define TCS34725_CDATAL      0x14
#define TCS34725_CONTROL     0x0F

/* Color definitions */
#define COLOR_UNKNOWN        0
#define COLOR_RED            1
#define COLOR_GREEN          2
#define COLOR_BLUE           3

#define MIN_CLEAR_VALUE      100
#define OBJECT_MIN_DISTANCE  2.0f
#define OBJECT_MAX_DISTANCE  15.0f

/* QR input from Python bridge */
#define QR_BUFFER_SIZE       64

/* IR sensor: PA4, active LOW = object detected
   External indicator LED: PA5 */
#define IR_SENSOR_PIN        GPIO_PIN_4
#define IR_LED_PIN           GPIO_PIN_5
#define IR_DETECTED_LEVEL    GPIO_PIN_RESET

/* Servo PWM on TIM3, 50 Hz:
     PA6 = TIM3_CH1 = Diverter servo
     PA7 = TIM3_CH2 = Conveyor servo (continuous rotation)
   1 timer tick = 1 us, period = 20000 us, compare value = pulse width in us.
   Pulse 0 = no signal on the pin.
   The two limits below are only guard rails against typing mistakes.
   They are NOT calibrated angles or conveyor speeds. */
#define SERVO_PULSE_MIN_US   500U
#define SERVO_PULSE_MAX_US   2500U

/* Calibrated pulse widths (us), found and re-tested on the real hardware
   in Step 4. Diverter is selected by the DETECTED color.
   Note: the pulse order is not the same as the angle order. This is what
   was measured, so do not "fix" it without re-testing. */
#define DIVERTER_RED_US      1500U   /* 45 deg  */
#define DIVERTER_GREEN_US    1100U   /* 90 deg  */
#define DIVERTER_BLUE_US     2200U   /* 160 deg */

/* Conveyor working value: about 2.2 cm/s measured.
   Stop = signal OFF (pulse 0), the same thing TEST:STOP does (tested: belt stops).
   The automatic cycle starts the belt with START and stops it with STOP. */
#define CONVEYOR_RUN_US      1800U
#define CONVEYOR_STOP_US     0U

/* Automatic stop-and-measure cycle (Steps 6 and 7). All times are in ms.
   These are starting values, not calibrated: tune them on the real belt. */
#define IR_CONFIRM_MS        40U     /* IR must stay detected this long to count     */
#define IR_TO_STOP_DELAY_MS  0U      /* belt keeps running this long after IR; use   */
                                     /* it if the product stops before reaching TCS  */
#define SETTLE_MS            400U    /* belt stopped, let the product settle         */
#define COLOR_SAMPLES        5U      /* readings averaged for one decision           */
#define COLOR_SAMPLE_GAP_MS  70U     /* must be longer than the 50 ms TCS integration */
#define DIVERTER_SETTLE_MS   700U    /* wait for the Diverter before the belt moves  */
#define IR_CLEAR_MS          300U    /* IR must be clear this long = product left    */
#define CLEAR_WARN_MS        10000U  /* warn if IR is still blocked after restart    */

/* What to do when the color is UNKNOWN (or the sensor fails):
     0 = Diverter keeps its last position, result is REJECT, the belt keeps going
     1 = the belt halts and waits for START
   The final choice is up to the project owner. */
#define UNKNOWN_HALTS_BELT   0

/* Model definitions */
#define MODEL_NONE           0
#define MODEL_A              1
#define MODEL_B              2
#define MODEL_C              3

/* Sensor data */
uint16_t red = 0;
uint16_t green = 0;
uint16_t blue = 0;
uint16_t clear = 0;

/* QR receive data */
uint8_t qr_rx_byte = 0;
char qr_buffer[QR_BUFFER_SIZE];
volatile uint8_t qr_index = 0;
volatile uint8_t qr_ready = 0;
volatile uint8_t qr_overflow = 0;

/* Current product model */
uint8_t current_model = MODEL_NONE;

/* Last servo pulse widths (us), 0 = no signal */
static uint16_t servo_diverter_us = 0U;
static uint16_t servo_conveyor_us = 0U;

/* TCS34725 initialisation result (START is refused when it is 0) */
static uint8_t tcs_ready = 0U;

/* Automatic cycle state */
typedef enum
{
    AUTO_OFF = 0,     /* manual mode (old behavior)                                  */
    AUTO_RUN,         /* belt running, waiting for a product at the color station    */
    AUTO_ADVANCE,     /* IR confirmed, belt keeps running for IR_TO_STOP_DELAY_MS    */
    AUTO_SETTLE,      /* belt stopped, waiting for the product to settle             */
    AUTO_MEASURE,     /* taking color samples                                        */
    AUTO_SORT,        /* Diverter commanded, waiting for it to reach its position    */
    AUTO_CLEAR,       /* belt running again, waiting for the product to leave        */
    AUTO_HALT         /* belt halted after UNKNOWN (only if UNKNOWN_HALTS_BELT = 1)  */
} AutoState_t;

static AutoState_t auto_state = AUTO_OFF;
static uint32_t auto_t0 = 0U;
static uint32_t ir_since = 0U;
static uint32_t clear_since = 0U;
static uint8_t ir_seen = 0U;
static uint8_t no_qr_reported = 0U;
static uint8_t clear_started = 0U;
static uint8_t clear_warned = 0U;
static uint32_t sum_r = 0U, sum_g = 0U, sum_b = 0U, sum_c = 0U;
static uint8_t sample_n = 0U;
static uint8_t good_n = 0U;
static float dist_samples[COLOR_SAMPLES];   /* valid HC-SR04 readings of this stop */
static uint8_t dist_n = 0U;

/* Function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void Error_Handler(void);

void delay_us(uint16_t us);
uint8_t HCSR04_Read(float *distance);
uint8_t TCS34725_Init(void);
uint8_t TCS34725_ReadColor(void);
uint8_t ClassifyColor(void);
const char *GetColorName(uint8_t color);
const char *GetModelName(uint8_t model);
uint8_t GetExpectedColor(uint8_t model);
void ProcessQRCode(const char *qr);
void PerformQC(void);
void Servo_SetDiverterUs(uint16_t us);
void Servo_SetConveyorUs(uint16_t us);
uint8_t Diverter_SortByColor(uint8_t color);
void TestColorSort(void);
void HandleTestCommand(const char *cmd);
void AutoStart(void);
void AutoStop(void);
void AutoProcess(uint8_t ir_detected);

/* printf output through USART2 */
int __io_putchar(int ch)
{
    uint8_t data = (uint8_t)ch;
    HAL_UART_Transmit(&huart2, &data, 1, HAL_MAX_DELAY);
    return ch;
}

/* UART receive interrupt: one byte at a time */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (qr_rx_byte == '\r' || qr_rx_byte == '\n')
        {
            if ((qr_index > 0U) && (qr_ready == 0U))
            {
                qr_buffer[qr_index] = '\0';
                qr_ready = 1U;
                qr_index = 0U;
            }
        }
        else if (qr_ready == 0U)
        {
            if (qr_index < (QR_BUFFER_SIZE - 1U))
            {
                qr_buffer[qr_index++] = (char)qr_rx_byte;
            }
            else
            {
                /* Discard an over-length QR string */
                qr_index = 0U;
                qr_overflow = 1U;
            }
        }

        /* Re-arm reception for the next byte */
        HAL_UART_Receive_IT(&huart2, &qr_rx_byte, 1);
    }
}

/* Microsecond delay using TIM2 (1 MHz counter) */
void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us)
    {
    }
}

/* HC-SR04: TRIG PA0, ECHO PA1.
   ECHO must be reduced to 3.3V using a resistor divider. */
uint8_t HCSR04_Read(float *distance)
{
    uint32_t timeout = 0;
    uint32_t pulse_width;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    delay_us(2);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

    /* Wait for ECHO rising edge */
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
        if (++timeout > 30000U)
            return 0;
    }

    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /* Wait for ECHO falling edge */
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
    {
        if (__HAL_TIM_GET_COUNTER(&htim2) > 30000U)
            return 0;
    }

    pulse_width = __HAL_TIM_GET_COUNTER(&htim2);
    *distance = (float)pulse_width * 0.0343f / 2.0f;

    return 1;
}

/* Initialize TCS34725 */
uint8_t TCS34725_Init(void)
{
    uint8_t data;

    data = 0x03; /* Power ON + RGBC enable */
    if (HAL_I2C_Mem_Write(&hi2c1, TCS34725_ADDR,
                          TCS34725_COMMAND | TCS34725_ENABLE,
                          I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK)
        return 0;

    HAL_Delay(10);

    data = 0xEB; /* Integration time */
    if (HAL_I2C_Mem_Write(&hi2c1, TCS34725_ADDR,
                          TCS34725_COMMAND | TCS34725_ATIME,
                          I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK)
        return 0;

    data = 0x01; /* Gain x4 */
    if (HAL_I2C_Mem_Write(&hi2c1, TCS34725_ADDR,
                          TCS34725_COMMAND | TCS34725_CONTROL,
                          I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK)
        return 0;

    HAL_Delay(50);
    return 1;
}

/* Read Clear, Red, Green, Blue channels */
uint8_t TCS34725_ReadColor(void)
{
    uint8_t buffer[8];

    if (HAL_I2C_Mem_Read(&hi2c1, TCS34725_ADDR,
                         TCS34725_COMMAND | TCS34725_CDATAL,
                         I2C_MEMADD_SIZE_8BIT, buffer, 8, 100) != HAL_OK)
    {
        red = green = blue = clear = 0;
        return 0;
    }

    clear = (uint16_t)((buffer[1] << 8) | buffer[0]);
    red   = (uint16_t)((buffer[3] << 8) | buffer[2]);
    green = (uint16_t)((buffer[5] << 8) | buffer[4]);
    blue  = (uint16_t)((buffer[7] << 8) | buffer[6]);

    return 1;
}

/* Color classification using the existing project's thresholds */
uint8_t ClassifyColor(void)
{
    if (clear < MIN_CLEAR_VALUE)
        return COLOR_UNKNOWN;

    if ((uint32_t)red * 100U > (uint32_t)green * 120U &&
        (uint32_t)red * 100U > (uint32_t)blue * 120U)
        return COLOR_RED;

    if (blue > red &&
        (uint32_t)blue * 100U >= (uint32_t)green * 85U)
        return COLOR_BLUE;

    if ((uint32_t)green * 100U > (uint32_t)red * 120U &&
        (uint32_t)green * 100U > (uint32_t)blue * 120U)
        return COLOR_GREEN;

    return COLOR_UNKNOWN;
}

const char *GetColorName(uint8_t color)
{
    switch (color)
    {
        case COLOR_RED:   return "RED";
        case COLOR_GREEN: return "GREEN";
        case COLOR_BLUE:  return "BLUE";
        default:          return "UNKNOWN";
    }
}

const char *GetModelName(uint8_t model)
{
    switch (model)
    {
        case MODEL_A: return "MODEL_A";
        case MODEL_B: return "MODEL_B";
        case MODEL_C: return "MODEL_C";
        default:      return "NONE";
    }
}

uint8_t GetExpectedColor(uint8_t model)
{
    switch (model)
    {
        case MODEL_A: return COLOR_RED;
        case MODEL_B: return COLOR_BLUE;
        case MODEL_C: return COLOR_GREEN;
        default:      return COLOR_UNKNOWN;
    }
}

/* Accept a new QR only when the previous product has completed QC */
void ProcessQRCode(const char *qr)
{
    if (current_model != MODEL_NONE)
    {
        printf("BUSY: Previous product has not completed QC\r\n");
        printf("QR ignored: %s\r\n", qr);
        return;
    }

    if (strcmp(qr, "MODEL_A") == 0)
        current_model = MODEL_A;
    else if (strcmp(qr, "MODEL_B") == 0)
        current_model = MODEL_B;
    else if (strcmp(qr, "MODEL_C") == 0)
        current_model = MODEL_C;
    else
    {
        printf("Invalid QR Code: %s\r\n", qr);
        current_model = MODEL_NONE;
        return;
    }

    printf("QR Accepted: %s\r\n", GetModelName(current_model));
    printf("Expected Color: %s\r\n",
           GetColorName(GetExpectedColor(current_model)));
    printf("Waiting for product at IR inspection station...\r\n");
}

/* Run one inspection when the IR sensor detects a product */
void PerformQC(void)
{
    float distance = 0.0f;
    uint8_t distance_ok;
    uint8_t sensor_ok;
    uint8_t detected_color;
    uint8_t expected_color;

    if (current_model == MODEL_NONE)
    {
        printf("IR detected, but no valid QR model is assigned\r\n");
        return;
    }

    printf("\r\n----- QC START -----\r\n");
    printf("Model: %s\r\n", GetModelName(current_model));

    /* Allow the product to settle at the inspection position */
    HAL_Delay(100);

    distance_ok = HCSR04_Read(&distance);

    if (!distance_ok)
    {
        printf("Distance sensor error: REJECT\r\n");
        printf("QC RESULT: REJECT\r\n");
        current_model = MODEL_NONE;
        printf("Ready for next product\r\n");
        return;
    }

    printf("Distance: %.1f cm\r\n", distance);

    if (distance < OBJECT_MIN_DISTANCE ||
        distance > OBJECT_MAX_DISTANCE)
    {
        printf("Object outside distance range: REJECT\r\n");
        printf("QC RESULT: REJECT\r\n");
        current_model = MODEL_NONE;
        printf("Ready for next product\r\n");
        return;
    }

    sensor_ok = TCS34725_ReadColor();

    if (!sensor_ok)
    {
        printf("TCS34725 read error: REJECT\r\n");
        printf("QC RESULT: REJECT\r\n");
        current_model = MODEL_NONE;
        printf("Ready for next product\r\n");
        return;
    }

    detected_color = ClassifyColor();
    expected_color = GetExpectedColor(current_model);

    printf("RGB: R=%u G=%u B=%u C=%u\r\n",
           red, green, blue, clear);
    printf("Detected Color: %s\r\n", GetColorName(detected_color));
    printf("Expected Color: %s\r\n", GetColorName(expected_color));

    if (detected_color != COLOR_UNKNOWN &&
        detected_color == expected_color)
    {
        printf("QC RESULT: PASS\r\n");
    }
    else
    {
        printf("QC RESULT: REJECT\r\n");
    }

    /* Clear model so the next QR belongs to the next product */
    current_model = MODEL_NONE;

    printf("----- QC END -----\r\n");
    printf("Ready for next product\r\n\r\n");
}

/* Set the Diverter servo pulse width in us (TIM3_CH1, PA6). 0 = no signal. */
void Servo_SetDiverterUs(uint16_t us)
{
    servo_diverter_us = us;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, us);
}

/* Set the Conveyor servo pulse width in us (TIM3_CH2, PA7). 0 = no signal. */
void Servo_SetConveyorUs(uint16_t us)
{
    servo_conveyor_us = us;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, us);
}

/* Move the Diverter according to the DETECTED color.
   Returns 1 if the servo was commanded, 0 if it was not.
   UNKNOWN (or any other value) never moves the servo: it keeps its
   last position. The expected color from the QR code is not used here. */
uint8_t Diverter_SortByColor(uint8_t color)
{
    switch (color)
    {
        case COLOR_RED:
            Servo_SetDiverterUs(DIVERTER_RED_US);
            return 1U;

        case COLOR_GREEN:
            Servo_SetDiverterUs(DIVERTER_GREEN_US);
            return 1U;

        case COLOR_BLUE:
            Servo_SetDiverterUs(DIVERTER_BLUE_US);
            return 1U;

        default:
            return 0U;
    }
}

/* Step 5 test: read the color once right now and move the Diverter.
   Not connected to the IR sensor or PerformQC() yet. */
void TestColorSort(void)
{
    uint8_t color;

    if (!TCS34725_ReadColor())
    {
        printf("[TEST] TCS34725 read error: Diverter NOT moved\r\n");
        return;
    }

    color = ClassifyColor();

    printf("[TEST] RGB: R=%u G=%u B=%u C=%u\r\n",
           red, green, blue, clear);
    printf("[TEST] Detected Color: %s\r\n", GetColorName(color));

    if (Diverter_SortByColor(color))
    {
        printf("[TEST] Diverter (PA6) set to %u us for %s\r\n",
               (unsigned int)servo_diverter_us, GetColorName(color));
    }
    else
    {
        printf("[TEST] Color not recognized: Diverter NOT moved\r\n");
    }
}

/* ===================== Automatic stop-and-measure cycle (Step 6) =====================
   Start it with START, stop it with STOP (typed in the Python bridge).
   Cycle: belt runs -> IR sees the product at the color station -> belt stops ->
          product settles -> several color samples are averaged -> Diverter moves
          to the detected color -> belt runs again -> wait until the product has
          left the IR beam -> ready for the next product.
   Everything is non-blocking (HAL_GetTick), so QR input keeps working. */

static void Auto_BeginSettle(uint32_t now)
{
    Servo_SetConveyorUs(CONVEYOR_STOP_US);
    auto_state = AUTO_SETTLE;
    auto_t0 = now;
    printf("Belt STOPPED for color measurement\r\n");
}

static void Auto_ResumeBelt(uint32_t now)
{
    Servo_SetConveyorUs(CONVEYOR_RUN_US);
    auto_state = AUTO_CLEAR;
    auto_t0 = now;
    clear_started = 0U;
    clear_warned = 0U;
    printf("Belt RUNNING again\r\n");
}

/* Text for a distance in cm with one decimal, built with integers only.
   (printf of a float needs a linker option in CubeIDE, so it is avoided here.) */
static void Fmt1(char *buf, uint8_t size, float value)
{
    uint32_t tenths;

    if (value < 0.0f)
        value = 0.0f;

    tenths = (uint32_t)((value * 10.0f) + 0.5f);
    snprintf(buf, size, "%lu.%lu", (unsigned long)(tenths / 10U), (unsigned long)(tenths % 10U));
}

/* Median of the valid distance readings (robust against a single bad echo).
   Returns 0 when there is no valid reading. */
static uint8_t Auto_Distance(float *median, float *dmin, float *dmax)
{
    float tmp[COLOR_SAMPLES];
    uint8_t i;
    uint8_t j;

    if (dist_n == 0U)
        return 0U;

    for (i = 0U; i < dist_n; i++)
        tmp[i] = dist_samples[i];

    /* insertion sort (at most COLOR_SAMPLES values) */
    for (i = 1U; i < dist_n; i++)
    {
        float v = tmp[i];
        j = i;
        while ((j > 0U) && (tmp[j - 1U] > v))
        {
            tmp[j] = tmp[j - 1U];
            j--;
        }
        tmp[j] = v;
    }

    if ((dist_n % 2U) != 0U)
        *median = tmp[dist_n / 2U];
    else
        *median = (tmp[(dist_n / 2U) - 1U] + tmp[dist_n / 2U]) / 2.0f;

    *dmin = tmp[0];
    *dmax = tmp[dist_n - 1U];
    return 1U;
}

/* Average the samples, classify, move the Diverter, print the result.
   The distance is REPORTED only: no distance limits are defined yet, so it
   does not change the QC result or the Diverter. */
static void Auto_FinishMeasure(uint32_t now)
{
    uint8_t detected;
    uint8_t expected = GetExpectedColor(current_model);
    uint8_t sorted;
    uint8_t qc_pass;
    uint8_t dist_ok;
    float dist_med = 0.0f;
    float dist_min = 0.0f;
    float dist_max = 0.0f;
    char dist_text[12];
    char t_med[12];
    char t_min[12];
    char t_max[12];

    if (good_n == 0U)
    {
        printf("TCS34725 read error on every sample: color UNKNOWN\r\n");
        red = 0U;
        green = 0U;
        blue = 0U;
        clear = 0U;
        detected = COLOR_UNKNOWN;
    }
    else
    {
        red   = (uint16_t)(sum_r / good_n);
        green = (uint16_t)(sum_g / good_n);
        blue  = (uint16_t)(sum_b / good_n);
        clear = (uint16_t)(sum_c / good_n);

        printf("Average: R=%u G=%u B=%u C=%u (%u samples)\r\n",
               red, green, blue, clear, (unsigned int)good_n);

        detected = ClassifyColor();
    }

    printf("Detected Color: %s\r\n", GetColorName(detected));
    printf("Expected Color: %s\r\n", GetColorName(expected));

    qc_pass = ((detected != COLOR_UNKNOWN) && (detected == expected)) ? 1U : 0U;

    dist_ok = Auto_Distance(&dist_med, &dist_min, &dist_max);

    if (dist_ok != 0U)
    {
        Fmt1(t_med, sizeof(t_med), dist_med);
        Fmt1(t_min, sizeof(t_min), dist_min);
        Fmt1(t_max, sizeof(t_max), dist_max);
        printf("Distance: %s cm (median of %u, range %s-%s cm)\r\n",
               t_med, (unsigned int)dist_n, t_min, t_max);
        snprintf(dist_text, sizeof(dist_text), "%s", t_med);
    }
    else
    {
        printf("Distance: HC-SR04 gave no valid reading\r\n");
        snprintf(dist_text, sizeof(dist_text), "NA");
    }

    printf("QC RESULT (color only, distance not judged yet): %s\r\n",
           (qc_pass != 0U) ? "PASS" : "REJECT");

    /* One line with everything about this product */
    printf("RECORD: model=%s detected=%s expected=%s R=%u G=%u B=%u C=%u dist_cm=%s qc=%s\r\n",
           GetModelName(current_model), GetColorName(detected), GetColorName(expected),
           red, green, blue, clear, dist_text,
           (qc_pass != 0U) ? "PASS" : "REJECT");

    /* The Diverter follows the DETECTED color. UNKNOWN never moves it. */
    sorted = Diverter_SortByColor(detected);

    if (sorted != 0U)
    {
        printf("Diverter (PA6) set to %u us for %s\r\n",
               (unsigned int)servo_diverter_us, GetColorName(detected));
    }
    else
    {
        printf("Color not recognized: Diverter NOT moved (keeps last position)\r\n");
    }

    /* The next QR belongs to the next product */
    current_model = MODEL_NONE;

    if (sorted != 0U)
    {
        auto_state = AUTO_SORT;     /* wait for the servo before the belt moves */
        auto_t0 = now;
    }
    else if (UNKNOWN_HALTS_BELT != 0)
    {
        auto_state = AUTO_HALT;
        printf("UNKNOWN color: belt HALTED. Fix the problem, then send START\r\n");
    }
    else
    {
        Auto_ResumeBelt(now);
    }
}

/* Call this every pass of the main loop while auto mode is not OFF.
   ir_detected = 1 when the IR sensor sees a product. */
void AutoProcess(uint8_t ir_detected)
{
    uint32_t now = HAL_GetTick();
    uint8_t color_ok;
    uint8_t dist_ok;
    float dist_now = 0.0f;
    char dist_txt[12];

    switch (auto_state)
    {
        case AUTO_RUN:
            if (ir_detected == 0U)
            {
                ir_seen = 0U;
                no_qr_reported = 0U;
                break;
            }

            if (ir_seen == 0U)
            {
                ir_seen = 1U;
                ir_since = now;
                break;
            }

            if ((now - ir_since) < IR_CONFIRM_MS)
                break;

            /* IR detection confirmed */
            if (current_model == MODEL_NONE)
            {
                if (no_qr_reported == 0U)
                {
                    no_qr_reported = 1U;
                    printf("IR: product detected, but no QR model is assigned. Ignored (scan the QR first)\r\n");
                }
                break;
            }

            printf("\r\n----- AUTO CYCLE -----\r\n");
            printf("Model: %s\r\n", GetModelName(current_model));
            printf("IR: PRODUCT DETECTED at the color station\r\n");

            if (IR_TO_STOP_DELAY_MS > 0U)
            {
                auto_state = AUTO_ADVANCE;
                auto_t0 = now;
            }
            else
            {
                Auto_BeginSettle(now);
            }
            break;

        case AUTO_ADVANCE:
            if ((now - auto_t0) > IR_TO_STOP_DELAY_MS)
                Auto_BeginSettle(now);
            break;

        case AUTO_SETTLE:
            if ((now - auto_t0) >= SETTLE_MS)
            {
                sum_r = 0U;
                sum_g = 0U;
                sum_b = 0U;
                sum_c = 0U;
                sample_n = 0U;
                good_n = 0U;
                dist_n = 0U;
                auto_state = AUTO_MEASURE;
                auto_t0 = now - COLOR_SAMPLE_GAP_MS;   /* first sample right away */
            }
            break;

        case AUTO_MEASURE:
            if ((now - auto_t0) < COLOR_SAMPLE_GAP_MS)
                break;

            auto_t0 = now;
            sample_n++;

            /* One color reading and one distance reading per sample.
               The 70 ms gap also respects the HC-SR04 minimum cycle time. */
            color_ok = TCS34725_ReadColor();
            dist_ok = HCSR04_Read(&dist_now);

            if (color_ok != 0U)
            {
                good_n++;
                sum_r += red;
                sum_g += green;
                sum_b += blue;
                sum_c += clear;
            }

            if ((dist_ok != 0U) && (dist_n < COLOR_SAMPLES))
            {
                dist_samples[dist_n] = dist_now;
                dist_n++;
            }

            printf("Sample %u:", (unsigned int)sample_n);

            if (color_ok != 0U)
                printf(" R=%u G=%u B=%u C=%u", red, green, blue, clear);
            else
                printf(" color read error");

            if (dist_ok != 0U)
            {
                Fmt1(dist_txt, sizeof(dist_txt), dist_now);
                printf(" D=%s cm\r\n", dist_txt);
            }
            else
                printf(" D=error\r\n");

            if (sample_n >= COLOR_SAMPLES)
                Auto_FinishMeasure(now);
            break;

        case AUTO_SORT:
            if ((now - auto_t0) >= DIVERTER_SETTLE_MS)
                Auto_ResumeBelt(now);
            break;

        case AUTO_CLEAR:
            if (ir_detected != 0U)
            {
                clear_started = 0U;

                if ((clear_warned == 0U) && ((now - auto_t0) >= CLEAR_WARN_MS))
                {
                    clear_warned = 1U;
                    printf("WARNING: IR still detects something after the belt restarted\r\n");
                }
                break;
            }

            if (clear_started == 0U)
            {
                clear_started = 1U;
                clear_since = now;
                break;
            }

            if ((now - clear_since) >= IR_CLEAR_MS)
            {
                auto_state = AUTO_RUN;
                ir_seen = 0U;
                no_qr_reported = 0U;
                printf("Ready for next product\r\n\r\n");
            }
            break;

        case AUTO_OFF:
        case AUTO_HALT:
        default:
            break;
    }
}

/* START command: begin the automatic cycle (or continue after a halt) */
void AutoStart(void)
{
    switch (auto_state)
    {
        case AUTO_OFF:
            if (tcs_ready == 0U)
            {
                printf("AUTO START refused: TCS34725 is not ready (check I2C wiring)\r\n");
                return;
            }

            Servo_SetConveyorUs(CONVEYOR_RUN_US);
            auto_state = AUTO_RUN;
            ir_seen = 0U;
            no_qr_reported = 0U;

            printf("AUTO MODE: ON - belt running at %u us\r\n",
                   (unsigned int)CONVEYOR_RUN_US);

            if (current_model == MODEL_NONE)
                printf("Scan a QR code: the next product will be measured\r\n");
            else
                printf("QR already assigned: %s\r\n", GetModelName(current_model));
            break;

        case AUTO_HALT:
            printf("Continuing after halt\r\n");
            Auto_ResumeBelt(HAL_GetTick());
            break;

        default:
            printf("AUTO MODE is already ON\r\n");
            break;
    }
}

/* STOP command: stop the belt and leave auto mode.
   The Diverter keeps its last position. */
void AutoStop(void)
{
    Servo_SetConveyorUs(CONVEYOR_STOP_US);

    if (auto_state != AUTO_OFF)
        printf("AUTO MODE: OFF - belt stopped (Diverter keeps its position)\r\n");
    else
        printf("Belt stopped (auto mode was already OFF)\r\n");

    auto_state = AUTO_OFF;
}

/* TEMPORARY test commands (typed in the Python bridge).
     TEST:D<us>   Diverter pulse width in us, e.g. TEST:D1500
     TEST:C<us>   Conveyor pulse width in us, e.g. TEST:C1550
     TEST:COLOR   Read the color now and move the Diverter to match
     TEST:STOP    Turn both servo signals off (pulse = 0)
   A value of 0 turns that servo signal off. Other values must be
   between SERVO_PULSE_MIN_US and SERVO_PULSE_MAX_US.
   Remove or reduce this function when the real sorting flow is added. */
void HandleTestCommand(const char *cmd)
{
    const char *arg = cmd + 5;   /* skip "TEST:" */
    char target;
    uint32_t value = 0U;
    uint8_t len = 0U;

    if (strcmp(arg, "COLOR") == 0)
    {
        TestColorSort();
        return;
    }

    if (auto_state != AUTO_OFF)
    {
        printf("[TEST] Auto mode is ON. Send STOP first (TEST:COLOR still works)\r\n");
        return;
    }

    if (strcmp(arg, "STOP") == 0)
    {
        Servo_SetDiverterUs(0U);
        Servo_SetConveyorUs(0U);
        printf("[TEST] STOP: both servo signals OFF (pulse = 0)\r\n");
        return;
    }

    target = arg[0];
    if ((target >= 'a') && (target <= 'z'))
    {
        target = (char)(target - 'a' + 'A');
    }

    if ((target != 'D') && (target != 'C'))
    {
        printf("[TEST] Unknown test command: %s\r\n", cmd);
        printf("[TEST] Use TEST:D<us>, TEST:C<us>, TEST:COLOR or TEST:STOP\r\n");
        return;
    }

    arg++;   /* skip the D or C */

    /* Accept 1 to 4 digits only */
    while ((arg[len] >= '0') && (arg[len] <= '9') && (len < 5U))
    {
        value = (value * 10U) + (uint32_t)(arg[len] - '0');
        len++;
    }

    if ((len == 0U) || (len > 4U) || (arg[len] != '\0'))
    {
        printf("[TEST] Bad number in command: %s\r\n", cmd);
        printf("[TEST] Use 1 to 4 digits, e.g. TEST:%c1500\r\n", target);
        return;
    }

    if ((value != 0U) &&
        ((value < SERVO_PULSE_MIN_US) || (value > SERVO_PULSE_MAX_US)))
    {
        printf("[TEST] Out of range: %u us (allowed 0 or %u-%u)\r\n",
               (unsigned int)value,
               (unsigned int)SERVO_PULSE_MIN_US,
               (unsigned int)SERVO_PULSE_MAX_US);
        return;
    }

    if (target == 'D')
    {
        Servo_SetDiverterUs((uint16_t)value);
        printf("[TEST] Diverter (PA6) pulse = %u us%s\r\n",
               (unsigned int)servo_diverter_us,
               (value == 0U) ? " (signal OFF)" : "");
    }
    else
    {
        Servo_SetConveyorUs((uint16_t)value);
        printf("[TEST] Conveyor (PA7) pulse = %u us%s\r\n",
               (unsigned int)servo_conveyor_us,
               (value == 0U) ? " (signal OFF)" : "");
    }
}

int main(void)
{
    uint8_t ir_now;
    uint8_t ir_previous = GPIO_PIN_SET;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
        Error_Handler();

    /* Start servo PWM. Both compare values are 0, so no pulse is sent
       until a TEST command sets a width. */
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();

    /* Start receiving QR data */
    if (HAL_UART_Receive_IT(&huart2, &qr_rx_byte, 1) != HAL_OK)
        Error_Handler();

    printf("\r\n===== SMART MOTOR COVER QC SYSTEM =====\r\n");
    printf("QR UART: READY (USART2, 115200)\r\n");
    printf("HC-SR04: READY (TRIG PA0, ECHO PA1)\r\n");
    printf("IR SENSOR: READY (PA4, active LOW)\r\n");
    printf("IR INDICATOR LED: PA5\r\n");
    printf("SERVO PWM: READY (TIM3 50 Hz, PA6 Diverter, PA7 Conveyor)\r\n");
    printf("Servo signals are OFF until START or a TEST command is sent\r\n");
    printf("Diverter by color (us): RED %u, GREEN %u, BLUE %u\r\n",
           (unsigned int)DIVERTER_RED_US,
           (unsigned int)DIVERTER_GREEN_US,
           (unsigned int)DIVERTER_BLUE_US);

    tcs_ready = TCS34725_Init();

    if (tcs_ready != 0U)
        printf("TCS34725: READY\r\n");
    else
        printf("TCS34725: ERROR - check I2C wiring\r\n");

    printf("AUTO MODE: OFF. Send START to run the belt automatically, STOP to halt it\r\n");
    printf("Auto cycle measures color AND distance (HC-SR04 at the color station)\r\n");
    printf("Waiting for QR Code...\r\n");

    while (1)
    {
        /* Copy completed QR out of the interrupt buffer safely */
        if (qr_ready != 0U)
        {
            char local_qr[QR_BUFFER_SIZE];

            __disable_irq();
            strncpy(local_qr, qr_buffer, QR_BUFFER_SIZE);
            local_qr[QR_BUFFER_SIZE - 1U] = '\0';
            qr_ready = 0U;
            __enable_irq();

            printf("\r\nQR Received: %s\r\n", local_qr);
            if (strcmp(local_qr, "START") == 0)
                AutoStart();
            else if (strcmp(local_qr, "STOP") == 0)
                AutoStop();
            else if (strncmp(local_qr, "TEST:", 5) == 0)
                HandleTestCommand(local_qr);
            else
                ProcessQRCode(local_qr);
        }

        if (qr_overflow != 0U)
        {
            __disable_irq();
            qr_overflow = 0U;
            __enable_irq();

            printf("QR ERROR: Code is too long\r\n");
        }

        /* IR module output is active LOW */
        ir_now = (uint8_t)HAL_GPIO_ReadPin(GPIOA, IR_SENSOR_PIN);

        /* Indicator LED turns on while an object is detected */
        HAL_GPIO_WritePin(GPIOA, IR_LED_PIN,
                          (ir_now == IR_DETECTED_LEVEL) ?
                          GPIO_PIN_SET : GPIO_PIN_RESET);

        if (auto_state == AUTO_OFF)
        {
            /* Manual mode: old behavior, only the transition CLEAR -> DETECTED */
            if ((ir_previous != IR_DETECTED_LEVEL) &&
                (ir_now == IR_DETECTED_LEVEL))
            {
                printf("IR: PRODUCT DETECTED\r\n");
                PerformQC();
            }
        }
        else
        {
            /* Automatic stop-and-measure cycle */
            AutoProcess((ir_now == IR_DETECTED_LEVEL) ? 1U : 0U);
        }

        ir_previous = ir_now;

        HAL_Delay(20);
    }
}

/* TIM2: 16 MHz clock / (15 + 1) = 1 MHz, 1 tick = 1 us */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 15;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
        Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
        Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
        Error_Handler();
}

/* TIM3 PWM: 16 MHz clock / (15 + 1) = 1 MHz, 1 tick = 1 us.
   Period 19999 -> 20000 us = 50 Hz.
   CH1 = PA6 (Diverter), CH2 = PA7 (Conveyor).
   GPIO setup for PA6/PA7 is in HAL_TIM_PWM_MspInit (stm32g0xx_hal_msp.c). */
static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 15;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 19999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
        Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
        Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
        Error_Handler();

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;                       /* no signal until commanded */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
        Error_Handler();
}

/* I2C1: PB8 = SCL, PB9 = SDA */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00303D5B;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
        Error_Handler();

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
        Error_Handler();

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
        Error_Handler();
}

/* USART2: PA2 = TX, PA3 = RX, 115200 8N1 */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart2) != HAL_OK)
        Error_Handler();
}

/* GPIO configuration */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Initial output states */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | IR_LED_PIN, GPIO_PIN_RESET);

    /* PA0: HC-SR04 TRIG */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA1: HC-SR04 ECHO input */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA4: IR digital output input */
    GPIO_InitStruct.Pin = IR_SENSOR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA5: external indicator LED output.
       Use a suitable series resistor with the LED. */
    GPIO_InitStruct.Pin = IR_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB8 = I2C1 SCL, PB9 = I2C1 SDA */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PA2 = USART2 TX, PA3 = USART2 RX */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* HSI 16 MHz system clock */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        Error_Handler();

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
