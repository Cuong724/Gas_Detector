#include "stm32f4xx.h"
#include <math.h>
#include <stdio.h>

#define VREF        3.3f
#define ADC_MAX 	4095.0f
#define RL_VALUE    10000.0f   // 10k Ohm
#define R0_VALUE    10000.0f   // cần hiệu chuẩn trong không khí sạch
#define A_CONST     1000.0f    // hệ số a (thay đổi theo loại khí)
#define B_CONST    	-2.0f      // hệ số b
#define LCD_ADDR (0x27 << 1)
#define SW1_PIN 13
#define SW2_PIN 14

void GPIOC_Init(void);
void delay_ms(uint32_t ms);
void I2C1_Init(void);
void I2C1_Write(uint8_t addr, uint8_t *data, uint8_t len);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_init(void);
void lcd_puts(char *str);
void lcd_gotoxy(uint8_t col, uint8_t row);
void ADC1_Init(void);
void LED_Init(void);
void LED_Set(uint8_t red, uint8_t yellow, uint8_t blue, uint8_t green);
void Timer2_Init(void);
void TIM2_IRQHandler(void);
void Buzzer_Init(void);
void Buzzer_Set(uint8_t on);
void Relay_Init(void);
void Relay_Set(uint8_t on);
void UART1_Init(void);
void UART1_SendChar(char c);
void UART1_SendString(char *str);
uint16_t MQ2_Read(void);
uint8_t read_SW(uint8_t pin);
volatile uint8_t system_active = 0;  // 0: stop, 1: active
volatile uint8_t led_blink_flag = 0;
volatile uint8_t blink_1hz_toggle = 0;
volatile uint8_t blink_fast_toggle = 0;
uint8_t fast_counter = 0;
uint8_t fast_interval = 5; // sẽ đổi theo ppm
uint8_t alert_status = 0;



int main(void)
{
    I2C1_Init();        // Khởi tạo I2C
    lcd_init();         // Khởi tạo LCD
    ADC1_Init();        // Khởi tạo ADC
    GPIOC_Init();       // Khởi tạo nút nhấn
    LED_Init();         // Khởi tạo LED rời
    Timer2_Init();      // Khởi tạo Timer ngắt 10ms
    Buzzer_Init();		// Khởi tạo còi
    Relay_Init();		// Khởi tạo rơ le
    UART1_Init();		// Khởi tạo UART1

    lcd_gotoxy(0, 0);
    lcd_puts("Gas:    ppm");
    lcd_gotoxy(0, 1);
    lcd_puts("TT:0 CB:0");

    char buffer[16];
    uint8_t sw1_last = 0, sw2_last = 0;
    static uint8_t alert_sent = 0;

    while (1) {
        uint8_t sw1_now = read_SW(SW1_PIN);
        uint8_t sw2_now = read_SW(SW2_PIN);
        uint16_t gas = 0;

        // Xử lý nút nhấn SW1 bật/tắt hệ thống
        if (sw1_now && !sw1_last) {
            system_active ^= 1;
        }

        // Xử lý nút nhấn SW2 reset hệ thống
        if (sw2_now && !sw2_last) {
            system_active = 0;
            alert_status = 0;
            lcd_gotoxy(0, 0);
            lcd_puts("Gas:     ppm     ");
            lcd_gotoxy(0, 1);
            lcd_puts("TT:0 CB:0       ");
            LED_Set(0, 0, 0, 1); // Xanh lá
            delay_ms(200);
        }

        sw1_last = sw1_now;
        sw2_last = sw2_now;

        if (system_active) {
            gas = MQ2_Read();
            sprintf(buffer, "PPM:%d\n", gas);
            UART1_SendString(buffer);

            if (gas >= 400 && !alert_sent) {
                UART1_SendString("ALERT\n");
                alert_sent = 1;
            }
            else if (gas < 400) {
                alert_sent = 0;
            }

            // Xác định mức cảnh báo và điều khiển LED
            if (gas < 200) {
                alert_status = 0;
                LED_Set(0, 0, 1, 0); // Xanh dương
                Buzzer_Set(0);
                Relay_Set(1);
            }
            else if (gas < 400) {
                alert_status = 1;
                LED_Set(0, 1, 0, 0); // Vàng
                Buzzer_Set(0);
                Relay_Set(1);
            }
            else if (gas < 600) {
                alert_status = 2;
                LED_Set(blink_1hz_toggle, 0, 0, 0); // Đỏ nháy 1Hz
                Buzzer_Set(0);
                Relay_Set(1);
            }
            else {
                alert_status = 3;
                float freq = 2.0f + ((float)(gas - 600) * 18.0f / 400.0f);
                if (freq > 20.0f) freq = 20.0f;

                fast_interval = (uint8_t)(1000.0f / (2.0f * freq) / 10.0f);
                if (fast_interval == 0) fast_interval = 1;

                LED_Set(blink_fast_toggle, 0, 0, 0); // LED đỏ nháy nhanh
                Buzzer_Set(1); //  Bật còi
                Relay_Set(0);
            }


            // Hiển thị lên LCD
            lcd_gotoxy(0, 0);
            sprintf(buffer, "Gas:%4d ppm", gas);
            lcd_puts(buffer);

            lcd_gotoxy(0, 1);
            sprintf(buffer, "TT:%d CB:%d", system_active, alert_status);
            lcd_puts(buffer);
        }
        else {
            alert_status = 0;
            LED_Set(0, 0, 0, 1); // Xanh lá cây
            lcd_gotoxy(0, 1);
            sprintf(buffer, "TT:%d CB:%d", system_active, alert_status);
            lcd_puts(buffer);
            Buzzer_Set(0);
            Relay_Set(1);
        }

        delay_ms(10); // kiểm tra mỗi 10ms
    }
}

void UART1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA9 (TX), PA10 (RX)
    GPIOA->MODER &= ~((3 << (2 * 9)) | (3 << (2 * 10)));
    GPIOA->MODER |= (2 << (2 * 9)) | (2 << (2 * 10));  // Alternate
    GPIOA->AFR[1] |= (7 << (4 * 1)) | (7 << (4 * 2));  // AF7 for USART1

    USART1->BRR = 16000000 / 9600; // Assuming 16 MHz clock
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // TX + RX + Enable
}

void UART1_SendChar(char c)
{
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = c;
}

void UART1_SendString(char *str)
{
    while (*str) {
        UART1_SendChar(*str++);
    }
}


void LED_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // PA1–PA4 output LED
    GPIOA->MODER &= ~((3 << (2 * 1)) | (3 << (2 * 2)) |
                      (3 << (2 * 3)) | (3 << (2 * 4)));
    GPIOA->MODER |=  (1 << (2 * 1)) | (1 << (2 * 2)) |
                     (1 << (2 * 3)) | (1 << (2 * 4));
}

void LED_Set(uint8_t red, uint8_t yellow, uint8_t blue, uint8_t green)
{
    if (red)    GPIOA->ODR |= (1 << 1); else GPIOA->ODR &= ~(1 << 1);
    if (yellow) GPIOA->ODR |= (1 << 2); else GPIOA->ODR &= ~(1 << 2);
    if (blue)   GPIOA->ODR |= (1 << 3); else GPIOA->ODR &= ~(1 << 3);
    if (green)  GPIOA->ODR |= (1 << 4); else GPIOA->ODR &= ~(1 << 4);
}


void Timer2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 16000 - 1;   // Prescaler: 16MHz / 16000 = 1kHz
    TIM2->ARR = 100 - 1;     // Tạo ngắt mỗi 100ms (1000Hz / 100)

    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;      // Bắt đầu timer

    NVIC_EnableIRQ(TIM2_IRQn);     // Cho phép ngắt TIM2 trong NVIC
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;  // Xóa cờ ngắt

        static uint8_t tick_500ms = 0;
        static uint8_t tick_fast = 0;

        // Nháy LED 1Hz (500ms)
        if (++tick_500ms >= 5) {
            blink_1hz_toggle ^= 1;
            tick_500ms = 0;
        }

        // Nháy nhanh từ 2–10Hz
        if (++tick_fast >= fast_interval) {
            blink_fast_toggle ^= 1;
            tick_fast = 0;
        }
    }
}

void Buzzer_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~(3 << (2 * 5)); // Xóa mode
    GPIOA->MODER |=  (1 << (2 * 5)); // Output mode
}

void Buzzer_Set(uint8_t on)
{
    if (on)
        GPIOA->ODR |= (1 << 5);  // Bật buzzer
    else
        GPIOA->ODR &= ~(1 << 5); // Tắt buzzer
}

void Relay_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Bật clock GPIOB

    GPIOB->MODER &= ~(3 << (2 * 0));  // Xóa mode PB0
    GPIOB->MODER |=  (1 << (2 * 0));  // Output mode
}

void Relay_Set(uint8_t on)
{
    if (on)
        GPIOB->ODR |= (1 << 0);  // Bật nguồn PB0
    else
        GPIOB->ODR &= ~(1 << 0); // Tắt nguồn PB0
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 4000; i++) __NOP();
}

void I2C1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB6 (SCL), PB7 (SDA)
    GPIOB->MODER &= ~(GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk);
    GPIOB->MODER |= (2 << GPIO_MODER_MODE6_Pos) | (2 << GPIO_MODER_MODE7_Pos); // Alternate Function

    GPIOB->AFR[0] &= ~((0xF << GPIO_AFRL_AFSEL6_Pos) | (0xF << GPIO_AFRL_AFSEL7_Pos));
    GPIOB->AFR[0] |= (4 << GPIO_AFRL_AFSEL6_Pos) | (4 << GPIO_AFRL_AFSEL7_Pos); // AF4

    GPIOB->OTYPER |= GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7;
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6_Msk | GPIO_PUPDR_PUPD7_Msk);
    GPIOB->PUPDR |= (1 << GPIO_PUPDR_PUPD6_Pos) | (1 << GPIO_PUPDR_PUPD7_Pos); // Pull-up

    // Init I2C
    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR2 = 16;       // APB1 = 16 MHz
    I2C1->CCR = 80;       // 100kHz mode
    I2C1->TRISE = 17;     // 1000ns / (1/16MHz) + 1
    I2C1->CR1 |= I2C_CR1_PE;
}


void I2C1_Write(uint8_t addr, uint8_t *data, uint8_t len)
{
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = addr & ~0x01; // Write
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2; // Clear ADDR

    for (int i = 0; i < len; i++) {
        while (!(I2C1->SR1 & I2C_SR1_TXE));
        I2C1->DR = data[i];
    }

    while (!(I2C1->SR1 & I2C_SR1_BTF));
    I2C1->CR1 |= I2C_CR1_STOP;
}



void lcd_send_cmd(char cmd)
{
    char u = cmd & 0xF0;
    char l = (cmd << 4) & 0xF0;
    uint8_t data[4] = {
        u | 0x0C, u | 0x08,
        l | 0x0C, l | 0x08
    };
    I2C1_Write(LCD_ADDR, data, 4);
}

void lcd_send_data(char data_char)
{
    char u = data_char & 0xF0;
    char l = (data_char << 4) & 0xF0;
    uint8_t data[4] = {
        u | 0x0D, u | 0x09,
        l | 0x0D, l | 0x09
    };
    I2C1_Write(LCD_ADDR, data, 4);
}

void lcd_init(void)
{
    delay_ms(50);
    lcd_send_cmd(0x30); delay_ms(5);
    lcd_send_cmd(0x30); delay_ms(1);
    lcd_send_cmd(0x30); delay_ms(10);
    lcd_send_cmd(0x20); delay_ms(10);

    lcd_send_cmd(0x28); delay_ms(1);  // Function set
    lcd_send_cmd(0x08); delay_ms(1);  // Display off
    lcd_send_cmd(0x01); delay_ms(2);  // Clear
    lcd_send_cmd(0x06); delay_ms(1);  // Entry mode
    lcd_send_cmd(0x0C); delay_ms(1);  // Display on
}

void lcd_puts(char *str)
{
    while (*str) {
        lcd_send_data(*str++);
    }
}

void lcd_gotoxy(uint8_t col, uint8_t row)
{
    uint8_t address;

    switch (row)
    {
        case 0: address = 0x80 + col; break;  // Dòng 1
        case 1: address = 0xC0 + col; break;  // Dòng 2
        case 2: address = 0x94 + col; break;  // Dòng 3
        case 3: address = 0xD4 + col; break;  // Dòng 4
        default: return;  // Sai row thì không gửi lệnh
    }

    lcd_send_cmd(address);  // Gửi lệnh set DDRAM address

}
void ADC1_Init(void)
    {
        // Bật clock cho GPIOA và ADC1
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

        // PA0 (ADC1_IN0) thành analog
        GPIOA->MODER |= (3 << GPIO_MODER_MODE0_Pos); // Analog mode

        // Cấu hình ADC
        ADC1->CR2 = 0;                  // Tắt ADC trước khi cấu hình
        ADC1->SQR3 = 0;                 // Channel 0 (PA0)
        ADC1->SMPR2 |= (7 << ADC_SMPR2_SMP0_Pos); // Chọn thời gian lấy mẫu dài
        ADC1->CR2 |= ADC_CR2_ADON;      // Bật ADC
    }

void GPIOC_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // PC13 và PC14 làm input, pull-up
    GPIOC->MODER &= ~((3 << (2 * SW1_PIN)) | (3 << (2 * SW2_PIN)));
    GPIOC->PUPDR &= ~((3 << (2 * SW1_PIN)) | (3 << (2 * SW2_PIN)));
    GPIOC->PUPDR |=  (1 << (2 * SW1_PIN)) | (1 << (2 * SW2_PIN));  // Pull-up
}

uint8_t read_SW(uint8_t pin)
{
    return !(GPIOC->IDR & (1 << pin));
}

uint16_t MQ2_Read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    uint16_t adc_val = ADC1->DR;

    float voltage = (adc_val * VREF) / ADC_MAX;
    float rs = (VREF - voltage) * RL_VALUE / voltage;
    float ratio = rs / R0_VALUE;
    float ppm = A_CONST * powf(ratio, B_CONST);

    // Giới hạn ppm từ 0 đến 1000
    if (ppm < 0) ppm = 0;
    if (ppm > 1000) ppm = 1000;
    return (uint16_t)ppm;
}
