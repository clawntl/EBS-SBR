#include "dfu.h"

#include <Arduino.h>
#include <cstring>

#include "stm32g4xx_hal.h" // needed for HAL_DeInit / rebootToDFU

#include "pins.h"

void rebootToDFU() {
    __disable_irq();

    // Stop SysTick
    SysTick->CTRL = 0;

    // Deinit HAL (shuts down peripherals & interrupts)
    HAL_RCC_DeInit();
    HAL_DeInit();

    // ---- Disable USB (STM32G431 version) ----

    // Disable USB clock
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_USBEN;

    // Reset USB peripheral
    RCC->APB1RSTR1 |=  RCC_APB1RSTR1_USBRST;
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_USBRST;

    // Reset USB pins PA11/PA12 to analog
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Remap system memory to 0x00000000
    SYSCFG->MEMRMP = SYSCFG_MEMRMP_MEM_MODE_0;

    // System memory bootloader start
    uint32_t bootAddr = 0x1FFF0000;

    // Load stack pointer and reset vector
    uint32_t sp = *(__IO uint32_t*)(bootAddr);
    uint32_t rv = *(__IO uint32_t*)(bootAddr + 4);

    // Set main stack pointer
    __set_MSP(sp);

    // Jump to bootloader
    ((void (*)(void))rv)();
}

void checkDfuButton() {
    static bool dfuButtonPrevState = HIGH;

    const bool dfuButtonState = digitalRead(DFU_BUTTON);
    if (dfuButtonState == LOW && dfuButtonPrevState == HIGH) {
        Serial.println("Entering DFU mode...");
        rebootToDFU();
    }
    dfuButtonPrevState = dfuButtonState;
}

// Non-blocking "dfu\n" line reader (Serial.readStringUntil() would stall the
// FOC loop for up to its timeout on a stray/partial byte, so bytes are
// accumulated one at a time instead).
void checkSerialForDFU() {
    static char buf[8];
    static uint8_t len = 0;

    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (len > 0) {
                buf[len] = '\0';
                if (strcmp(buf, "dfu") == 0) {
                    Serial.println("Entering DFU mode...");
                    Serial.flush();
                    rebootToDFU();
                }
                len = 0;
            }
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        }
    }
}
