#pragma once

// Checks the DFU button (active low) for a press edge and reboots straight
// into the USB DFU bootloader if pressed. Call once per loop() iteration.
void checkDfuButton();

// Non-blocking check for a "dfu" line on Serial; reboots into DFU mode if
// received. Call once per loop() iteration.
void checkSerialForDFU();

// Reboots straight into the STM32 system memory (USB DFU) bootloader, e.g. so a
// PlatformIO/dfu-util upload can find the device without a manual BOOT0+reset.
// Never returns.
void rebootToDFU();
