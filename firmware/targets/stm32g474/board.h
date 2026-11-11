/* Pin and peripheral assignment. Bring-up target is a NUCLEO-G474RE (plan milestone M1); the
 * custom board (hardware/) should keep these assignments or update this file only.
 *
 *   Host link (bring-up)  LPUART1  PA2 TX / PA3 RX, AF12, to the ST-LINK virtual COM port
 *   Host link (board)     USB FS CDC  PA11 / PA12 (usb_cdc_glue.c, TinyUSB), default build
 *                         (-DARMFW_HOST_LINK_USB=OFF selects LPUART1 instead)
 *   Servo bus             USART1 single-wire half-duplex on PA9 (TX pin), AF7, 1 Mbps,
 *                         through a 74LVC2G241-style buffer; PA8 = buffer direction (1 = TX)
 *   E-stop loop sense     PB12 input, pull-down; high = loop closed. EXTI12 falling edge.
 *   Analog (adc_stm32g4.c, channels per hardware/board/out/mcu_pins.md):
 *                         PA0 = ADC1_IN1 INA240 servo-rail current, PA1 = ADC1_IN2 servo rail
 *                         100k/15k, PC0 = ADC1_IN6 board NTC, PC1 = ADC1_IN7 protected 12 V
 *                         100k/15k, + VREFINT (ADC1_IN18). DMA1 ch5 (DMAMUX ch4), circular.
 *   Calibration flash     bank 2 pages 126/127 (0x0807F000, 0x0807F800), flash_stm32g4.c
 *   Status LED            PA5 (NUCLEO LD2)
 *
 * DMA (DMA1 via DMAMUX1): ch1 LPUART1_RX (circular), ch2 LPUART1_TX,
 *                         ch3 USART1_TX, ch4 USART1_RX (circular)
 * Timers: TIM6 = 1 kHz control tick (update IRQ), TIM2 = 32-bit 1 MHz timebase (time_us).
 * IRQ priorities (lower = more urgent): EXTI e-stop 0, TIM6 control 1, USART1 TC 2, DMA/LPUART 3.
 */
/* NOTE: the servo bus, e-stop and LED pins above are the NUCLEO-G474RE bring-up assignment. The
 * custom controller board (hardware/board/out/mcu_pins.md) differs: servo bus on USART3
 * PB10/PB11 with DE on PB14, e-stop sense PC7, servo power enable PC6, LEDs PB0/PB1, debug log on
 * USART1 PA9/PA10. Only the analog channels and USB already follow the board pin map. */
#ifndef BOARD_H
#define BOARD_H

#define BOARD_SYSCLK_HZ 170000000u
#define BOARD_HOST_BAUD 1000000u
#define BOARD_SERVO_BAUD 1000000u

#define BOARD_HOST_RX_RING 1024u
#define BOARD_HOST_TX_BUF 1024u
#define BOARD_SERVO_RX_RING 256u
#define BOARD_SERVO_TX_BUF 160u
#define BOARD_USB_RX_RING 1024u /* host_link rings, powers of two */
#define BOARD_USB_TX_RING 2048u

#endif
