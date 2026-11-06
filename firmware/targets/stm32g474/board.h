/* Pin and peripheral assignment. Bring-up target is a NUCLEO-G474RE (plan milestone M1); the
 * custom board (hardware/) should keep these assignments or update this file only.
 *
 *   Host link (bring-up)  LPUART1  PA2 TX / PA3 RX, AF12, to the ST-LINK virtual COM port
 *   Host link (board)     USB FS CDC  PA11 / PA12 (see usb_cdc_glue.c, stub)
 *   Servo bus             USART1 single-wire half-duplex on PA9 (TX pin), AF7, 1 Mbps,
 *                         through a 74LVC2G241-style buffer; PA8 = buffer direction (1 = TX)
 *   E-stop loop sense     PB12 input, pull-down; high = loop closed. EXTI12 falling edge.
 *   Servo rail current    PA0 = ADC1_IN1 (INA240 output)     [ADC: sketched, see hal]
 *   Servo rail voltage    PA1 = ADC1_IN2 (divider 12 V -> ~2 V)
 *   Status LED            PA5 (NUCLEO LD2)
 *
 * DMA (DMA1 via DMAMUX1): ch1 LPUART1_RX (circular), ch2 LPUART1_TX,
 *                         ch3 USART1_TX, ch4 USART1_RX (circular)
 * Timers: TIM6 = 1 kHz control tick (update IRQ), TIM2 = 32-bit 1 MHz timebase (time_us).
 * IRQ priorities (lower = more urgent): EXTI e-stop 0, TIM6 control 1, USART1 TC 2, DMA/LPUART 3.
 */
#ifndef BOARD_H
#define BOARD_H

#define BOARD_SYSCLK_HZ 170000000u
#define BOARD_HOST_BAUD 1000000u
#define BOARD_SERVO_BAUD 1000000u

#define BOARD_HOST_RX_RING 1024u
#define BOARD_HOST_TX_BUF 1024u
#define BOARD_SERVO_RX_RING 256u
#define BOARD_SERVO_TX_BUF 160u

#endif
