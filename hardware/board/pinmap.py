"""STM32G474RET6 pin assignment and the alternate-function facts it relies on.

AF numbers are from the author's knowledge of the STM32G474 datasheet (DS12288, "Alternate function"
tables) - NOT from a downloaded copy of the datasheet. `confidence`:
  high    - widely used mapping, same as on STM32F3/G4 Nucleo boards; still tick it off in CubeMX
  medium  - believed correct, MUST be confirmed in CubeMX / DS12288 before layout
Every medium entry is listed in board/out/mcu_pins.md and in the design-review checklist.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Assign:
    signal: str        # net name in the schematic
    pin: str           # MCU pin name as in library.MCU_PADS
    function: str      # peripheral function
    af: str            # "AF7", "analog", "GPIO", "system", ...
    confidence: str    # high / medium
    note: str = ""


ASSIGNMENTS = [
    Assign("SERVO_TX", "PB10", "USART3_TX", "AF7", "high", "servo bus TX -> 74AHCT1G126 A"),
    Assign("SERVO_RX", "PB11", "USART3_RX", "AF7", "high", "servo bus RX <- 74LVC1G125 Y"),
    Assign("SERVO_DIR", "PB14", "USART3_DE (RTS/DE, DEM=1)", "AF7", "medium",
           "hardware driver-enable: high during TX; drives both buffer OEs. If not available, use PB14 as GPIO "
           "toggled in the TC interrupt"),
    Assign("LOG_TX", "PA9", "USART1_TX", "AF7", "high", "debug log; USART1 PA9/PA10 is also a ROM-bootloader port"),
    Assign("LOG_RX", "PA10", "USART1_RX", "AF7", "high", ""),
    Assign("USB_DM", "PA11", "USB_DM", "USB (fixed pin)", "medium",
           "USB FS pins are fixed; confirm in RM0440 whether an AF selection is needed (CubeMX does it)"),
    Assign("USB_DP", "PA12", "USB_DP", "USB (fixed pin)", "medium", "internal DP pull-up, no external 1.5k"),
    Assign("CAN_RXD", "PB12", "FDCAN2_RX", "AF9", "medium",
           "FDCAN1 is avoided: PA11/12 clash with USB, PB8 is BOOT0 (CAN RXD idles high -> would boot ROM)"),
    Assign("CAN_TXD", "PB13", "FDCAN2_TX", "AF9", "medium", ""),
    Assign("CAN_STB", "PC9", "GPIO output", "GPIO", "high", "TCAN1462 standby; 10k pull-up = standby at reset"),
    Assign("ISENSE_ADC", "PA0", "ADC12_IN1", "analog", "high", "INA240 output via 100R/1nF"),
    Assign("VSERVO_ADC", "PA1", "ADC12_IN2", "analog", "high", "servo-rail divider 100k/15k"),
    Assign("NTC_ADC", "PC0", "ADC12_IN6", "analog", "medium", "board NTC"),
    Assign("VIN_ADC", "PC1", "ADC12_IN7", "analog", "medium", "protected 12 V divider 100k/15k"),
    Assign("LED1_DRV", "PB0", "TIM3_CH3", "AF2", "high", "status LED (PWM-able)"),
    Assign("LED2_DRV", "PB1", "TIM3_CH4", "AF2", "high", "status LED (PWM-able)"),
    Assign("SERVO_PWR_EN", "PC6", "GPIO output", "GPIO", "high", "firmware enable of servo power (AND-ed with e-stop)"),
    Assign("ESTOP_SENSE", "PC7", "GPIO input / EXTI7", "GPIO", "high", "e-stop loop state, high = loop closed"),
    Assign("VBUS_SENSE", "PC8", "GPIO input", "GPIO", "high", "USB VBUS present (divider 10k/20k)"),
    Assign("SWDIO", "PA13", "SWDIO", "AF0 (reset default)", "high", ""),
    Assign("SWCLK", "PA14", "SWCLK", "AF0 (reset default)", "high", ""),
    Assign("SWO", "PB3", "TRACESWO", "AF0", "high", "PB3 is also JTDO"),
    Assign("OSC_IN", "PF0-OSC_IN", "RCC_OSC_IN (HSE)", "system", "high", "24 MHz crystal"),
    Assign("OSC_OUT", "PF1-OSC_OUT", "RCC_OSC_OUT (HSE)", "system", "high", ""),
    Assign("NRST", "PG10-NRST", "NRST", "system", "high", "100 nF to GND + reset button"),
    Assign("BOOT0", "PB8-BOOT0", "BOOT0", "system", "high",
           "10k pull-down + BOOT button; nSWBOOT0/nBOOT0 option bytes decide whether the pin is sampled"),
]

AVOIDED = {
    "PB4": "UCPD1_CC2 dead-battery pull-down active at reset (also NJTRST)",
    "PB6": "UCPD1_CC1 dead-battery pull-down active at reset",
    "PB8": "shared with BOOT0 - only BOOT0 use",
    "PC14/PC15": "LSE pins, weak drive",
    "PC13": "RTC/tamper domain, weak drive (<3 mA)",
}

# Known AF facts used above (pin, function) -> AF. A test checks every AF-mapped assignment is listed here.
AF_FACTS = {
    ("PB10", "USART3_TX"): "AF7", ("PB11", "USART3_RX"): "AF7", ("PB14", "USART3_DE (RTS/DE, DEM=1)"): "AF7",
    ("PA9", "USART1_TX"): "AF7", ("PA10", "USART1_RX"): "AF7",
    ("PB12", "FDCAN2_RX"): "AF9", ("PB13", "FDCAN2_TX"): "AF9",
    ("PB0", "TIM3_CH3"): "AF2", ("PB1", "TIM3_CH4"): "AF2",
    ("PA13", "SWDIO"): "AF0 (reset default)", ("PA14", "SWCLK"): "AF0 (reset default)", ("PB3", "TRACESWO"): "AF0",
}
