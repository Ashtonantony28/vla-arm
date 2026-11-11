#!/usr/bin/env python3
"""Check targets/stm32g474/stm32g4_min.h (register definitions written from memory) against ST's
CMSIS device header (third_party/cmsis_device_g4) and Arm CMSIS-Core.

Every numeric #define, peripheral base, struct field offset and IRQ number in the minimal header
is evaluated twice by arm-none-eabi-gcc: once with the minimal header, once with ST's. Values are
read back from the compiled objects (no target execution), and any difference is reported.
Definitions that ST's CMSIS header does not contain (DMAMUX request IDs live in the HAL/LL
headers) are listed as unverifiable.

usage: tools/check_regs.py            exit status 1 if anything differs
"""
import os
import re
import struct
import subprocess
import sys
import tempfile

FW = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TGT = os.path.join(FW, "targets", "stm32g474")
TP = os.path.join(FW, "third_party")
CC = "arm-none-eabi-gcc"
OBJCOPY = "arm-none-eabi-objcopy"
FLAGS = ["-mcpu=cortex-m4", "-mthumb", "-std=gnu11", "-O0", "-c", "-w"]

# names whose CMSIS counterpart is spelled differently: name -> (min expression, cmsis expression)
SPECIAL = {
    # core peripherals (CMSIS-Core)
    "NVIC_ISER": ("(uintptr_t)NVIC_ISER", "(uintptr_t)&NVIC->ISER[0]"),
    "NVIC_ICER": ("(uintptr_t)NVIC_ICER", "(uintptr_t)&NVIC->ICER[0]"),
    "NVIC_IPR": ("(uintptr_t)NVIC_IPR", "(uintptr_t)&NVIC->IP[0]"),
    "SCB_CPACR": ("(uintptr_t)&SCB_CPACR", "(uintptr_t)&SCB->CPACR"),
    "SCB_VTOR": ("(uintptr_t)&SCB_VTOR", "(uintptr_t)&SCB->VTOR"),
    "SCB_AIRCR": ("(uintptr_t)&SCB_AIRCR", "(uintptr_t)&SCB->AIRCR"),
    "DWT_CTRL": ("(uintptr_t)&DWT_CTRL", "(uintptr_t)&DWT->CTRL"),
    "DWT_CYCCNT": ("(uintptr_t)&DWT_CYCCNT", "(uintptr_t)&DWT->CYCCNT"),
    "DEMCR": ("(uintptr_t)&DEMCR", "(uintptr_t)&CoreDebug->DEMCR"),
    "DEMCR_TRCENA": ("DEMCR_TRCENA", "CoreDebug_DEMCR_TRCENA_Msk"),
    "SysTick": ("(uintptr_t)SysTick", "(uintptr_t)SysTick"),
    "DMA_CCR_PL_HIGH": ("DMA_CCR_PL_HIGH", "DMA_CCR_PL_1"),
    # device registers addressed through plain pointers in the minimal header
    "FLASH_ACR": ("(uintptr_t)&FLASH_ACR", "(uintptr_t)&FLASH->ACR"),
    "PWR_CR5": ("(uintptr_t)&PWR_CR5", "(uintptr_t)&PWR->CR5"),
    "DMAMUX1_CCR": ("(uintptr_t)DMAMUX1_CCR", "DMAMUX1_Channel0_BASE"),
    "SYSCFG_EXTICR": ("(uintptr_t)SYSCFG_EXTICR", "(uintptr_t)&SYSCFG->EXTICR[0]"),
    "DMA1.CH[0]": ("(uintptr_t)&DMA1->CH[0]", "DMA1_Channel1_BASE"),
    "DMA1.CH[1]": ("(uintptr_t)&DMA1->CH[1]", "DMA1_Channel2_BASE"),
    "DMA1.CH[2]": ("(uintptr_t)&DMA1->CH[2]", "DMA1_Channel3_BASE"),
    "DMA1.CH[3]": ("(uintptr_t)&DMA1->CH[3]", "DMA1_Channel4_BASE"),
    "DMA1.CH[7]": ("(uintptr_t)&DMA1->CH[7]", "DMA1_Channel8_BASE"),
    "DMAMUX1_CCR[3]": ("(uintptr_t)&DMAMUX1_CCR[3]", "DMAMUX1_Channel3_BASE"),
    # startup_stm32g474.c: vector table size = last IRQn + 1
    "N_IRQ(startup)": ("102", "FMAC_IRQn + 1"),
}
# not in ST's CMSIS header at all (from ST's LL/HAL headers or the reference manual)
UNVERIFIABLE_PREFIX = ("DMAMUX_REQ_",)
SKIP = {"__IO", "STM32G4_MIN_H"}
PERIPHS = ["RCC", "GPIOA", "GPIOB", "GPIOC", "TIM2", "TIM6", "USART1", "LPUART1", "DMA1", "EXTI"]


def parse_min(text):
    defines, structs, enums = [], {}, []
    for m in re.finditer(r"^#define\s+([A-Za-z_][A-Za-z0-9_]*)\s+(.+?)\s*(?:/\*.*)?$", text, re.M):
        name, val = m.group(1), m.group(2)
        if name in SKIP or "(" in name:
            continue
        defines.append((name, val))
    for m in re.finditer(r"typedef struct \{(.*?)\}\s*([A-Za-z_0-9]+);", text, re.S):
        body, tname = m.group(1), m.group(2)
        fields = []
        for line in body.split(";"):
            line = re.sub(r"/\*.*?\*/", "", line).strip()
            if not line.startswith("__IO"):
                continue
            for f in line[len("__IO uint32_t"):].split(","):
                f = f.strip()
                if f:
                    fields.append(f)
        structs[tname] = fields
    em = re.search(r"enum \{(.*?)\};", text, re.S)
    if em:
        for m in re.finditer(r"([A-Za-z0-9_]+_IRQn)\s*=", em.group(1)):
            enums.append(m.group(1))
    return defines, structs, enums


def build_items(defines, structs, enums):
    items = []  # (label, min_expr, cmsis_expr or None)
    pointer_like = set(PERIPHS) | {"SysTick"}
    for name, val in defines:
        if name in SPECIAL or name in pointer_like:
            continue
        if name.startswith(UNVERIFIABLE_PREFIX):
            items.append((name, name, None))
            continue
        if "*" in val:  # a register pointer macro without a SPECIAL mapping
            items.append((name, f"(uintptr_t)&{name}", None))
            continue
        items.append((name, f"(uint32_t)({name})", f"(uint32_t)({name})"))
    for label, (a, b) in SPECIAL.items():
        items.append((label, a, b))
    for p in PERIPHS:
        items.append((p, f"(uintptr_t){p}", f"(uintptr_t){p}"))
    for e in enums:
        items.append((e, e, e))
    for tname, fields in structs.items():
        if tname == "DMA_TypeDef":
            fields = ["ISR", "IFCR"]
        for f in fields:
            fn = f.split("[")[0]
            items.append((f"{tname}.{fn}", f"offsetof({tname}, {fn})", f"offsetof({tname}, {fn})"))
    return items


def compile_values(items, which):
    src = ["#include <stddef.h>", "#include <stdint.h>"]
    if which == "min":
        src.append('#include "stm32g4_min.h"')
    else:
        src += ["#define STM32G474xx", '#include "stm32g4xx.h"']
    vals, have = [], []
    for label, mexpr, cexpr in items:
        expr = mexpr if which == "min" else cexpr
        if expr is None:
            vals.append("0xDEADBEEFu")
            have.append("0")
            continue
        # a same-named macro that the other header lacks: report instead of failing the build
        if which == "cmsis" and re.fullmatch(r"\(uint32_t\)\(([A-Za-z0-9_]+)\)", expr):
            n = re.fullmatch(r"\(uint32_t\)\(([A-Za-z0-9_]+)\)", expr).group(1)
            vals.append(f"\n#ifdef {n}\n{expr}\n#else\n0xDEADBEEFu\n#endif\n")
            have.append(f"\n#ifdef {n}\n1\n#else\n0\n#endif\n")
            continue
        vals.append(f"(uint32_t)({expr})")
        have.append("1")
    src.append('__attribute__((used, section(".chkv"))) const uint32_t chk_v[] = {' + ",".join(vals) + "};")
    src.append('__attribute__((used, section(".chkh"))) const uint32_t chk_h[] = {' + ",".join(have) + "};")
    with tempfile.TemporaryDirectory() as td:
        c = os.path.join(td, "chk.c")
        o = os.path.join(td, "chk.o")
        with open(c, "w") as f:
            f.write("\n".join(src) + "\n")
        inc = ["-I" + TGT] if which == "min" else [
            "-I" + os.path.join(TP, "cmsis_device_g4", "Include"),
            "-I" + os.path.join(TP, "cmsis_5", "CMSIS", "Core", "Include")]
        subprocess.run([CC, *FLAGS, *inc, c, "-o", o], check=True)
        out = []
        for sec in (".chkv", ".chkh"):
            b = os.path.join(td, sec[1:] + ".bin")
            subprocess.run([OBJCOPY, "-O", "binary", "-j", sec, o, b], check=True)
            data = open(b, "rb").read()
            out.append(list(struct.unpack("<%dI" % (len(data) // 4), data)))
        return out


def main():
    text = open(os.path.join(TGT, "stm32g4_min.h")).read()
    items = build_items(*parse_min(text))
    mv, _ = compile_values(items, "min")
    cv, ch = compile_values(items, "cmsis")
    bad, unver, ok = [], [], 0
    for (label, _, cexpr), a, b, h in zip(items, mv, cv, ch):
        if cexpr is None or not h:
            unver.append((label, a))
        elif a != b:
            bad.append((label, a, b))
        else:
            ok += 1
    print(f"stm32g4_min.h vs ST CMSIS (stm32g474xx.h v1.2.6 + CMSIS-Core 5.9.0): {ok} match, "
          f"{len(bad)} differ, {len(unver)} not checkable")
    for label, a, b in bad:
        print(f"  DIFFERS  {label:28s} min 0x{a:08X}  ST 0x{b:08X}")
    for label, a in unver:
        print(f"  UNCHECKED {label:27s} = {a} (not in ST's CMSIS header)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
