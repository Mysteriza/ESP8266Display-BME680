# ESP8266 IRAM Overflow Fix - Complete Guide

## Problem

When compiling ESP8266 projects with large libraries (especially **Bosch BSEC** for BME680), you get this error:

```
c:/users/.../xtensa-lx106-elf/bin/ld.exe: 
ESP8266Display-BME680.ino.elf section `.text1' will not fit in region `iram1_0_seg'
collect2.exe: error: ld returned 1 exit status
exit status 1
Compilation error: exit status 1
```

**Root Cause:** The ESP8266 has limited IRAM (Instruction RAM). The default configuration only allocates **32KB IRAM**, which is insufficient for large libraries like Bosch BSEC + Adafruit GFX + OLED drivers.

---

## Solution Overview

This fix requires **TWO changes**:

1. ✅ **Change Flash Size** → From 1MB to 4MB
2. ✅ **Change MMU Setting** → Enable 16KB Cache + 48KB IRAM

Both must be applied together for the fix to work.

---

## Method 1: Via Arduino IDE (Recommended for Beginners)

### Step 1: Change Flash Size

1. Open your project in **Arduino IDE**
2. Go to **Tools** menu
3. Find **Flash Size**
4. Change from `1M (64K SPIFFS)` → **`4M (3M SPIFFS)`** or `4M (1M SPIFFS)`

### Step 2: Change MMU Setting

1. Still in **Tools** menu
2. Find **MMU** (Memory Management Unit)
3. Select **`16KB cache + 48KB IRAM`** (sometimes labeled as `48KB IRAM, 16KB cache`)

### Step 3: Restart Arduino IDE

**Important:** You must restart Arduino IDE for changes to take effect.

### Step 4: Upload Again

Compile and upload your sketch. The IRAM overflow error should be resolved.

---

## Method 2: Via VS Code / PlatformIO (Advanced)

### Step 1: Update arduino.json

Open `.vscode/arduino.json` and change the `eesz` parameter:

**Before:**
```json
"configuration": "...eesz=1M64,mmu=3232..."
```

**After:**
```json
"configuration": "...eesz=4M3M,mmu=4816..."
```

**Key Changes:**
- `eesz=1M64` → `eesz=4M3M` (Flash: 1MB → 4MB)
- `mmu=3232` → `mmu=4816` (MMU: 32KB IRAM → 48KB IRAM)

### Step 2: Reload VS Code Window

Press `Ctrl+Shift+P` → Type "Reload Window" → Press Enter

### Step 3: Upload

Use Arduino CLI or upload normally.

---

## Method 3: Manual platform.txt Replacement (If Methods 1-2 Fail)

Sometimes the MMU option doesn't appear in Arduino IDE. This means you need to update the `platform.txt` file manually.

### Step 1: Locate platform.txt

Navigate to this folder (Windows):

```
C:\Users\<YOUR_USERNAME>\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\<VERSION>\
```

**Example:**
```
C:\Users\rifqi\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\3.1.2\
```

**macOS/Linux:**
```
~/.arduino15/packages/esp8266/hardware/esp8266/<VERSION>/
```

### Step 2: Backup Original

**IMPORTANT:** Before making changes, backup the original file!

```
Copy: platform.txt
To:   platform.txt.backup
```

### Step 3: Replace platform.txt

1. Open the `platform.txt` file provided in this project (included in project root)
2. Copy the **entire content**
3. Paste it into the original `platform.txt` at the Arduino15 location
4. Save the file

### Step 4: Verify MMU Options Appear

1. Open Arduino IDE
2. Go to **Tools** → **MMU**
3. You should now see these options:
   - `32KB cache + 32KB IRAM (balanced)` ← Default
   - **`16KB cache + 48KB IRAM`** ← **Select this one**
   - `No cache + 64KB IRAM (legacy)`

### Step 5: Select Correct MMU

Select: **`16KB cache + 48KB IRAM`**

### Step 6: Restart Arduino IDE

Close and reopen Arduino IDE completely.

### Step 7: Upload

Compile and upload. The error should be resolved.

---

## Complete Settings Summary

### For Arduino IDE:

| Setting | Value |
|---------|-------|
| **Board** | NodeMCU 1.0 (ESP-12E Module) |
| **Flash Size** | `4M (3M SPIFFS)` or `4M (1M SPIFFS)` |
| **MMU** | `16KB cache + 48KB IRAM` |
| **Upload Speed** | `115200` |
| **CPU Frequency** | `80 MHz` |
| **Flash Mode** | `DOUT` |
| **Flash Frequency** | `40 MHz` |
| **Debug Port** | `Disabled` |
| **Debug Level** | `None` |

### For VS Code (arduino.json):

```json
{
    "board": "esp8266:esp8266:generic",
    "configuration": "xtal=80,vt=flash,exception=disabled,stacksmash=disabled,ssl=all,mmu=4816,non32xfer=fast,ResetMethod=nodemcu,CrystalFreq=26,FlashFreq=40,FlashMode=dout,eesz=4M3M,led=2,sdk=nonosdk_190703,ip=lm2f,dbg=Disabled,lvl=None____,wipe=none,baud=115200",
    "port": "COM3",
    "sketch": "ESP8266Display-BME680.ino"
}
```

**Key parameters:**
- `eesz=4M3M` (Flash size)
- `mmu=4816` (48KB IRAM, 16KB cache)

---

## Troubleshooting

### Q: MMU option doesn't appear in Arduino IDE

**A:** Follow **Method 3** above to manually replace `platform.txt`.

### Q: Still getting IRAM overflow after applying fix

**A:** Verify these:
1. ✅ Flash Size = 4MB (not 1MB)
2. ✅ MMU = 16KB cache + 48KB IRAM
3. ✅ Restarted Arduino IDE after changes
4. ✅ Selected correct board (NodeMCU 1.0)

### Q: Will this damage my ESP8266?

**A:** **No.** These are safe configuration changes. Most ESP8266 modules have 4MB flash physically, but default to 1MB in software. This just unlocks the full capacity.

### Q: Can I use this fix for other large libraries?

**A:** **Yes!** This fix works for any library that causes IRAM overflow:
- Bosch BSEC
- TFT display libraries
- Audio libraries
- WiFi + MQTT combinations
- Large graphics libraries

### Q: What's the difference between MMU options?

| MMU Setting | IRAM | Cache | Best For |
|-------------|------|-------|----------|
| 32KB cache + 32KB IRAM | 32KB | 32KB | Small projects |
| **16KB cache + 48KB IRAM** | **48KB** | **16KB** | **Large libraries (BSEC, etc.)** |
| No cache + 64KB IRAM | 64KB | 0KB | Legacy code (not recommended) |

**Recommendation:** Use `16KB cache + 48KB IRAM` for most projects with BSEC.

---

## Technical Explanation

### What is IRAM?

**IRAM (Instruction RAM)** is where the ESP8266 stores code that needs to run quickly. It's limited to:
- **Default:** 32KB
- **With fix:** 48KB (50% more!)

### Why does BSEC need so much IRAM?

The Bosch BSEC library is a **precompiled binary** that contains:
- Sensor fusion algorithms
- IAQ calculation engine
- Calibration routines
- Temperature/humidity compensation

All of this requires significant memory.

### What does MMU do?

**MMU (Memory Management Unit)** controls how flash memory is mapped:
- **Cache:** Stores frequently-used code for faster access
- **IRAM:** Stores critical code that must run immediately

More IRAM = more space for large libraries.

---

## Credits

- **Original Solution:** ESP8266 Arduino Core community
- **Documented by:** rifqi
- **Tested with:** ESP8266 Core 3.1.2, Bosch BSEC Library
- **Date:** 2026-04-11

---

## Related Files

- `platform.txt` - Fixed platform configuration (can replace Arduino15 version)
- `Fix RAM ESP8266.txt` - Original fix notes
- `README.md` - Main project documentation

---

## License

This documentation is provided as-is for educational purposes.
