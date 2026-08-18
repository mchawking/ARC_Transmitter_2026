# ARC Project Archive Documentation

**Date:** August 9, 2026  
**Maintained by:** Todd (toddimus-prime)

---

## Active Projects

### ✅ ARC V2.1 - Aug 26 (WORKING COPY)
**Status:** ACTIVE - New development branch based on RCRA V1.1  
**Location:** `c:\Users\manea\OneDrive\Documents\PlatformIO\Projects\ARC V2.1 - Aug 26`  
**Source:** `c:\Users\manea\OneDrive\Documents\PlatformIO\Projects\RCRA V1.1 - Oct 25`

**Description:**  
Development copy of the RCRA display and control system for ARC tuning and iteration:
- ST7789 TFT Display (320x170)
- 4x AS5600 Encoder support via I2C multiplexer
- CRSF Protocol for ELRS/ExpressLRS transmission
- FRAM I2C persistence for calibration data
- Full calibration menu system
- Real-time sensor visualization with color-gradient bars
- v1.4.8 stable release

**Hardware:** ESP32-S3-DevKitC-1 (8MB Flash + 8MB PSRAM)

---

## Archived Projects (OBSOLETE - DO NOT USE)

### ⚠️ Exo_Arm_TX_CRSF
**Status:** ARCHIVED (November 20, 2025)  
**Location:** `c:\Users\manea\Platform IO Projects\Exo_Arm_TX_CRSF`  
**Archive Notice:** `ARCHIVED_README.md` added to project root

**Reason for Archival:**  
This ESP-NOW transmitter code is superseded by the CRSF-based implementation in RCRA V1.1. The ESP-NOW protocol was replaced with CRSF for better compatibility with ELRS systems.

**DO NOT USE:** This code will cause confusion as it has no display support and uses an incompatible protocol.

---

### ⚠️ Armageddon-RCRA-Hand-Display-Board
**Status:** ARCHIVED (November 20, 2025)  
**Location:** `c:\Users\manea\OneDrive\Documents\PlatformIO\Projects\Armageddon-RCRA-Hand-Display-Board`  
**Archive Notice:** `ARCHIVED_README.md` added to project root

**Reason for Archival:**  
Early prototype of the hand display board that has been fully integrated into the comprehensive RCRA V1.1 project. All functionality has been merged and improved.

**DO NOT USE:** Incomplete implementation superseded by RCRA V1.1.

---

## Migration Notes

If you encounter references to archived projects:

1. **Exo_Arm_TX_CRSF** → Use **RCRA V1.1 - Oct 25**
   - ESP-NOW replaced with CRSF protocol
   - All transmitter functionality included
   - Added display and calibration features

2. **Armageddon-RCRA-Hand-Display-Board** → Use **RCRA V1.1 - Oct 25**
   - All display functionality integrated
   - Enhanced with full sensor suite
   - Complete calibration system added

---

## Upload Safety

To avoid accidentally uploading archived code:

1. **Always verify the project name** before uploading
2. **Check serial monitor output** after upload:
   - ✅ Should see: `=== RCRA Display with CRSF TX ===`
   - ❌ If you see ESP-NOW messages, wrong code is uploaded
3. **Close serial monitors** before uploading new code
4. **Check COM port** - ensure only one board is connected

---

## Project Maintenance

**Current Version:** v2.1.0-dev  
**Last Updated:** August 9, 2026  
**Platform:** PlatformIO + Arduino ESP32  
**Framework:** Arduino

For questions or issues, refer to the main RCRA V1.1 - Oct 25 project.

---

## Milestone Log

### 2026-08-09 - USB/rig diagnosis complete + calibration preservation fix

**Status:** Milestone reached, field validation pending

**Completed today:**
1. Confirmed upload path works on bench hardware (COM7) after driver and boot-mode recovery.
2. Confirmed both ESP32 boards are healthy and flashable.
3. Isolated failure mode to rig integration conditions, not firmware image integrity.
4. Implemented bug fix in calibration flow so changing direction or gear ratio no longer zeroes stored values.
   - Direction toggle now mirrors stored calibration values.
   - Gear ratio change now rescales stored calibration values using newScale/oldScale.
5. Verified ARC project builds successfully after patch (PlatformIO build exit code 0).

**Observed hardware behavior:**
- In-rig USB detection is intermittent unless BOOT is held during attach in some cases.
- Symptom pattern strongly suggests rig-side power and/or pin-loading interference during enumeration/programming.

**Next step for tomorrow morning:**
1. Upload patched ARC build to in-rig board.
2. Validate that changing direction and gear ratio preserves the first three saved values.
3. If needed, continue rig-side power and pin isolation tests.
