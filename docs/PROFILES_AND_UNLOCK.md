# Ride profiles, safety envelope and advanced-mode unlock

This is the spec for the controller side **and** for the app team.

## 1. Profiles

| Profile | Motor current | Battery current | Torque ramp | Speed limit | Throttle curve | Field weakening | Editable |
|---|---|---|---|---|---|---|---|
| **ECO** | 50 % of factory | 50 % | 0.5× | 80 % of factory limit | soft (expo 0.5) | off | no |
| **SPORT** | 80 % | 80 % | 1× | factory limit | medium (0.3) | off | no |
| **RACE** | 100 % | 100 % | 1.5× | factory limit | sharp (0.15) | factory setting | no |
| **CUSTOM** ("VESC mode") | user | user | user | user (0 = off) | user | user | only after unlock |

- "Factory" means `g_config.limits` and `vehicle.speed_limit_kmh`. These are set per scooter model in
  `config_load_defaults()` (src/config/config.c) and **cannot be changed over UART/CAN at all**, locked or
  unlocked. Presets never raise the factory speed limit.
- Switching between ECO/SPORT/RACE is allowed while riding. A higher limit is reached through the normal
  torque ramp, and a lower limit applies immediately.
- CUSTOM can only be **entered at standstill**, and it is never the boot profile. After every power-up the
  scooter starts in the boot profile (default SPORT, settable to ECO/SPORT/RACE).
- Regen braking is 100 % in every preset: braking power is never reduced by picking ECO.
- Motor detection always uses the factory limits, whatever profile is active.

## 2. Safety envelope (hard ceilings)

Defined per board in `src/config/board_config.h`. Only a firmware rebuild can change them.

| Setting | Ceiling |
|---|---|
| Motor (phase) current, drive and regen | **150 A** |
| Overcurrent trip | **160 A** (must stay ≥ 1.05 × operating current) |
| Battery current | 100 A (and ≤ motor current) |
| Battery regen current | 40 A |
| Field weakening | 50 A (and ≤ motor current) |
| eRPM | 60 000 |
| Torque ramp | 1000 A/s |
| Speed limit | 100 km/h |
| Current-loop bandwidth | 2500 Hz |
| Modulation | 0.95 |

What enforces it:
- **Refuse, don't clamp.** `COMMIT_CONFIG` with any value outside the envelope is rejected with
  `NACK_ENVELOPE` plus the rule that was broken (`env_violation_t`), so the app can tell the user exactly
  which setting is too high. NaN, Inf and negative values are rejected too.
- **Compile-time check.** The build fails if the trip ceiling is above what the current sensors can measure
  (`BOARD_CURRENT_SENSE_RANGE_A`, 165 A on this board) or above `BOARD_MAX_PHASE_CURRENT`.
- **Boot and runtime check.** Values loaded from flash, and the active profile, are clamped to the envelope
  again. A tampered flash image cannot exceed it.
- **Protected fields.** Even when unlocked, a commit that changes factory limits, factory speed limit,
  wheel diameter, gear ratio, pole pairs, vehicle type or cell count is refused with `NACK_PROTECTED_FIELD`.
  Wheel diameter and pole pairs are protected because changing them is the usual way to fool the
  speedometer and the speed limit.

> The envelope protects the controller only if its numbers match the hardware. 150 A is safe only if your
> FETs, copper, connectors and cooling are rated for 150 A with margin. Validate with a sustained-load
> thermal test (docs/BRINGUP.md step 8) before trusting it.

## 3. Unlock protocol (for the app)

The unlock is a challenge-response signed with a secret key, so only your app can unlock it. It is bound to
one controller and one version of your terms.

```
App                                         Controller
 │  UNLOCK_CHALLENGE (0x12)  ─────────────▶  │  (motor must be stopped)
 │  ◀──────────── nonce[16] uid[12] terms_required:u16le
 │
 │  show terms v{terms_required} → rider taps "I agree"
 │
 │  mac = HMAC-SHA256(key, "SK2V-UNLOCK-v1" ‖ nonce ‖ uid ‖ terms_version:u16le)
 │  UNLOCK_RESPONSE (0x13): terms_version:u16le ‖ mac[32]  ─▶ │
 │  ◀──────────── ACK (0x93)  or  NACK 0x7F [0x13, 11, unlock_result_t]
```

Rules enforced by the controller:
- The nonce comes from the hardware RNG, can be used **once**, and expires after **30 s**.
- **5 wrong signatures lead to a 60 s lockout.**
- `terms_version` must be ≥ `UNLOCK_TERMS_VERSION_REQUIRED` (src/comms/unlock.h). When you change your
  terms, bump that constant in a firmware update. Existing unlocks are then revoked at boot and riders
  must agree again.
- The unlock and the accepted terms version are stored in flash and survive power cycles.
  `LOCK_ADVANCED (0x14)` revokes it (only at standstill if CUSTOM is active). `LOAD_DEFAULTS` also re-locks.

Reference implementation (Python, the same code as `tools/sk2v_cli.py unlock`):
```python
import hmac, hashlib, struct
def unlock_mac(key: bytes, nonce: bytes, uid: bytes, terms_version: int) -> bytes:
    msg = b"SK2V-UNLOCK-v1" + nonce + uid + struct.pack("<H", terms_version)
    return hmac.new(key, msg, hashlib.sha256).digest()
```
Kotlin: `Mac.getInstance("HmacSHA256")`. Swift: `CryptoKit.HMAC<SHA256>`. Dart: `package:crypto` `Hmac(sha256, key)`.

Recommended app records (these are for your legal trail; the controller can't enforce them): user account,
controller UID, terms version, timestamp, app version. Store them server-side when the user agrees.

### Other commands the app needs

| Cmd | Payload → reply |
|---|---|
| `PROFILE_INFO 0x10` | → `u8 active, u8 boot, u8 unlocked, u8 dev_key, u16 terms_accepted, u16 terms_required, f32 envelope[9], f32 active_params[10]` (84 B). Use `envelope[]` as the slider maximums in the UI |
| `SET_PROFILE 0x11` | `u8 profile` (0 eco, 1 sport, 2 race, 3 custom) |
| `SET_BOOT_PROFILE 0x15` | `u8 profile` (0–2) |
| `GET_CONFIG / SET_CONFIG / COMMIT_CONFIG` | edit the CUSTOM values in `app_config_t.ride.custom` (and tuning in `foc`). Start with `GET_CONFIG` at offset 0, which syncs the staging buffer, and recompute the CRC32 before committing |
| Telemetry | `profile` byte and `flags` bit 4 = unlocked |
| CAN `0x300+n` | byte 2 = requested profile (0xFF = no change), for handlebar displays |

## 4. Key management

1. `./tools/gen_unlock_key.py` creates `secrets/unlock_key.h` (compiled into the firmware) and
   `secrets/unlock_key.hex` (for the app). Both are git-ignored, so back them up somewhere safe.
2. Without that file the build uses a **public development key** and prints a compiler warning.
   `PING`/`PROFILE_INFO` report `dev_key = 1`, and the CLI prints a warning. Never ship that build.

**Be honest with yourselves about the threat model:**
- A key embedded in a mobile app can be extracted by someone who decompiles it. Better: keep the key on
  **your server**. The app forwards `nonce + uid`, and the server checks the user's login plus recorded
  agreement, then returns the MAC. The firmware doesn't change for this.
- Anyone with an SWD programmer can reflash the whole controller. For production, set STM32 **readout
  protection (RDP level 1)** so the firmware and key can't be read out.
- The unlock only controls **permission**. **Physical safety** comes from the envelope, which applies even
  to a fully unlocked controller.
