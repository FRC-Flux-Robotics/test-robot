# FLUX Robotics Test Robot - Driver Cheat Sheet

**Purpose: Robot Testing & Driver Training** (Not for competition use)

---

## Pre-Run Checklist

### Driver Station Setup
1. Connect to robot WiFi (10.104.13.X network)
2. Open Driver Station and verify:
   - Robot communication (green)
   - Joystick detected (Port 0)
   - Battery voltage > 12V

### Log Level Configuration
In Driver Station > Settings:
| Log Level | When to Use |
|-----------|-------------|
| **Debug** | Troubleshooting issues, tuning PIDs |
| **Info** | Normal training sessions |
| **Warning** | Competition practice (less disk usage) |

### Before Enabling
- [ ] Clear area around robot (3m minimum)
- [ ] Spotter in position
- [ ] Battery fully charged (>12.5V)
- [ ] Emergency stop tested (both bumpers)

---

## Controller: Xbox (Port 0)

### Basic Driving (Field-Centric)
| Control | Action |
|---------|--------|
| **Left Stick Y** | Drive forward/backward |
| **Left Stick X** | Strafe left/right |
| **Right Stick X** | Rotate left/right |

*Note: 10% deadband on all sticks. Sensitivity curves applied for smoother control.*

### Buttons
| Button | Action |
|--------|--------|
| **X** (hold) | Brake - locks wheels in X pattern |
| **B** (hold) | Point wheels - aims all wheels in left stick direction |
| **Right Bumper** (press) | Reset field-centric heading (use when facing away from driver station) |

## Autonomous Mode
- **Action**: Drives backward toward driver station
- **Speed**: 80%
- **Duration**: 2.2 seconds

---

## Specs
- **Max Speed**: ~5 m/s (~11 mph)
- **Max Rotation**: 0.75 rotations/second

---

## Quick Troubleshooting
1. **Robot won't drive**: Check if disabled, verify controller connected
2. **Driving feels weird**: Press Right Bumper to reset heading
3. **Need to stop fast**: Hold X button for brake mode

---

## Post-Run Procedures

### Download Logs
1. Keep robot powered on
2. In Driver Station: **File > Download Logs**
3. Or use SFTP: `sftp lvuser@10.104.13.2:/home/lvuser/logs/`
4. Save to `~/frc-logs/YYYY-MM-DD/` on your laptop

### Log Files to Collect
| File | Contents |
|------|----------|
| `FRC_*.wpilog` | Match/session data (auto-generated) |
| `stdout.log` | Console output, errors |
| `stderr.log` | Error stack traces |

### After Training Session
- [ ] Download logs before powering off
- [ ] Note any issues in session log
- [ ] Charge battery immediately
- [ ] Report hardware issues to mentor

### Quick Log Analysis
```bash
# View recent errors
grep -i "error\|exception" stderr.log

# Check for brownouts
grep -i "brownout" stdout.log
```

---

*Team 10413 FLUX Robotics - 2026 Season - Testing & Training Guide*
