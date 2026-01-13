# FLUX Robotics Test Robot - Driver Cheat Sheet

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

*Team 10413 FLUX Robotics - 2026 Season*
