# Emergency Stop Real Robot Validation Checklist

**Task:** S3-ROBOT-1
**Date:** _______________
**Tester:** _______________
**Mentor Present:** _______________

---

## Safety Precautions

Before testing, ensure ALL of the following:

- [ ] Robot is on blocks (wheels off ground) for initial tests
- [ ] Clear 10-foot radius around robot for floor tests
- [ ] Battery fully charged (>12.5V)
- [ ] Safety glasses worn by all present
- [ ] Emergency power disconnect accessible
- [ ] Mentor present and aware of testing

---

## Pre-Test Setup

1. [ ] Connect to robot via Driver Station
2. [ ] Verify robot code deployed successfully
3. [ ] Check SmartDashboard shows `Safety/EmergencyStopActive: false`
4. [ ] Verify controller connected (XboxController)
5. [ ] Confirm all motors at zero output

---

## Test Cases

### Test 1: Emergency Stop Trigger (On Blocks)

**Goal:** Verify both bumpers trigger emergency stop

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 1.1 | Enable robot in teleop mode | Robot enabled, drivetrain responsive | |
| 1.2 | Give small drive input | Wheels spin | |
| 1.3 | Press ONLY left bumper | Nothing happens (robot still active) | |
| 1.4 | Press ONLY right bumper | Nothing happens (robot still active) | |
| 1.5 | Press BOTH bumpers simultaneously | Robot stops immediately | |
| 1.6 | Check SmartDashboard | `Safety/EmergencyStopActive: true` | |
| 1.7 | Check Driver Station | Warning: "EMERGENCY STOP ACTIVATED" | |

**Test 1 Result:** [ ] PASS [ ] FAIL

---

### Test 2: Emergency Stop Blocks All Control

**Goal:** Verify robot stays stopped while emergency stop active

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 2.1 | With emergency stop active, try driving | No motor movement | |
| 2.2 | Move joysticks to max | Still no movement | |
| 2.3 | Wait 5 seconds | Robot remains stopped | |

**Test 2 Result:** [ ] PASS [ ] FAIL

---

### Test 3: Emergency Stop Reset

**Goal:** Verify A button resets when disabled

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 3.1 | With emergency stop active, press A | Nothing (must be disabled first) | |
| 3.2 | Disable robot via Driver Station | Robot disabled | |
| 3.3 | Press A button | Reset acknowledged | |
| 3.4 | Check SmartDashboard | `Safety/EmergencyStopActive: false` | |
| 3.5 | Check Driver Station | Warning: "Emergency stop reset" | |
| 3.6 | Re-enable robot | Robot operational again | |
| 3.7 | Test drive inputs | Robot responds normally | |

**Test 3 Result:** [ ] PASS [ ] FAIL

---

### Test 4: Floor Test (Full Movement)

**Goal:** Verify emergency stop works during actual driving

**CAUTION:** Perform only after Tests 1-3 pass. Ensure clear area.

| Step | Action | Expected Result | Pass/Fail |
|------|--------|-----------------|-----------|
| 4.1 | Remove robot from blocks | Robot on floor | |
| 4.2 | Enable and drive slowly | Robot moves | |
| 4.3 | While moving, press both bumpers | Robot stops immediately | |
| 4.4 | Measure stopping distance | < 6 inches at slow speed | |
| 4.5 | Disable, reset, re-test at 50% speed | Robot stops quickly | |

**Test 4 Result:** [ ] PASS [ ] FAIL

---

## Test Summary

| Test | Description | Result |
|------|-------------|--------|
| Test 1 | Trigger with both bumpers | |
| Test 2 | Control blocked during stop | |
| Test 3 | Reset via A button | |
| Test 4 | Floor test with movement | |

**Overall Result:** [ ] ALL PASS [ ] ISSUES FOUND

---

## Issues Found

| Issue # | Description | Severity | Action Required |
|---------|-------------|----------|-----------------|
| | | | |
| | | | |

---

## Sign-Off

**Tester Signature:** _______________  **Date:** _______________

**Mentor Signature:** _______________  **Date:** _______________

---

## Implementation Reference

- **Trigger method:** `CommandSwerveDrivetrain.triggerEmergencyStop()` (line 419)
- **Reset method:** `CommandSwerveDrivetrain.resetEmergencyStop()` (line 430)
- **Controller binding:** `RobotContainer.java` (lines 135-143)
- **Unit tests:** `SafetyTest.java` (27 tests)
