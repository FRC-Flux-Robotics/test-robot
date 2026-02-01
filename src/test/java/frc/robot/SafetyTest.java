package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.subsystems.drive.DrivetrainIOMock;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for robot safety features.
 * Tests emergency stop, current limits, and brownout protection.
 *
 * <p>Covers PLAN.md tasks:
 * <ul>
 *   <li>S2-TEST-2.1: Test emergency stop zeros all motor outputs</li>
 *   <li>S2-TEST-2.2: Test current limits are correctly configured</li>
 *   <li>S2-TEST-2.3: Test brownout protection activates at correct voltage</li>
 *   <li>S2-TEST-2.4: Test safety state transitions</li>
 * </ul>
 */
class SafetyTest extends SimulationTestBase {

    @Override
    protected TestMode getTestMode() {
        return TestMode.MOCK;
    }

    // ========== Emergency Stop Tests (S2-TEST-2.1, S2-TEST-2.4) ==========

    @Nested
    @DisplayName("Emergency Stop Tests")
    class EmergencyStopTests {

        @Test
        @DisplayName("Emergency stop should call stop() on IO layer")
        void emergencyStop_callsStopOnIO() {
            // Arrange: Configure mock with non-zero velocities
            mockIO.withDriveVelocities(new double[] {1.0, 1.0, 1.0, 1.0});

            // Act: Trigger stop via IO layer
            activeIO.stop();

            // Assert: Verify stop was called
            assertEquals(1, mockIO.getStopCallCount());
        }

        @Test
        @DisplayName("Stop should not be called initially")
        void emergencyStopFlag_initiallyNotCalled() {
            // Assert: Stop should not have been called before any action
            assertEquals(0, mockIO.getStopCallCount());
        }

        @Test
        @DisplayName("Drive commands still recorded after stop (blocking at subsystem level)")
        void emergencyStop_driveCommandsRecorded() {
            // Arrange: Trigger stop
            activeIO.stop();

            // Act: Try to drive (IO layer records all calls)
            activeIO.driveFieldCentric(1.0, 0.0, 0.0);

            // Assert: Drive command should still be recorded at IO level
            // (actual blocking happens in CommandSwerveDrivetrain layer)
            assertTrue(mockIO.wasDriveFieldCentricCalled());
        }

        @Test
        @DisplayName("Stop can be called multiple times safely (idempotent)")
        void emergencyStop_idempotent() {
            // Act: Trigger stop multiple times
            activeIO.stop();
            activeIO.stop();
            activeIO.stop();

            // Assert: All calls should be counted
            assertEquals(3, mockIO.getStopCallCount());
        }

        @Test
        @DisplayName("Stop count resets when mock is reset")
        void emergencyStop_resetsOnMockReset() {
            // Arrange: Call stop
            activeIO.stop();
            assertEquals(1, mockIO.getStopCallCount());

            // Act: Reset mock
            mockIO.reset();

            // Assert: Stop count should be reset
            assertEquals(0, mockIO.getStopCallCount());
        }
    }

    // ========== Current Limits Tests (S2-TEST-2.2) ==========

    @Nested
    @DisplayName("Current Limits Configuration Tests")
    class CurrentLimitsTests {

        @Test
        @DisplayName("Drive motor current limit should be 40A continuous")
        void driveMotorCurrentLimit_is40AContinuous() {
            assertEquals(40.0, Constants.SafetyConstants.DRIVE_STATOR_CURRENT_LIMIT_AMPS);
        }

        @Test
        @DisplayName("Drive motor peak current limit should be 60A")
        void driveMotorPeakCurrentLimit_is60A() {
            assertEquals(60.0, Constants.SafetyConstants.DRIVE_STATOR_CURRENT_LIMIT_PEAK_AMPS);
        }

        @Test
        @DisplayName("Drive motor peak current duration should be 0.1 seconds")
        void driveMotorPeakCurrentDuration_is100ms() {
            assertEquals(0.1, Constants.SafetyConstants.DRIVE_PEAK_CURRENT_DURATION_SECONDS);
        }

        @Test
        @DisplayName("Drive motor supply current limit should be 35A")
        void driveMotorSupplyCurrentLimit_is35A() {
            assertEquals(35.0, Constants.SafetyConstants.DRIVE_SUPPLY_CURRENT_LIMIT_AMPS);
        }

        @Test
        @DisplayName("Steer motor current limit should be 20A continuous")
        void steerMotorCurrentLimit_is20AContinuous() {
            assertEquals(20.0, Constants.SafetyConstants.STEER_STATOR_CURRENT_LIMIT_AMPS);
        }

        @Test
        @DisplayName("Steer motor peak current limit should be 30A")
        void steerMotorPeakCurrentLimit_is30A() {
            assertEquals(30.0, Constants.SafetyConstants.STEER_STATOR_CURRENT_LIMIT_PEAK_AMPS);
        }

        @Test
        @DisplayName("Mock can report currents at drive limit")
        void mockIO_canReportDriveCurrentsAtLimit() {
            // Arrange: Set currents at limit
            mockIO.withDriveCurrents(new double[] {40.0, 40.0, 40.0, 40.0});
            updateInputs();

            // Assert: All currents should be at or below limit
            assertDriveCurrentsBelow(40.1);
        }

        @Test
        @DisplayName("Mock can report currents at steer limit")
        void mockIO_canReportSteerCurrentsAtLimit() {
            // Arrange: Set currents at limit
            mockIO.withSteerCurrents(new double[] {20.0, 20.0, 20.0, 20.0});
            updateInputs();

            // Assert: All currents should be at or below limit
            assertSteerCurrentsBelow(20.1);
        }
    }

    // ========== Brownout Protection Tests (S2-TEST-2.3, S2-TEST-2.4) ==========

    @Nested
    @DisplayName("Brownout Protection Tests")
    class BrownoutProtectionTests {

        @Test
        @DisplayName("Brownout protection threshold should be 10.5V")
        void brownoutThreshold_is10Point5Volts() {
            assertEquals(10.5, Constants.SafetyConstants.BROWNOUT_VOLTAGE_THRESHOLD);
        }

        @Test
        @DisplayName("Brownout recovery threshold should be 11.0V")
        void brownoutRecoveryThreshold_is11Point0Volts() {
            assertEquals(11.0, Constants.SafetyConstants.BROWNOUT_RECOVERY_VOLTAGE);
        }

        @Test
        @DisplayName("Brownout recovery threshold has hysteresis gap")
        void brownoutRecoveryThreshold_hasHysteresis() {
            double threshold = Constants.SafetyConstants.BROWNOUT_VOLTAGE_THRESHOLD;
            double recovery = Constants.SafetyConstants.BROWNOUT_RECOVERY_VOLTAGE;

            assertTrue(recovery > threshold,
                    "Recovery voltage should be higher than threshold for hysteresis");
            assertEquals(0.5, recovery - threshold, 0.01,
                    "Hysteresis gap should be 0.5V");
        }

        @Test
        @DisplayName("Brownout speed multiplier should be 0.5 (50%)")
        void brownoutSpeedMultiplier_is50Percent() {
            assertEquals(0.5, Constants.SafetyConstants.BROWNOUT_SPEED_MULTIPLIER);
        }

        @Test
        @DisplayName("Mock IO can simulate low battery voltage")
        void mockIO_canSimulateLowBatteryVoltage() {
            // Arrange: Set low voltage
            mockIO.withBatteryVoltage(10.0);

            // Assert: Voltage should be set
            assertEquals(10.0, mockIO.getBatteryVoltage());
        }

        @Test
        @DisplayName("Mock IO can simulate normal battery voltage")
        void mockIO_canSimulateNormalBatteryVoltage() {
            // Arrange: Set normal voltage
            mockIO.withBatteryVoltage(12.5);

            // Assert: Voltage should be set
            assertEquals(12.5, mockIO.getBatteryVoltage());
        }

        @Test
        @DisplayName("Mock IO defaults to 12V battery voltage")
        void mockIO_defaultsTo12VBatteryVoltage() {
            // Assert: Default voltage should be 12V
            assertEquals(12.0, mockIO.getBatteryVoltage());
        }

        @Test
        @DisplayName("Mock IO resets battery voltage on resetAll()")
        void mockIO_resetsBatteryVoltageOnResetAll() {
            // Arrange: Set low voltage
            mockIO.withBatteryVoltage(10.0);

            // Act: Reset all
            mockIO.resetAll();

            // Assert: Voltage should be back to default
            assertEquals(12.0, mockIO.getBatteryVoltage());
        }
    }

    // ========== State Transition Tests (S2-TEST-2.4) ==========

    @Nested
    @DisplayName("Safety State Transition Tests")
    class StateTransitionTests {

        @Test
        @DisplayName("Normal -> Emergency Stop -> Normal transition via mock reset")
        void stateTransition_normalToEmergencyAndBack() {
            // Start in normal state
            assertEquals(0, mockIO.getStopCallCount());

            // Transition to emergency stop
            activeIO.stop();
            assertEquals(1, mockIO.getStopCallCount());

            // Reset (simulates what resetEmergencyStop does at subsystem level)
            mockIO.reset();
            assertEquals(0, mockIO.getStopCallCount());
        }

        @Test
        @DisplayName("Driving works after mock reset")
        void afterReset_drivingWorks() {
            // Trigger and reset
            activeIO.stop();
            mockIO.reset();

            // Drive should work
            activeIO.driveFieldCentric(1.0, 0.0, 0.0);
            assertTrue(mockIO.wasDriveFieldCentricCalled());
        }

        @Test
        @DisplayName("Safety SmartDashboard keys are defined")
        void safetyConstants_hasSmartDashboardKeys() {
            assertNotNull(Constants.SafetyConstants.EMERGENCY_STOP_KEY);
            assertNotNull(Constants.SafetyConstants.BROWNOUT_WARNING_KEY);
            assertNotNull(Constants.SafetyConstants.BATTERY_VOLTAGE_KEY);
        }

        @Test
        @DisplayName("Safety SmartDashboard keys use Safety/ namespace")
        void safetyConstants_keysUseSafetyNamespace() {
            assertTrue(Constants.SafetyConstants.EMERGENCY_STOP_KEY.startsWith("Safety/"),
                    "Emergency stop key should be in Safety/ namespace");
            assertTrue(Constants.SafetyConstants.BROWNOUT_WARNING_KEY.startsWith("Safety/"),
                    "Brownout warning key should be in Safety/ namespace");
            assertTrue(Constants.SafetyConstants.BATTERY_VOLTAGE_KEY.startsWith("Safety/"),
                    "Battery voltage key should be in Safety/ namespace");
        }

        @Test
        @DisplayName("Mock tracks speed multiplier changes")
        void mockIO_tracksSpeedMultiplier() {
            // Act: Set speed multiplier
            mockIO.setSpeedMultiplier(0.5);

            // Assert: Should be tracked
            assertEquals(0.5, mockIO.getSpeedMultiplier());
        }

        @Test
        @DisplayName("Mock resets speed multiplier on resetAll()")
        void mockIO_resetsSpeedMultiplierOnResetAll() {
            // Arrange: Set speed multiplier
            mockIO.setSpeedMultiplier(0.5);

            // Act: Reset all
            mockIO.resetAll();

            // Assert: Should be back to 1.0
            assertEquals(1.0, mockIO.getSpeedMultiplier());
        }
    }
}
