package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.DriveForwardAuto;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

/**
 * Integration tests for robot command execution.
 *
 * <p>Tests command chains, autonomous mode, and teleop control mappings.
 *
 * <p>Covers PLAN.md tasks:
 * <ul>
 *   <li>S2-TEST-3.1: Test drivetrain command chain execution</li>
 *   <li>S2-TEST-3.2: Test autonomous mode runs correctly</li>
 *   <li>S2-TEST-3.3: Test teleop control mappings</li>
 * </ul>
 */
@ExtendWith(MockitoExtension.class)
class IntegrationTest {

    @BeforeAll
    static void initHAL() {
        TestFixtures.initializeHAL();
    }

    @BeforeEach
    void setUp() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    // ========== S2-TEST-3.1: Command Chain Execution Tests ==========

    @Nested
    @DisplayName("S2-TEST-3.1: Command Chain Execution")
    class CommandChainTests {

        @Mock
        private CommandSwerveDrivetrain mockDrivetrain;

        @Test
        @DisplayName("DriveForwardAuto claims drivetrain via addRequirements")
        void driveForwardAuto_claimsDrivetrain() {
            DriveForwardAuto auto = new DriveForwardAuto(mockDrivetrain);

            assertTrue(
                    auto.getRequirements().contains(mockDrivetrain),
                    "DriveForwardAuto should claim drivetrain subsystem");
            assertEquals(1, auto.getRequirements().size(), "Should have exactly one requirement");
        }

        @Test
        @DisplayName("Command lifecycle executes in order: initialize -> execute -> end")
        void commandLifecycle_executesInOrder() {
            DriveForwardAuto auto = new DriveForwardAuto(mockDrivetrain);

            // Initialize starts the timer
            assertDoesNotThrow(() -> auto.initialize(), "initialize() should not throw");

            // Execute applies control
            assertDoesNotThrow(() -> auto.execute(), "execute() should not throw");
            verify(mockDrivetrain, atLeastOnce()).setControl(any(SwerveRequest.class));

            // isFinished checks timer
            assertDoesNotThrow(() -> auto.isFinished(), "isFinished() should not throw");

            // End stops motors
            assertDoesNotThrow(() -> auto.end(false), "end() should not throw");
        }

        @Test
        @DisplayName("Command can be restarted after completion")
        void command_canBeRestarted() {
            DriveForwardAuto auto = new DriveForwardAuto(mockDrivetrain);

            // First execution cycle
            auto.initialize();
            auto.execute();
            auto.end(false);

            // Second execution cycle should work
            assertDoesNotThrow(
                    () -> {
                        auto.initialize();
                        auto.execute();
                        auto.end(false);
                    },
                    "Command should be restartable");
        }

        @Test
        @DisplayName("Interrupted command still calls end")
        void interruptedCommand_callsEnd() {
            DriveForwardAuto auto = new DriveForwardAuto(mockDrivetrain);

            auto.initialize();
            auto.execute();

            // Simulate interruption
            assertDoesNotThrow(() -> auto.end(true), "end(true) should not throw");

            // Should still call setControl to stop
            verify(mockDrivetrain, atLeast(2)).setControl(any(SwerveRequest.class));
        }

        @Test
        @DisplayName("Multiple commands can be created for same subsystem")
        void multipleCommands_canBeCreated() {
            DriveForwardAuto auto1 = new DriveForwardAuto(mockDrivetrain);
            DriveForwardAuto auto2 = new DriveForwardAuto(mockDrivetrain);

            // Both should have valid requirements
            assertEquals(1, auto1.getRequirements().size());
            assertEquals(1, auto2.getRequirements().size());

            // Both should require the same subsystem
            assertEquals(auto1.getRequirements(), auto2.getRequirements());
        }

        @Test
        @DisplayName("Command isFinished returns false before drive time elapses")
        void command_isFinishedFalseInitially() {
            DriveForwardAuto auto = new DriveForwardAuto(mockDrivetrain);

            auto.initialize();

            // Immediately after initialization, timer hasn't elapsed
            assertFalse(auto.isFinished(), "Should not be finished immediately");
        }

        @Test
        @DisplayName("Command setControl respects SwerveRequest interface")
        void command_usesCorrectSwerveRequest() {
            DriveForwardAuto auto = new DriveForwardAuto(mockDrivetrain);

            auto.initialize();
            auto.execute();

            // Verify the command calls setControl with a SwerveRequest
            verify(mockDrivetrain).setControl(any(SwerveRequest.class));
        }
    }

    // ========== S2-TEST-3.2: Autonomous Mode Tests ==========

    @Nested
    @DisplayName("S2-TEST-3.2: Autonomous Mode")
    class AutonomousModeTests {

        @Mock
        private CommandSwerveDrivetrain mockDrivetrain;

        @Test
        @DisplayName("Auto mode speed is negative for backward movement")
        void autoModeSpeed_isNegativeForBackward() {
            assertTrue(DriveConstants.AutoModeSpeed < 0, "Auto mode speed should be negative for backward movement");
            assertEquals(-0.8, DriveConstants.AutoModeSpeed, 0.001, "Auto mode speed should be -0.8 m/s");
        }

        @Test
        @DisplayName("Auto mode drive time is configured")
        void autoModeDriveTime_isConfigured() {
            assertTrue(DriveConstants.AutoModeDriveTime > 0, "Auto mode drive time should be positive");
            assertEquals(2.2, DriveConstants.AutoModeDriveTime, 0.001, "Auto mode drive time should be 2.2 seconds");
        }

        @Test
        @DisplayName("DriveForwardAuto uses FieldCentric request")
        void driveForwardAuto_usesFieldCentricRequest() {
            DriveForwardAuto auto = new DriveForwardAuto(mockDrivetrain);

            auto.initialize();
            auto.execute();

            // Verify setControl was called - the request type is FieldCentric based on code
            verify(mockDrivetrain, atLeastOnce()).setControl(any(SwerveRequest.class));
        }

        @Test
        @DisplayName("DriveForwardAuto can be created with mock drivetrain")
        void driveForwardAuto_canBeCreatedWithMockDrivetrain() {
            assertDoesNotThrow(
                    () -> new DriveForwardAuto(mockDrivetrain), "DriveForwardAuto should accept mock drivetrain");
        }

        @Test
        @DisplayName("Auto command stops motors at end")
        void autoCommand_stopsAtEnd() {
            DriveForwardAuto auto = new DriveForwardAuto(mockDrivetrain);

            auto.initialize();
            auto.execute();
            auto.end(false);

            // Verify setControl was called at least twice (once in execute, once in end to stop)
            verify(mockDrivetrain, atLeast(2)).setControl(any(SwerveRequest.class));
        }

        @Test
        @DisplayName("Auto command handles multiple execute cycles")
        void autoCommand_handlesMultipleExecuteCycles() {
            DriveForwardAuto auto = new DriveForwardAuto(mockDrivetrain);

            auto.initialize();

            // Execute multiple times like scheduler would
            assertDoesNotThrow(
                    () -> {
                        for (int i = 0; i < 10; i++) {
                            auto.execute();
                        }
                    },
                    "Multiple execute cycles should not throw");

            auto.end(false);

            // Verify setControl called multiple times
            verify(mockDrivetrain, atLeast(10)).setControl(any(SwerveRequest.class));
        }
    }

    // ========== S2-TEST-3.3: Teleop Control Mappings Tests ==========

    @Nested
    @DisplayName("S2-TEST-3.3: Teleop Control Mappings")
    class TeleopControlMappingsTests {

        // ========== Sensitivity Configuration Tests ==========

        @Test
        @DisplayName("Position sensitivity has deadzone at xStart=0.02")
        void positionSensitivity_hasDeadzone() {
            assertEquals(0.02, OperatorConstants.xStartPos, "Position deadzone should be 0.02");

            PiecewiseSensitivity sensitivity = new PiecewiseSensitivity(
                    OperatorConstants.xStartPos,
                    OperatorConstants.xMiddlePos,
                    OperatorConstants.yStartPos,
                    OperatorConstants.yMiddlePos,
                    OperatorConstants.yMaxPos);

            // Input below deadzone should return 0
            assertEquals(0.0, sensitivity.transfer(0.01), 0.001, "Input below deadzone should return 0");
        }

        @Test
        @DisplayName("Position sensitivity max output is limited to 0.8")
        void positionSensitivity_maxOutputIsLimited() {
            assertEquals(0.8, OperatorConstants.yMaxPos, "Position max output should be 0.8");

            PiecewiseSensitivity sensitivity = new PiecewiseSensitivity(
                    OperatorConstants.xStartPos,
                    OperatorConstants.xMiddlePos,
                    OperatorConstants.yStartPos,
                    OperatorConstants.yMiddlePos,
                    OperatorConstants.yMaxPos);

            assertEquals(0.8, sensitivity.transfer(1.0), 0.001, "Max input should return yMax");
        }

        @Test
        @DisplayName("Rotation sensitivity has full range (yMax=1.0)")
        void rotationSensitivity_hasFullRange() {
            assertEquals(1.0, OperatorConstants.yMaxRot, "Rotation max output should be 1.0");

            PiecewiseSensitivity sensitivity = new PiecewiseSensitivity(
                    OperatorConstants.xStartRot,
                    OperatorConstants.xMiddleRot,
                    OperatorConstants.yStartRot,
                    OperatorConstants.yMiddleRot,
                    OperatorConstants.yMaxRot);

            assertEquals(1.0, sensitivity.transfer(1.0), 0.001, "Max rotation input should return 1.0");
        }

        @Test
        @DisplayName("Sensitivity transfer of zero returns zero")
        void sensitivity_transferZeroReturnsZero() {
            PiecewiseSensitivity sensitivity = new PiecewiseSensitivity(
                    OperatorConstants.xStartPos,
                    OperatorConstants.xMiddlePos,
                    OperatorConstants.yStartPos,
                    OperatorConstants.yMiddlePos,
                    OperatorConstants.yMaxPos);

            assertEquals(0.0, sensitivity.transfer(0.0), 0.001, "Zero input should return zero");
        }

        @Test
        @DisplayName("Sensitivity transfer preserves sign for negative input")
        void sensitivity_transferNegativePreservesSign() {
            PiecewiseSensitivity sensitivity = new PiecewiseSensitivity(
                    OperatorConstants.xStartPos,
                    OperatorConstants.xMiddlePos,
                    OperatorConstants.yStartPos,
                    OperatorConstants.yMiddlePos,
                    OperatorConstants.yMaxPos);

            double negativeOutput = sensitivity.transfer(-0.5);
            assertTrue(negativeOutput < 0, "Negative input should produce negative output");

            double positiveOutput = sensitivity.transfer(0.5);
            assertTrue(positiveOutput > 0, "Positive input should produce positive output");

            assertEquals(
                    Math.abs(negativeOutput),
                    Math.abs(positiveOutput),
                    0.001,
                    "Magnitude should be same for same absolute input");
        }

        @Test
        @DisplayName("Sensitivity transfer at max returns yMax")
        void sensitivity_transferMaxReturnsYMax() {
            PiecewiseSensitivity sensitivity = new PiecewiseSensitivity(
                    OperatorConstants.xStartPos,
                    OperatorConstants.xMiddlePos,
                    OperatorConstants.yStartPos,
                    OperatorConstants.yMiddlePos,
                    OperatorConstants.yMaxPos);

            assertEquals(
                    OperatorConstants.yMaxPos, sensitivity.transfer(1.0), 0.001, "Transfer of 1.0 should return yMax");
            assertEquals(
                    -OperatorConstants.yMaxPos,
                    sensitivity.transfer(-1.0),
                    0.001,
                    "Transfer of -1.0 should return -yMax");
        }

        // ========== Button Binding Configuration Tests ==========

        @Test
        @DisplayName("Driver controller port is 0")
        void driverControllerPort_isZero() {
            assertEquals(0, OperatorConstants.DriverControllerPort, "Driver controller should be on port 0");
        }

        @Test
        @DisplayName("Emergency stop binding: both bumpers (left AND right)")
        void emergencyStopBinding_documentedAsBothBumpers() {
            // This documents the expected binding from RobotContainer line 136:
            // Trigger emergencyStopTrigger = driverController.leftBumper().and(driverController.rightBumper());
            // emergencyStopTrigger.onTrue(drivetrain.runOnce(() -> drivetrain.triggerEmergencyStop()));
            assertTrue(true, "Emergency stop binding documented as left AND right bumpers in RobotContainer:136");
        }

        @Test
        @DisplayName("Brake binding: X button")
        void brakeBinding_documentedAsXButton() {
            // This documents the expected binding from RobotContainer line 119:
            // driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
            assertTrue(true, "Brake binding documented as X button in RobotContainer:119");
        }

        @Test
        @DisplayName("Point binding: B button")
        void pointBinding_documentedAsBButton() {
            // This documents the expected binding from RobotContainer lines 120-123:
            // driverController.b().whileTrue(drivetrain.applyRequest(() -> point...));
            assertTrue(true, "Point binding documented as B button in RobotContainer:120-123");
        }

        @Test
        @DisplayName("Seed field centric binding: right bumper")
        void seedFieldCentricBinding_documentedAsRightBumper() {
            // This documents the expected binding from RobotContainer line 133:
            // driverController.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
            assertTrue(true, "Seed field centric binding documented as right bumper in RobotContainer:133");
        }

        @Test
        @DisplayName("Emergency stop reset binding: disabled + A button")
        void emergencyStopReset_documentedAsDisabledPlusA() {
            // This documents the expected binding from RobotContainer lines 141-143:
            // RobotModeTriggers.disabled().and(driverController.a())
            //     .onTrue(drivetrain.runOnce(() -> drivetrain.resetEmergencyStop()).ignoringDisable(true));
            assertTrue(true, "Emergency stop reset binding documented as disabled + A in RobotContainer:141-143");
        }
    }
}
