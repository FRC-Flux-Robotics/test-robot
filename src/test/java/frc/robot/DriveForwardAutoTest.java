package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.autos.DriveForwardAuto;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Unit tests for DriveForwardAuto command.
 * Tests the autonomous command lifecycle and behavior.
 */
@ExtendWith(MockitoExtension.class)
class DriveForwardAutoTest {

    @Mock
    private CommandSwerveDrivetrain mockDrivetrain;

    private DriveForwardAuto command;

    @BeforeAll
    static void initHAL() {
        TestFixtures.initializeHAL();
    }

    @BeforeEach
    void setUp() {
        command = new DriveForwardAuto(mockDrivetrain);
    }

    @Test
    void constructor_requiresDrivetrain() {
        // Verify the command requires the drivetrain subsystem
        assertTrue(command.getRequirements().contains(mockDrivetrain),
            "DriveForwardAuto should require the drivetrain subsystem");
    }

    @Test
    void constructor_hasExactlyOneRequirement() {
        // Command should only require the drivetrain, nothing else
        assertEquals(1, command.getRequirements().size(),
            "DriveForwardAuto should have exactly one requirement");
    }

    @Test
    void isFinished_returnsFalseInitially() {
        // Before initialize is called, timer hasn't started
        // After initialize, timer starts but hasn't reached drive time
        command.initialize();
        assertFalse(command.isFinished(),
            "Command should not be finished immediately after initialize");
    }

    @Test
    void execute_callsSetControlOnDrivetrain() {
        // Initialize first to start the timer
        command.initialize();

        // Execute the command
        command.execute();

        // Verify setControl was called on the drivetrain
        verify(mockDrivetrain, atLeastOnce()).setControl(any(SwerveRequest.class));
    }

    @Test
    void end_callsSetControlToStop() {
        // End the command (simulating interruption or completion)
        command.end(false);

        // Verify setControl was called to stop the robot
        verify(mockDrivetrain).setControl(any(SwerveRequest.class));
    }

    @Test
    void end_whenInterrupted_stillStopsRobot() {
        // End the command as interrupted
        command.end(true);

        // Verify setControl was still called to stop the robot
        verify(mockDrivetrain).setControl(any(SwerveRequest.class));
    }

    @Test
    void initialize_canBeCalledMultipleTimes() {
        // Should not throw when called multiple times (timer.restart handles this)
        assertDoesNotThrow(() -> {
            command.initialize();
            command.initialize();
            command.initialize();
        }, "initialize should be safely callable multiple times");
    }

    @Test
    void commandLifecycle_fullCycle() {
        // Test a complete command lifecycle
        command.initialize();
        assertFalse(command.isFinished(), "Should not be finished after initialize");

        command.execute();
        // Command should still be running (drive time not elapsed)

        command.end(false);
        // Verify drivetrain was controlled during the lifecycle
        verify(mockDrivetrain, atLeast(1)).setControl(any(SwerveRequest.class));
    }
}
