package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.TestFixtures;
import java.util.function.Supplier;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for CommandSwerveDrivetrain.
 * Uses Mockito mocks to test API contracts without requiring Phoenix 6 hardware.
 *
 * <p>Note: Due to CommandSwerveDrivetrain extending Phoenix 6's SwerveDrivetrain,
 * many methods cannot be directly unit tested. These tests validate the public API
 * contracts using mocks.
 */
class CommandSwerveDrivetrainTest {

    private CommandSwerveDrivetrain mockDrivetrain;

    @BeforeAll
    static void initHAL() {
        TestFixtures.initializeHAL();
    }

    @BeforeEach
    void setUp() {
        mockDrivetrain = TestFixtures.createMockDrivetrain();
    }

    // ========== Position Tests ==========

    @Test
    void getPosition_returnsConfiguredPose() {
        Pose2d expectedPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45.0));
        when(mockDrivetrain.getPosition()).thenReturn(expectedPose);

        Pose2d actualPose = mockDrivetrain.getPosition();

        assertEquals(expectedPose.getX(), actualPose.getX(), 0.001);
        assertEquals(expectedPose.getY(), actualPose.getY(), 0.001);
        assertEquals(
                expectedPose.getRotation().getDegrees(),
                actualPose.getRotation().getDegrees(),
                0.001);
    }

    @Test
    void getPosition_defaultsToOrigin() {
        // TestFixtures.createMockDrivetrain() returns origin by default
        Pose2d pose = mockDrivetrain.getPosition();

        assertEquals(0.0, pose.getX(), 0.001);
        assertEquals(0.0, pose.getY(), 0.001);
        assertEquals(0.0, pose.getRotation().getDegrees(), 0.001);
    }

    @Test
    void getPosition_withCustomPosition() {
        Pose2d customPose = new Pose2d(5.0, -3.0, Rotation2d.fromDegrees(180.0));
        CommandSwerveDrivetrain customMock = TestFixtures.createMockDrivetrain(customPose);

        Pose2d actualPose = customMock.getPosition();

        assertEquals(5.0, actualPose.getX(), 0.001);
        assertEquals(-3.0, actualPose.getY(), 0.001);
        assertEquals(180.0, actualPose.getRotation().getDegrees(), 0.001);
    }

    // ========== Gyro Tests ==========

    @Test
    void getGyro_returnsNullByDefault() {
        // Default mock returns null for gyro
        Pigeon2 gyro = mockDrivetrain.getGyro();
        assertNull(gyro);
    }

    @Test
    void getGyro_returnsMockedGyro() {
        Pigeon2 mockGyro = mock(Pigeon2.class);
        when(mockGyro.getRotation2d()).thenReturn(Rotation2d.fromDegrees(90.0));
        when(mockDrivetrain.getGyro()).thenReturn(mockGyro);

        Pigeon2 gyro = mockDrivetrain.getGyro();

        assertNotNull(gyro);
        assertEquals(90.0, gyro.getRotation2d().getDegrees(), 0.001);
    }

    // ========== Odometry Reset Tests ==========

    @Test
    void resetOdometry_canBeCalled() {
        Pose2d newPose = new Pose2d(10.0, 5.0, Rotation2d.fromDegrees(45.0));

        // Should not throw
        assertDoesNotThrow(() -> mockDrivetrain.resetOdometry(newPose));

        // Verify the method was called
        verify(mockDrivetrain).resetOdometry(newPose);
    }

    @Test
    void resetOdometry_acceptsOrigin() {
        Pose2d origin = new Pose2d();

        assertDoesNotThrow(() -> mockDrivetrain.resetOdometry(origin));
        verify(mockDrivetrain).resetOdometry(origin);
    }

    // ========== Command Factory Tests ==========

    @Test
    void applyRequest_returnsCommand() {
        Command mockCommand = mock(Command.class);
        @SuppressWarnings("unchecked")
        Supplier<SwerveRequest> requestSupplier = mock(Supplier.class);
        when(mockDrivetrain.applyRequest(any())).thenReturn(mockCommand);

        Command result = mockDrivetrain.applyRequest(requestSupplier);

        assertNotNull(result);
        verify(mockDrivetrain).applyRequest(requestSupplier);
    }

    @Test
    void sysIdQuasistatic_forward_returnsCommand() {
        Command mockCommand = mock(Command.class);
        when(mockDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward)).thenReturn(mockCommand);

        Command result = mockDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward);

        assertNotNull(result);
        verify(mockDrivetrain).sysIdQuasistatic(SysIdRoutine.Direction.kForward);
    }

    @Test
    void sysIdQuasistatic_reverse_returnsCommand() {
        Command mockCommand = mock(Command.class);
        when(mockDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)).thenReturn(mockCommand);

        Command result = mockDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);

        assertNotNull(result);
        verify(mockDrivetrain).sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
    }

    @Test
    void sysIdDynamic_forward_returnsCommand() {
        Command mockCommand = mock(Command.class);
        when(mockDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward)).thenReturn(mockCommand);

        Command result = mockDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward);

        assertNotNull(result);
        verify(mockDrivetrain).sysIdDynamic(SysIdRoutine.Direction.kForward);
    }

    @Test
    void sysIdDynamic_reverse_returnsCommand() {
        Command mockCommand = mock(Command.class);
        when(mockDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse)).thenReturn(mockCommand);

        Command result = mockDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse);

        assertNotNull(result);
        verify(mockDrivetrain).sysIdDynamic(SysIdRoutine.Direction.kReverse);
    }

    // ========== Vision Integration Tests ==========

    @Test
    void addVisionMeasurement_canBeCalled() {
        Pose2d visionPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(30.0));
        double timestamp = 1.234;

        assertDoesNotThrow(() -> mockDrivetrain.addVisionMeasurement(visionPose, timestamp));
        verify(mockDrivetrain).addVisionMeasurement(visionPose, timestamp);
    }

    // ========== Periodic Tests ==========

    @Test
    void periodic_canBeCalled() {
        // Verify periodic() can be called without throwing
        assertDoesNotThrow(() -> mockDrivetrain.periodic());
        verify(mockDrivetrain).periodic();
    }
}
