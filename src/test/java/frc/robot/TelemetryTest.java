package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for the Telemetry class.
 * Tests telemetry initialization and state publishing.
 */
class TelemetryTest {

    @BeforeAll
    static void initHAL() {
        TestFixtures.initializeHAL();
    }

    @Test
    void constructor_withPositiveMaxSpeed_succeeds() {
        // Standard use case with typical FRC max speed
        assertDoesNotThrow(
                () -> new Telemetry(4.5),
                "Telemetry should construct with positive max speed");
    }

    @Test
    void constructor_withZeroMaxSpeed_succeeds() {
        // Edge case - zero max speed should not throw
        assertDoesNotThrow(
                () -> new Telemetry(0.0),
                "Telemetry should construct with zero max speed");
    }

    @Test
    void constructor_withNegativeMaxSpeed_succeeds() {
        // Edge case - negative speed (unusual but should not crash)
        assertDoesNotThrow(
                () -> new Telemetry(-1.0),
                "Telemetry should construct with negative max speed");
    }

    @Test
    void telemeterize_withValidState_doesNotThrow() {
        Telemetry telemetry = new Telemetry(4.5);
        SwerveDriveState state = createValidDriveState();

        assertDoesNotThrow(
                () -> telemetry.telemeterize(state),
                "telemeterize should not throw with valid state");
    }

    @Test
    void telemeterize_withZeroVelocities_doesNotThrow() {
        Telemetry telemetry = new Telemetry(4.5);
        SwerveDriveState state = createZeroVelocityState();

        assertDoesNotThrow(
                () -> telemetry.telemeterize(state),
                "telemeterize should handle zero velocities");
    }

    @Test
    void telemeterize_calledMultipleTimes_doesNotThrow() {
        Telemetry telemetry = new Telemetry(4.5);
        SwerveDriveState state = createValidDriveState();

        // Simulate multiple telemetry cycles
        assertDoesNotThrow(
                () -> {
                    for (int i = 0; i < 10; i++) {
                        telemetry.telemeterize(state);
                    }
                },
                "telemeterize should handle repeated calls");
    }

    /**
     * Creates a valid SwerveDriveState for testing.
     * All fields populated with reasonable test values.
     */
    private SwerveDriveState createValidDriveState() {
        SwerveDriveState state = new SwerveDriveState();
        state.Pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45.0));
        state.Speeds = new ChassisSpeeds(1.0, 0.5, 0.1);
        state.ModuleStates = createModuleStates(1.0);
        state.ModuleTargets = createModuleStates(1.2);
        state.ModulePositions = createModulePositions();
        state.Timestamp = 1.234;
        state.OdometryPeriod = 0.02;
        return state;
    }

    /**
     * Creates a SwerveDriveState with zero velocities.
     */
    private SwerveDriveState createZeroVelocityState() {
        SwerveDriveState state = new SwerveDriveState();
        state.Pose = new Pose2d();
        state.Speeds = new ChassisSpeeds();
        state.ModuleStates = createModuleStates(0.0);
        state.ModuleTargets = createModuleStates(0.0);
        state.ModulePositions = createModulePositions();
        state.Timestamp = 0.0;
        state.OdometryPeriod = 0.02;
        return state;
    }

    /**
     * Creates an array of 4 SwerveModuleState objects.
     */
    private SwerveModuleState[] createModuleStates(double speed) {
        return new SwerveModuleState[] {
            new SwerveModuleState(speed, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(speed, Rotation2d.fromDegrees(90)),
            new SwerveModuleState(speed, Rotation2d.fromDegrees(180)),
            new SwerveModuleState(speed, Rotation2d.fromDegrees(270))
        };
    }

    /**
     * Creates an array of 4 SwerveModulePosition objects.
     */
    private SwerveModulePosition[] createModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, Rotation2d.fromDegrees(0)),
            new SwerveModulePosition(0.0, Rotation2d.fromDegrees(90)),
            new SwerveModulePosition(0.0, Rotation2d.fromDegrees(180)),
            new SwerveModulePosition(0.0, Rotation2d.fromDegrees(270))
        };
    }
}
