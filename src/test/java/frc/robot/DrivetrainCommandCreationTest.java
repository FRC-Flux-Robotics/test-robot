package frc.robot;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for drivetrain command creation patterns.
 * Tests SwerveRequest creation and configuration used in RobotContainer.
 */
class DrivetrainCommandCreationTest {

    private static final double MAX_SPEED = 4.0; // m/s, example value
    private static final double MAX_ANGULAR_RATE = 4.7; // rad/s, example value

    @BeforeAll
    static void initHAL() {
        TestFixtures.initializeHAL();
    }

    @Test
    void swerveRequest_fieldCentric_createsValidRequest() {
        // Create a field-centric drive request as used in RobotContainer
        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                .withDeadband(MAX_SPEED * 0.1)
                .withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        assertNotNull(request, "FieldCentric request should not be null");
    }

    @Test
    void swerveRequest_fieldCentric_acceptsVelocityInputs() {
        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                .withDeadband(MAX_SPEED * 0.1)
                .withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Should be able to set velocity values without throwing
        SwerveRequest result = request.withVelocityX(1.0).withVelocityY(0.5).withRotationalRate(0.25);

        assertNotNull(result, "FieldCentric with velocities should not be null");
    }

    @Test
    void swerveRequest_brake_createsValidRequest() {
        // Create a brake request as used in RobotContainer
        SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        assertNotNull(brake, "SwerveDriveBrake request should not be null");
    }

    @Test
    void swerveRequest_pointWheelsAt_createsValidRequest() {
        // Create a point wheels request as used in RobotContainer
        SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        assertNotNull(point, "PointWheelsAt request should not be null");
    }

    @Test
    void swerveRequest_pointWheelsAt_acceptsRotation() {
        SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        // Should be able to set direction without throwing
        SwerveRequest result = point.withModuleDirection(new Rotation2d(0.5, 0.5));

        assertNotNull(result, "PointWheelsAt with direction should not be null");
    }

    @Test
    void swerveRequest_idle_createsValidRequest() {
        // Create an idle request as used for disabled mode
        SwerveRequest.Idle idle = new SwerveRequest.Idle();

        assertNotNull(idle, "Idle request should not be null");
    }

    @Test
    void swerveRequest_fieldCentric_withZeroDeadband() {
        // Edge case: zero deadband should still work
        SwerveRequest.FieldCentric request =
                new SwerveRequest.FieldCentric().withDeadband(0.0).withRotationalDeadband(0.0);

        assertNotNull(request, "FieldCentric with zero deadband should not be null");
    }

    @Test
    void swerveRequest_fieldCentric_withMaxValues() {
        // Edge case: maximum velocity values
        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                .withDeadband(MAX_SPEED * 0.1)
                .withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(MAX_SPEED)
                .withVelocityY(MAX_SPEED)
                .withRotationalRate(MAX_ANGULAR_RATE);

        assertNotNull(request, "FieldCentric with max values should not be null");
    }
}
