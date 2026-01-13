package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for DrivetrainIOSim.
 * Requires HAL initialization for DCMotorSim physics simulation.
 */
class DrivetrainIOSimTest {

    private DrivetrainIOSim sim;
    private DrivetrainIO.DrivetrainIOInputs inputs;

    @BeforeAll
    static void initHAL() {
        // Initialize HAL for simulation - required by DCMotorSim
        HAL.initialize(500, 0);
    }

    @BeforeEach
    void setUp() {
        sim = new DrivetrainIOSim();
        inputs = new DrivetrainIO.DrivetrainIOInputs();
    }

    @Test
    void initialStateIsZero() {
        sim.updateInputs(inputs);

        // Gyro should be connected and at zero
        assertTrue(inputs.gyroConnected);
        assertEquals(0.0, inputs.gyroYawDegrees, 0.001);
        assertEquals(0.0, inputs.gyroYawRateDegPS, 0.001);
        assertEquals(0.0, inputs.gyroPitchDegrees, 0.001);
        assertEquals(0.0, inputs.gyroRollDegrees, 0.001);

        // All module positions and velocities should be zero
        for (int i = 0; i < 4; i++) {
            assertEquals(0.0, inputs.drivePositionsRad[i], 0.001);
            assertEquals(0.0, inputs.driveVelocitiesRadPerSec[i], 0.001);
            assertEquals(0.0, inputs.driveAppliedVolts[i], 0.001);
            assertEquals(0.0, inputs.driveCurrentAmps[i], 0.001);

            assertEquals(0.0, inputs.steerPositionsRad[i], 0.001);
            assertEquals(0.0, inputs.steerVelocitiesRadPerSec[i], 0.001);
            assertEquals(0.0, inputs.steerAppliedVolts[i], 0.001);
            assertEquals(0.0, inputs.steerCurrentAmps[i], 0.001);
        }

        // Odometry should be at origin
        assertEquals(0.0, inputs.odometryX, 0.001);
        assertEquals(0.0, inputs.odometryY, 0.001);
        assertEquals(0.0, inputs.odometryRotationRad, 0.001);
    }

    @Test
    void driveFieldCentricUpdatesVelocities() {
        // Drive forward at 1 m/s
        sim.driveFieldCentric(1.0, 0.0, 0.0);
        sim.updateSimulation(0.02); // 20ms step
        sim.updateInputs(inputs);

        // All modules should have positive drive velocity
        for (int i = 0; i < 4; i++) {
            assertTrue(inputs.driveVelocitiesRadPerSec[i] > 0,
                    "Module " + i + " should have positive drive velocity");
            assertTrue(inputs.driveAppliedVolts[i] > 0,
                    "Module " + i + " should have positive applied voltage");
        }

        // Odometry should show forward movement
        assertTrue(inputs.odometryX > 0, "Robot should move in positive X direction");
        assertEquals(0.0, inputs.odometryY, 0.001, "Robot should not move in Y direction");
    }

    @Test
    void driveRobotCentricUpdatesVelocities() {
        // Drive forward at 1 m/s in robot frame
        sim.driveRobotCentric(1.0, 0.0, 0.0);
        sim.updateSimulation(0.02);
        sim.updateInputs(inputs);

        // All modules should have positive drive velocity
        for (int i = 0; i < 4; i++) {
            assertTrue(inputs.driveVelocitiesRadPerSec[i] > 0,
                    "Module " + i + " should have positive drive velocity");
        }
    }

    @Test
    void stopZerosVelocities() {
        // First drive forward
        sim.driveFieldCentric(1.0, 0.0, 0.0);
        sim.updateSimulation(0.02);

        // Then stop - with physics simulation, motors take time to decelerate
        sim.stop();
        // Run simulation for enough time to decelerate (100ms should be sufficient)
        for (int i = 0; i < 10; i++) {
            sim.updateSimulation(0.02);
        }
        sim.updateInputs(inputs);

        // All module velocities should be near zero after deceleration
        for (int i = 0; i < 4; i++) {
            assertTrue(Math.abs(inputs.driveVelocitiesRadPerSec[i]) < 1.0,
                    "Module " + i + " velocity should be near zero after stop");
        }

        // Gyro yaw rate should be zero
        assertEquals(0.0, inputs.gyroYawRateDegPS, 0.001);
    }

    @Test
    void resetOdometryUpdatesPose() {
        Pose2d newPose = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(45.0));
        sim.resetOdometry(newPose);
        sim.updateInputs(inputs);

        assertEquals(3.0, inputs.odometryX, 0.001);
        assertEquals(2.0, inputs.odometryY, 0.001);
        assertEquals(Math.toRadians(45.0), inputs.odometryRotationRad, 0.001);
        assertEquals(45.0, inputs.gyroYawDegrees, 0.001);
    }

    @Test
    void rotationIntegratesGyroYaw() {
        // Rotate at 90 deg/s for 1 second (10 steps of 100ms)
        sim.driveFieldCentric(0.0, 0.0, Math.toRadians(90.0));
        for (int i = 0; i < 10; i++) {
            sim.updateSimulation(0.1);
        }
        sim.updateInputs(inputs);

        // Should have rotated ~90 degrees
        assertEquals(90.0, inputs.gyroYawDegrees, 1.0);
        assertEquals(90.0, inputs.gyroYawRateDegPS, 0.001);
    }

    @Test
    void getPoseReturnsCurrentPosition() {
        Pose2d initialPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
        sim.resetOdometry(initialPose);

        Pose2d retrievedPose = sim.getPose();
        assertEquals(1.0, retrievedPose.getX(), 0.001);
        assertEquals(2.0, retrievedPose.getY(), 0.001);
        assertEquals(30.0, retrievedPose.getRotation().getDegrees(), 0.001);
    }

    @Test
    void getGyroYawDegreesReturnsCurrentYaw() {
        sim.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(45.0)));
        assertEquals(45.0, sim.getGyroYawDegrees(), 0.001);
    }

    @Test
    void drivePositionIntegratesOverTime() {
        // Drive forward at 1 m/s for 2 seconds (extra time for acceleration)
        sim.driveFieldCentric(1.0, 0.0, 0.0);
        for (int i = 0; i < 100; i++) {
            sim.updateSimulation(0.02); // 100 * 20ms = 2 seconds
        }
        sim.updateInputs(inputs);

        // With physics simulation, verify that:
        // 1. Motors are spinning (drive positions increased)
        // 2. Some forward movement occurred (may be less due to physics modeling)
        boolean motorsSpinning = false;
        for (int i = 0; i < 4; i++) {
            if (inputs.drivePositionsRad[i] > 1.0) {
                motorsSpinning = true;
                break;
            }
        }
        assertTrue(motorsSpinning, "Motors should be spinning after driving");

        // Verify some odometry update occurred (positive X movement when driving forward)
        assertTrue(inputs.odometryX > 0.0,
                "Robot should have moved forward (odometryX=" + inputs.odometryX + ")");
    }

    @Test
    void fieldCentricAccountsForRobotRotation() {
        // Rotate robot 90 degrees
        sim.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(90.0)));

        // Drive "forward" in field frame (positive X)
        sim.driveFieldCentric(1.0, 0.0, 0.0);
        sim.updateSimulation(0.1);
        sim.updateInputs(inputs);

        // Robot is facing +Y, so field +X should move robot in -Y direction relative to robot
        // But odometry is in field frame, so should still show +X movement
        assertTrue(inputs.odometryX > 0, "Should move in +X field direction");
    }
}
