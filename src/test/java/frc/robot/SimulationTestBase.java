package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.DrivetrainIO;
import frc.robot.subsystems.drive.DrivetrainIOMock;
import frc.robot.subsystems.drive.DrivetrainIOSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;

/**
 * Base class for simulation-aware tests.
 * Provides automatic HAL initialization, simulation time management,
 * and support for both mock and physics-based testing.
 *
 * <p>Subclasses must implement {@link #getTestMode()} to specify whether
 * to use mock IO (fast, deterministic) or simulation IO (physics-based).
 *
 * <p>Example usage:
 * <pre>
 * class MyDrivetrainTest extends SimulationTestBase {
 *     &#64;Override
 *     protected TestMode getTestMode() {
 *         return TestMode.SIMULATION;
 *     }
 *
 *     &#64;Test
 *     void driveForwardMovesRobot() {
 *         activeIO.driveFieldCentric(1.0, 0.0, 0.0);
 *         advanceSimulation(1.0); // 1 second
 *         updateInputs();
 *         assertTrue(inputs.odometryX > 0);
 *     }
 * }
 * </pre>
 */
public abstract class SimulationTestBase {

    /** Test mode selection for subclasses. */
    protected enum TestMode {
        /** Use DrivetrainIOMock for fast, deterministic tests. */
        MOCK,
        /** Use DrivetrainIOSim for physics-based simulation tests. */
        SIMULATION
    }

    /** Default time step in seconds (20ms = 50Hz, matching robot loop). */
    protected static final double DEFAULT_DT_SECONDS = 0.02;

    /** Simulation IO instance (created when mode is SIMULATION). */
    protected DrivetrainIOSim simIO;

    /** Mock IO instance (created when mode is MOCK). */
    protected DrivetrainIOMock mockIO;

    /** Active IO interface - points to either simIO or mockIO based on test mode. */
    protected DrivetrainIO activeIO;

    /** Input container for reading drivetrain state. */
    protected DrivetrainIO.DrivetrainIOInputs inputs;

    /** Test robot configuration. */
    protected RobotConfig testConfig;

    /** Accumulated simulation time in seconds. */
    private double simulationTime = 0.0;

    /**
     * Initialize the WPILib HAL for simulation mode.
     * Called once before any tests in the class run.
     */
    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    /**
     * Set up fresh IO instances before each test.
     * Creates the appropriate IO based on {@link #getTestMode()}.
     */
    @BeforeEach
    void setUpSimulation() {
        testConfig = TestFixtures.createTestRobotConfig();
        inputs = new DrivetrainIO.DrivetrainIOInputs();
        simulationTime = 0.0;

        // Create IO based on test mode
        TestMode mode = getTestMode();
        switch (mode) {
            case SIMULATION:
                simIO = new DrivetrainIOSim();
                mockIO = null;
                activeIO = simIO;
                break;
            case MOCK:
            default:
                mockIO = new DrivetrainIOMock();
                simIO = null;
                activeIO = mockIO;
                break;
        }
    }

    /**
     * Returns the test mode for this test class.
     * Subclasses must override to specify mock or simulation mode.
     *
     * @return The test mode to use
     */
    protected abstract TestMode getTestMode();

    /**
     * Advances the simulation by the specified time.
     * In SIMULATION mode, steps the physics simulation.
     * In MOCK mode, this is a no-op (mock doesn't simulate physics).
     *
     * @param seconds Time to advance in seconds
     */
    protected void advanceSimulation(double seconds) {
        if (simIO != null) {
            int steps = (int) Math.ceil(seconds / DEFAULT_DT_SECONDS);
            for (int i = 0; i < steps; i++) {
                simIO.updateSimulation(DEFAULT_DT_SECONDS);
            }
        }
        simulationTime += seconds;
    }

    /**
     * Advances the simulation by the specified number of time steps.
     * Each step is 20ms (DEFAULT_DT_SECONDS).
     *
     * @param steps Number of 20ms steps to advance
     */
    protected void advanceSimulationSteps(int steps) {
        if (simIO != null) {
            for (int i = 0; i < steps; i++) {
                simIO.updateSimulation(DEFAULT_DT_SECONDS);
            }
        }
        simulationTime += steps * DEFAULT_DT_SECONDS;
    }

    /**
     * Updates the inputs from the active IO.
     * Call this after advancing simulation to read the new state.
     */
    protected void updateInputs() {
        activeIO.updateInputs(inputs);
    }

    /**
     * Gets the current simulation time in seconds.
     *
     * @return Accumulated simulation time
     */
    protected double getSimulationTime() {
        return simulationTime;
    }

    /**
     * Resets the simulation to initial state.
     * For MOCK mode, resets all tracking.
     * For SIMULATION mode, resets odometry to origin.
     */
    protected void resetSimulation() {
        simulationTime = 0.0;
        if (mockIO != null) {
            mockIO.resetAll();
        }
        if (simIO != null) {
            simIO.resetOdometry(new Pose2d());
        }
    }

    /**
     * Asserts that the current odometry position is near the expected position.
     *
     * @param expected Expected position
     * @param toleranceMeters Position tolerance in meters
     */
    protected void assertPositionNear(Pose2d expected, double toleranceMeters) {
        updateInputs();
        double dx = inputs.odometryX - expected.getX();
        double dy = inputs.odometryY - expected.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
        assertTrue(
                distance <= toleranceMeters,
                String.format(
                        "Position (%.3f, %.3f) should be within %.3f m of (%.3f, %.3f), actual distance: %.3f m",
                        inputs.odometryX,
                        inputs.odometryY,
                        toleranceMeters,
                        expected.getX(),
                        expected.getY(),
                        distance));
    }

    /**
     * Asserts that the current heading is near the expected heading.
     *
     * @param expectedDegrees Expected heading in degrees
     * @param toleranceDegrees Heading tolerance in degrees
     */
    protected void assertHeadingNear(double expectedDegrees, double toleranceDegrees) {
        updateInputs();
        double actualDegrees = Math.toDegrees(inputs.odometryRotationRad);
        double diff = Math.abs(normalizeAngleDegrees(actualDegrees - expectedDegrees));
        assertTrue(
                diff <= toleranceDegrees,
                String.format(
                        "Heading %.1f deg should be within %.1f deg of %.1f deg, actual diff: %.1f deg",
                        actualDegrees, toleranceDegrees, expectedDegrees, diff));
    }

    /**
     * Asserts that all drive motor currents are below the specified limit.
     *
     * @param maxAmps Maximum allowed current in amps
     */
    protected void assertDriveCurrentsBelow(double maxAmps) {
        updateInputs();
        for (int i = 0; i < 4; i++) {
            assertTrue(
                    inputs.driveCurrentAmps[i] <= maxAmps,
                    String.format(
                            "Module %d drive current %.1f A exceeds limit %.1f A",
                            i, inputs.driveCurrentAmps[i], maxAmps));
        }
    }

    /**
     * Asserts that all steer motor currents are below the specified limit.
     *
     * @param maxAmps Maximum allowed current in amps
     */
    protected void assertSteerCurrentsBelow(double maxAmps) {
        updateInputs();
        for (int i = 0; i < 4; i++) {
            assertTrue(
                    inputs.steerCurrentAmps[i] <= maxAmps,
                    String.format(
                            "Module %d steer current %.1f A exceeds limit %.1f A",
                            i, inputs.steerCurrentAmps[i], maxAmps));
        }
    }

    /**
     * Asserts that at least one module has positive drive velocity,
     * indicating the robot is moving forward.
     */
    protected void assertMovingForward() {
        updateInputs();
        boolean moving = false;
        for (int i = 0; i < 4; i++) {
            if (inputs.driveVelocitiesRadPerSec[i] > 0.1) {
                moving = true;
                break;
            }
        }
        assertTrue(moving, "Robot should be moving forward (positive drive velocity)");
    }

    /**
     * Asserts that all drive velocities are near zero (robot is stopped).
     *
     * @param toleranceRadPerSec Velocity tolerance in rad/s
     */
    protected void assertStopped(double toleranceRadPerSec) {
        updateInputs();
        for (int i = 0; i < 4; i++) {
            assertTrue(
                    Math.abs(inputs.driveVelocitiesRadPerSec[i]) <= toleranceRadPerSec,
                    String.format(
                            "Module %d velocity %.2f rad/s should be near zero (tolerance %.2f)",
                            i, inputs.driveVelocitiesRadPerSec[i], toleranceRadPerSec));
        }
    }

    /**
     * Asserts that the gyro is connected.
     */
    protected void assertGyroConnected() {
        updateInputs();
        assertTrue(inputs.gyroConnected, "Gyro should be connected");
    }

    /**
     * Gets the mock IO instance (for verification in MOCK mode).
     *
     * @return The DrivetrainIOMock, or null if in SIMULATION mode
     */
    protected DrivetrainIOMock getMockIO() {
        return mockIO;
    }

    /**
     * Gets the simulation IO instance (for direct access in SIMULATION mode).
     *
     * @return The DrivetrainIOSim, or null if in MOCK mode
     */
    protected DrivetrainIOSim getSimIO() {
        return simIO;
    }

    /**
     * Normalizes an angle to the range [-180, 180) degrees.
     */
    private static double normalizeAngleDegrees(double degrees) {
        degrees = degrees % 360;
        if (degrees >= 180) degrees -= 360;
        if (degrees < -180) degrees += 360;
        return degrees;
    }
}
