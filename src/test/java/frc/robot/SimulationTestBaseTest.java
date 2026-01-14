package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for SimulationTestBase.
 * Uses nested test classes to test both MOCK and SIMULATION modes.
 */
class SimulationTestBaseTest {

    /**
     * Tests for MOCK mode.
     */
    @Nested
    class MockModeTests extends SimulationTestBase {

        @Override
        protected TestMode getTestMode() {
            return TestMode.MOCK;
        }

        @Test
        void mockModeCreatesCorrectIO() {
            assertNotNull(mockIO, "mockIO should be created in MOCK mode");
            assertNull(simIO, "simIO should be null in MOCK mode");
            assertSame(mockIO, activeIO, "activeIO should be mockIO");
        }

        @Test
        void updateInputsReadsFromMock() {
            // Configure mock with specific values
            mockIO.withGyroYaw(45.0).withGyroConnected(true);
            mockIO.withOdometry(1.0, 2.0, Math.PI / 4);

            updateInputs();

            assertEquals(45.0, inputs.gyroYawDegrees, 0.001);
            assertTrue(inputs.gyroConnected);
            assertEquals(1.0, inputs.odometryX, 0.001);
            assertEquals(2.0, inputs.odometryY, 0.001);
        }

        @Test
        void advanceSimulationIsNoOpForMock() {
            // In mock mode, advanceSimulation should not throw
            assertDoesNotThrow(() -> advanceSimulation(1.0));
            assertDoesNotThrow(() -> advanceSimulationSteps(10));
        }

        @Test
        void getSimulationTimeTracksTime() {
            assertEquals(0.0, getSimulationTime(), 0.001);

            advanceSimulation(0.5);
            assertEquals(0.5, getSimulationTime(), 0.001);

            advanceSimulationSteps(10); // 10 * 0.02 = 0.2s
            assertEquals(0.7, getSimulationTime(), 0.001);
        }

        @Test
        void resetSimulationClearsMockState() {
            mockIO.withOdometry(5.0, 5.0, 0.0);
            advanceSimulation(1.0);

            resetSimulation();

            assertEquals(0.0, getSimulationTime(), 0.001);
            // Mock's resetAll clears odometry
            updateInputs();
            assertEquals(0.0, inputs.odometryX, 0.001);
        }

        @Test
        void assertGyroConnectedPassesWhenConnected() {
            mockIO.withGyroConnected(true);
            assertDoesNotThrow(() -> assertGyroConnected());
        }

        @Test
        void getMockIOReturnsInstance() {
            assertNotNull(getMockIO());
            assertSame(mockIO, getMockIO());
        }

        @Test
        void getSimIOReturnsNullInMockMode() {
            assertNull(getSimIO());
        }

        @Test
        void driveCallsAreTrackedByMock() {
            activeIO.driveFieldCentric(1.0, 0.0, 0.0);

            assertTrue(mockIO.wasDriveFieldCentricCalled());
            assertEquals(1, mockIO.getFieldCentricCalls().size());
        }
    }

    /**
     * Tests for SIMULATION mode.
     */
    @Nested
    class SimulationModeTests extends SimulationTestBase {

        @Override
        protected TestMode getTestMode() {
            return TestMode.SIMULATION;
        }

        @Test
        void simulationModeCreatesCorrectIO() {
            assertNotNull(simIO, "simIO should be created in SIMULATION mode");
            assertNull(mockIO, "mockIO should be null in SIMULATION mode");
            assertSame(simIO, activeIO, "activeIO should be simIO");
        }

        @Test
        void initialStateIsAtOrigin() {
            updateInputs();

            assertEquals(0.0, inputs.odometryX, 0.001);
            assertEquals(0.0, inputs.odometryY, 0.001);
            assertEquals(0.0, inputs.odometryRotationRad, 0.001);
            assertTrue(inputs.gyroConnected);
        }

        @Test
        void advanceSimulationUpdatesPhysics() {
            // Drive forward
            activeIO.driveFieldCentric(1.0, 0.0, 0.0);

            // Advance simulation
            advanceSimulation(0.5);
            updateInputs();

            // Robot should have moved forward
            assertTrue(inputs.odometryX > 0, "Robot should have positive X after driving forward");
        }

        @Test
        void advanceSimulationStepsUpdatesPhysics() {
            activeIO.driveFieldCentric(1.0, 0.0, 0.0);

            // 25 steps = 25 * 0.02 = 0.5 seconds
            advanceSimulationSteps(25);
            updateInputs();

            assertTrue(inputs.odometryX > 0);
        }

        @Test
        void resetSimulationClearsOdometry() {
            // Move the robot
            simIO.resetOdometry(new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(45)));
            advanceSimulation(0.1);

            // Reset
            resetSimulation();
            updateInputs();

            assertEquals(0.0, inputs.odometryX, 0.001);
            assertEquals(0.0, inputs.odometryY, 0.001);
            assertEquals(0.0, getSimulationTime(), 0.001);
        }

        @Test
        void assertPositionNearPassesWhenClose() {
            simIO.resetOdometry(new Pose2d(1.0, 2.0, Rotation2d.kZero));

            assertDoesNotThrow(() -> assertPositionNear(new Pose2d(1.0, 2.0, Rotation2d.kZero), 0.1));
        }

        @Test
        void assertPositionNearFailsWhenFar() {
            simIO.resetOdometry(new Pose2d(1.0, 2.0, Rotation2d.kZero));

            assertThrows(AssertionError.class, () -> assertPositionNear(new Pose2d(5.0, 5.0, Rotation2d.kZero), 0.1));
        }

        @Test
        void assertHeadingNearPassesWhenClose() {
            simIO.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(45)));

            assertDoesNotThrow(() -> assertHeadingNear(45.0, 1.0));
        }

        @Test
        void assertHeadingNearFailsWhenFar() {
            simIO.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(45)));

            assertThrows(AssertionError.class, () -> assertHeadingNear(90.0, 1.0));
        }

        @Test
        void assertMovingForwardPassesWhenDriving() {
            activeIO.driveFieldCentric(1.0, 0.0, 0.0);
            advanceSimulation(0.1);

            assertDoesNotThrow(() -> assertMovingForward());
        }

        @Test
        void assertStoppedPassesWhenStopped() {
            // Don't drive, just check stopped state
            advanceSimulation(0.1);

            assertDoesNotThrow(() -> assertStopped(0.1));
        }

        @Test
        void assertDriveCurrentsBelowPassesNormally() {
            // Simulation starts with low currents
            advanceSimulation(0.02);

            assertDoesNotThrow(() -> assertDriveCurrentsBelow(100.0));
        }

        @Test
        void assertSteerCurrentsBelowPassesNormally() {
            advanceSimulation(0.02);

            assertDoesNotThrow(() -> assertSteerCurrentsBelow(50.0));
        }

        @Test
        void getSimIOReturnsInstance() {
            assertNotNull(getSimIO());
            assertSame(simIO, getSimIO());
        }

        @Test
        void getMockIOReturnsNullInSimMode() {
            assertNull(getMockIO());
        }

        @Test
        void rotationIntegratesOverTime() {
            // Rotate at 90 deg/s
            activeIO.driveFieldCentric(0.0, 0.0, Math.toRadians(90));

            // Rotate for 1 second
            advanceSimulation(1.0);
            updateInputs();

            // Should be near 90 degrees
            double actualDegrees = Math.toDegrees(inputs.odometryRotationRad);
            assertEquals(90.0, actualDegrees, 5.0, "Robot should rotate ~90 degrees after 1 second at 90 deg/s");
        }
    }

    /**
     * Tests that verify test configuration is set up correctly.
     */
    @Nested
    class ConfigurationTests extends SimulationTestBase {

        @Override
        protected TestMode getTestMode() {
            return TestMode.MOCK;
        }

        @Test
        void testConfigIsProvided() {
            assertNotNull(testConfig, "testConfig should be provided");
            assertNotNull(testConfig.frontLeft);
            assertNotNull(testConfig.frontRight);
            assertNotNull(testConfig.backLeft);
            assertNotNull(testConfig.backRight);
        }

        @Test
        void inputsObjectIsCreated() {
            assertNotNull(inputs, "inputs should be created");
        }

        @Test
        void defaultDtIs20ms() {
            assertEquals(0.02, DEFAULT_DT_SECONDS, 0.001);
        }
    }
}
