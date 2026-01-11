package frc.robot;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;

/**
 * Base test class for drivetrain-related tests.
 * Handles HAL initialization and provides common test configuration.
 *
 * Extend this class for any tests that need to interact with
 * WPILib or CTRE Phoenix 6 components.
 */
public abstract class DrivetrainTestBase {

    protected RobotConfig testConfig;

    /**
     * Initialize the HAL once before any tests in the class run.
     * This is required for WPILib components to function in tests.
     */
    @BeforeAll
    static void initHAL() {
        TestFixtures.initializeHAL();
    }

    /**
     * Set up fresh test configuration before each test.
     * Subclasses can override this but should call super.setUp().
     */
    @BeforeEach
    void setUp() {
        testConfig = TestFixtures.createTestRobotConfig();
    }

    /**
     * Get the front-left swerve module config from the test configuration.
     * @return Front-left SwerveModuleConfig
     */
    protected SwerveModuleConfig getFrontLeftModule() {
        return testConfig.frontLeft;
    }

    /**
     * Get the front-right swerve module config from the test configuration.
     * @return Front-right SwerveModuleConfig
     */
    protected SwerveModuleConfig getFrontRightModule() {
        return testConfig.frontRight;
    }

    /**
     * Get the back-left swerve module config from the test configuration.
     * @return Back-left SwerveModuleConfig
     */
    protected SwerveModuleConfig getBackLeftModule() {
        return testConfig.backLeft;
    }

    /**
     * Get the back-right swerve module config from the test configuration.
     * @return Back-right SwerveModuleConfig
     */
    protected SwerveModuleConfig getBackRightModule() {
        return testConfig.backRight;
    }
}
