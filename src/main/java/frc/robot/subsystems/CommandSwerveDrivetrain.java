package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotConfig;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.drive.DrivetrainIO;
import frc.robot.subsystems.drive.DrivetrainIOInputsAutoLogged;
import frc.robot.subsystems.drive.DrivetrainIOSim;
import frc.robot.subsystems.drive.DrivetrainIOTalonFX;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // AdvantageKit IO layer for logging
    private final DrivetrainIO io;
    private final DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();
    private DrivetrainIOSim ioSim = null; // Only non-null in simulation

    protected Pigeon2 gyro;
    protected SwerveDriveKinematics kinematics;
    protected SwerveDriveOdometry odometry;
    protected Pose2d initPose = new Pose2d();
    protected Pose2d currentPose;

    // Safety: Emergency Stop
    /** Emergency stop flag - volatile for thread safety across periodic loops. */
    private volatile boolean emergencyStopActive = false;

    /** Request to apply when emergency stop is active. */
    private final SwerveRequest.Idle emergencyStopRequest = new SwerveRequest.Idle();

    // Safety: Brownout Protection
    /** Current speed multiplier for brownout protection. */
    private double speedMultiplier = 1.0;

    /** Whether brownout protection is currently active. */
    private boolean brownoutProtectionActive = false;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
            new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
            new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
            new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            RobotConfig config,
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        gyro = getPigeon2();
        initOdometry(
                new Translation2d(config.frontLeft.xPos, config.frontLeft.yPos),
                new Translation2d(config.frontRight.xPos, config.frontRight.yPos),
                new Translation2d(config.backLeft.xPos, config.backLeft.yPos),
                new Translation2d(config.backRight.xPos, config.backRight.yPos));

        // Initialize AdvantageKit IO layer
        if (Utils.isSimulation()) {
            ioSim = new DrivetrainIOSim();
            io = ioSim;
            startSimThread();
        } else {
            io = new DrivetrainIOTalonFX(this, gyro);
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            RobotConfig config,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);

        gyro = getPigeon2();
        initOdometry(
                new Translation2d(config.frontLeft.xPos, config.frontLeft.yPos),
                new Translation2d(config.frontRight.xPos, config.frontRight.yPos),
                new Translation2d(config.backLeft.xPos, config.backLeft.yPos),
                new Translation2d(config.backRight.xPos, config.backRight.yPos));

        // Initialize AdvantageKit IO layer
        if (Utils.isSimulation()) {
            ioSim = new DrivetrainIOSim();
            io = ioSim;
            startSimThread();
        } else {
            io = new DrivetrainIOTalonFX(this, gyro);
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            RobotConfig config,
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                odometryStandardDeviation,
                visionStandardDeviation,
                modules);

        gyro = getPigeon2();
        initOdometry(
                new Translation2d(config.frontLeft.xPos, config.frontLeft.yPos),
                new Translation2d(config.frontRight.xPos, config.frontRight.yPos),
                new Translation2d(config.backLeft.xPos, config.backLeft.yPos),
                new Translation2d(config.backRight.xPos, config.backRight.yPos));

        // Initialize AdvantageKit IO layer
        if (Utils.isSimulation()) {
            ioSim = new DrivetrainIOSim();
            io = ioSim;
            startSimThread();
        } else {
            io = new DrivetrainIOTalonFX(this, gyro);
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        // Safety: If emergency stop is active, ensure motors remain stopped
        if (emergencyStopActive) {
            io.stop();
            super.setControl(emergencyStopRequest);
            // Still log telemetry but skip normal operations
            io.updateInputs(inputs);
            Logger.processInputs("Drive", inputs);
            return;
        }

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // Safety: Update brownout protection
        updateBrownoutProtection();

        // AdvantageKit: Update and log drivetrain inputs
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        // Update pose from Phoenix 6 built-in 250Hz odometry
        currentPose = getState().Pose;

        SmartDashboard.putNumber("Position_X", currentPose.getX());
        SmartDashboard.putNumber("Position_Y", currentPose.getY());
        SmartDashboard.putNumber("Rotation_Deg", currentPose.getRotation().getDegrees());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());

            // Update AdvantageKit IO simulation
            if (ioSim != null) {
                ioSim.updateSimulation(deltaTime);
            }
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(
                visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    protected void initOdometry(
            Translation2d frontLeft, Translation2d frontRight, Translation2d backLeft, Translation2d backRight) {
        kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);

        SwerveDriveState driveState = getState();

        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), driveState.ModulePositions, initPose);
    }

    public Pose2d getPosition() {
        return currentPose; // odometry.getPoseMeters();
    }

    /**
     * Returns the Pigeon2 gyro used by this drivetrain.
     *
     * @return The Pigeon2 gyro instance
     */
    public Pigeon2 getGyro() {
        return gyro;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getState().ModulePositions, pose);
    }

    // ==================== Safety Methods ====================

    /**
     * Triggers the emergency stop. All motors will be immediately zeroed
     * and remain stopped until {@link #resetEmergencyStop()} is called.
     *
     * <p>This is triggered by pressing both controller bumpers simultaneously.
     */
    public void triggerEmergencyStop() {
        emergencyStopActive = true;
        io.stop();
        SmartDashboard.putBoolean(Constants.SafetyConstants.EMERGENCY_STOP_KEY, true);
        DriverStation.reportWarning("EMERGENCY STOP ACTIVATED", false);
    }

    /**
     * Resets the emergency stop, allowing normal robot operation to resume.
     * Should only be called after the cause of the emergency stop is resolved.
     */
    public void resetEmergencyStop() {
        emergencyStopActive = false;
        SmartDashboard.putBoolean(Constants.SafetyConstants.EMERGENCY_STOP_KEY, false);
        DriverStation.reportWarning("Emergency stop reset - robot operational", false);
    }

    /**
     * Checks if the emergency stop is currently active.
     *
     * @return true if emergency stop is active, false otherwise
     */
    public boolean isEmergencyStopActive() {
        return emergencyStopActive;
    }

    /**
     * Updates brownout protection state based on battery voltage.
     * Called from periodic().
     */
    private void updateBrownoutProtection() {
        double voltage = RobotController.getBatteryVoltage();
        SmartDashboard.putNumber(Constants.SafetyConstants.BATTERY_VOLTAGE_KEY, voltage);

        if (!brownoutProtectionActive) {
            // Check if we need to activate brownout protection
            if (voltage < Constants.SafetyConstants.BROWNOUT_VOLTAGE_THRESHOLD) {
                brownoutProtectionActive = true;
                speedMultiplier = Constants.SafetyConstants.BROWNOUT_SPEED_MULTIPLIER;
                SmartDashboard.putBoolean(Constants.SafetyConstants.BROWNOUT_WARNING_KEY, true);
                DriverStation.reportWarning(
                        "Low battery voltage (" + String.format("%.1f", voltage) + "V) - speed reduced to 50%", false);
            }
        } else {
            // Check if we can deactivate brownout protection (with hysteresis)
            if (voltage > Constants.SafetyConstants.BROWNOUT_RECOVERY_VOLTAGE) {
                brownoutProtectionActive = false;
                speedMultiplier = 1.0;
                SmartDashboard.putBoolean(Constants.SafetyConstants.BROWNOUT_WARNING_KEY, false);
            }
        }
    }

    /**
     * Gets the current speed multiplier based on brownout protection state.
     *
     * @return Speed multiplier between 0.0 and 1.0
     */
    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    /**
     * Checks if brownout protection is currently active.
     *
     * @return true if brownout protection is limiting speed
     */
    public boolean isBrownoutProtectionActive() {
        return brownoutProtectionActive;
    }

    /**
     * Overrides setControl to respect emergency stop.
     * When emergency stop is active, all control requests are blocked.
     *
     * @param request The swerve request to apply
     */
    @Override
    public void setControl(SwerveRequest request) {
        // Safety: Block all control requests during emergency stop
        if (emergencyStopActive) {
            super.setControl(emergencyStopRequest);
            return;
        }
        super.setControl(request);
    }
}
