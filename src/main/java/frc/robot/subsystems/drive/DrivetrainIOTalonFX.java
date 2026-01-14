package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * DrivetrainIO implementation for real TalonFX hardware using Phoenix 6 swerve.
 * Wraps an existing SwerveDrivetrain instance to read sensor data and send drive commands.
 */
public class DrivetrainIOTalonFX implements DrivetrainIO {

    private static final int MODULE_COUNT = 4;
    private static final double ROTATIONS_TO_RADIANS = 2.0 * Math.PI;

    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    private final Pigeon2 gyro;

    // Swerve requests (pre-allocated for reuse)
    private final SwerveRequest.FieldCentric fieldCentricRequest;
    private final SwerveRequest.RobotCentric robotCentricRequest;
    private final SwerveRequest.Idle idleRequest;

    // Cached status signals for efficient reading
    private final StatusSignal<Angle>[] drivePositionSignals;
    private final StatusSignal<AngularVelocity>[] driveVelocitySignals;
    private final StatusSignal<Voltage>[] driveVoltageSignals;
    private final StatusSignal<Current>[] driveCurrentSignals;

    private final StatusSignal<Angle>[] steerPositionSignals;
    private final StatusSignal<AngularVelocity>[] steerVelocitySignals;
    private final StatusSignal<Voltage>[] steerVoltageSignals;
    private final StatusSignal<Current>[] steerCurrentSignals;

    // Gyro status signals
    private final StatusSignal<Angle> gyroYaw;
    private final StatusSignal<AngularVelocity> gyroYawRate;
    private final StatusSignal<Angle> gyroPitch;
    private final StatusSignal<Angle> gyroRoll;

    /**
     * Creates a DrivetrainIOTalonFX that wraps an existing Phoenix 6 swerve drivetrain.
     *
     * @param drivetrain The SwerveDrivetrain to wrap (already configured)
     * @param gyro The Pigeon2 gyro used by the drivetrain
     */
    @SuppressWarnings("unchecked")
    public DrivetrainIOTalonFX(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain, Pigeon2 gyro) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;

        // Initialize swerve requests
        this.fieldCentricRequest =
                new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        this.robotCentricRequest =
                new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        this.idleRequest = new SwerveRequest.Idle();

        // Initialize signal arrays
        drivePositionSignals = new StatusSignal[MODULE_COUNT];
        driveVelocitySignals = new StatusSignal[MODULE_COUNT];
        driveVoltageSignals = new StatusSignal[MODULE_COUNT];
        driveCurrentSignals = new StatusSignal[MODULE_COUNT];

        steerPositionSignals = new StatusSignal[MODULE_COUNT];
        steerVelocitySignals = new StatusSignal[MODULE_COUNT];
        steerVoltageSignals = new StatusSignal[MODULE_COUNT];
        steerCurrentSignals = new StatusSignal[MODULE_COUNT];

        // Cache motor status signals from modules
        for (int i = 0; i < MODULE_COUNT; i++) {
            SwerveModule<TalonFX, TalonFX, CANcoder> module = drivetrain.getModule(i);
            TalonFX driveMotor = module.getDriveMotor();
            TalonFX steerMotor = module.getSteerMotor();

            drivePositionSignals[i] = driveMotor.getPosition();
            driveVelocitySignals[i] = driveMotor.getVelocity();
            driveVoltageSignals[i] = driveMotor.getMotorVoltage();
            driveCurrentSignals[i] = driveMotor.getStatorCurrent();

            steerPositionSignals[i] = steerMotor.getPosition();
            steerVelocitySignals[i] = steerMotor.getVelocity();
            steerVoltageSignals[i] = steerMotor.getMotorVoltage();
            steerCurrentSignals[i] = steerMotor.getStatorCurrent();
        }

        // Cache gyro signals
        gyroYaw = gyro.getYaw();
        gyroYawRate = gyro.getAngularVelocityZWorld();
        gyroPitch = gyro.getPitch();
        gyroRoll = gyro.getRoll();
    }

    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        // Refresh all signals in one batch for efficiency
        BaseStatusSignal.refreshAll(
                gyroYaw,
                gyroYawRate,
                gyroPitch,
                gyroRoll,
                drivePositionSignals[0],
                driveVelocitySignals[0],
                driveVoltageSignals[0],
                driveCurrentSignals[0],
                drivePositionSignals[1],
                driveVelocitySignals[1],
                driveVoltageSignals[1],
                driveCurrentSignals[1],
                drivePositionSignals[2],
                driveVelocitySignals[2],
                driveVoltageSignals[2],
                driveCurrentSignals[2],
                drivePositionSignals[3],
                driveVelocitySignals[3],
                driveVoltageSignals[3],
                driveCurrentSignals[3],
                steerPositionSignals[0],
                steerVelocitySignals[0],
                steerVoltageSignals[0],
                steerCurrentSignals[0],
                steerPositionSignals[1],
                steerVelocitySignals[1],
                steerVoltageSignals[1],
                steerCurrentSignals[1],
                steerPositionSignals[2],
                steerVelocitySignals[2],
                steerVoltageSignals[2],
                steerCurrentSignals[2],
                steerPositionSignals[3],
                steerVelocitySignals[3],
                steerVoltageSignals[3],
                steerCurrentSignals[3]);

        // Gyro data
        inputs.gyroConnected = BaseStatusSignal.isAllGood(gyroYaw, gyroYawRate, gyroPitch, gyroRoll);
        inputs.gyroYawDegrees = gyroYaw.getValue().in(Degrees);
        inputs.gyroYawRateDegPS = gyroYawRate.getValue().in(DegreesPerSecond);
        inputs.gyroPitchDegrees = gyroPitch.getValue().in(Degrees);
        inputs.gyroRollDegrees = gyroRoll.getValue().in(Degrees);

        // Module data (order: FL=0, FR=1, BL=2, BR=3)
        for (int i = 0; i < MODULE_COUNT; i++) {
            // Drive motor data - convert rotations to radians
            inputs.drivePositionsRad[i] = drivePositionSignals[i].getValue().in(Rotations) * ROTATIONS_TO_RADIANS;
            inputs.driveVelocitiesRadPerSec[i] =
                    driveVelocitySignals[i].getValue().in(RotationsPerSecond) * ROTATIONS_TO_RADIANS;
            inputs.driveAppliedVolts[i] = driveVoltageSignals[i].getValue().in(Volts);
            inputs.driveCurrentAmps[i] = driveCurrentSignals[i].getValue().in(Amps);

            // Steer motor data - convert rotations to radians
            inputs.steerPositionsRad[i] = steerPositionSignals[i].getValue().in(Rotations) * ROTATIONS_TO_RADIANS;
            inputs.steerVelocitiesRadPerSec[i] =
                    steerVelocitySignals[i].getValue().in(RotationsPerSecond) * ROTATIONS_TO_RADIANS;
            inputs.steerAppliedVolts[i] = steerVoltageSignals[i].getValue().in(Volts);
            inputs.steerCurrentAmps[i] = steerCurrentSignals[i].getValue().in(Amps);
        }

        // Odometry from drivetrain state
        SwerveDriveState state = drivetrain.getState();
        inputs.odometryX = state.Pose.getX();
        inputs.odometryY = state.Pose.getY();
        inputs.odometryRotationRad = state.Pose.getRotation().getRadians();
    }

    @Override
    public void driveFieldCentric(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {
        drivetrain.setControl(fieldCentricRequest
                .withVelocityX(vxMetersPerSec)
                .withVelocityY(vyMetersPerSec)
                .withRotationalRate(omegaRadPerSec));
    }

    @Override
    public void driveRobotCentric(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {
        drivetrain.setControl(robotCentricRequest
                .withVelocityX(vxMetersPerSec)
                .withVelocityY(vyMetersPerSec)
                .withRotationalRate(omegaRadPerSec));
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    @Override
    public void stop() {
        drivetrain.setControl(idleRequest);
    }

    @Override
    public void setOperatorPerspective(Rotation2d rotation) {
        drivetrain.setOperatorPerspectiveForward(rotation);
    }
}
