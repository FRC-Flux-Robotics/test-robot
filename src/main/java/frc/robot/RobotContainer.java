// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.DriveForwardAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  public final SwerveDrivetrainConstants DrivetrainConstants;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public final CommandSwerveDrivetrain drivetrain;

  // Autonomous command
  private final DriveForwardAuto autoDriveForward;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  protected final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DriverControllerPort);

  double xStartPos = OperatorConstants.xStartPos;
  double xMiddlePos = OperatorConstants.xMiddlePos;
  double yStartPos = OperatorConstants.yStartPos;
  double yMiddlePos = OperatorConstants.yMiddlePos;
  double yMaxPos = OperatorConstants.yMaxPos;

  private final PiecewiseSensitivity sensitivityPos =
      new PiecewiseSensitivity(OperatorConstants.xStartPos, OperatorConstants.xMiddlePos, OperatorConstants.yStartPos, OperatorConstants.yMiddlePos, OperatorConstants.yMaxPos);

  private final PiecewiseSensitivity sensitivityRot =
    new PiecewiseSensitivity(OperatorConstants.xStartRot, OperatorConstants.xMiddleRot, OperatorConstants.yStartRot, OperatorConstants.yMiddleRot, OperatorConstants.yMaxRot);

  protected final Telemetry logger = new Telemetry(MaxSpeed);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(RobotConfig config)
  {
    DrivetrainConstants = new SwerveDrivetrainConstants()
      .withCANBusName(config.driveCANBus)
      .withPigeon2Id(config.pigeonId)
      .withPigeon2Configs(config.pigeonConfigs);

    drivetrain = createDrivetrain(config);
    drivetrain.registerTelemetry(logger::telemeterize);

    // Create autonomous command
    autoDriveForward = new DriveForwardAuto(drivetrain);

    SmartDashboard.putNumber("Start_X", xStartPos);
    SmartDashboard.putNumber("Middle_X", xMiddlePos);
    SmartDashboard.putNumber("Start_Y", yStartPos);
    SmartDashboard.putNumber("Middle_Y", yMiddlePos);
    SmartDashboard.putNumber("Max_Y", yMaxPos);

    // Configure trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  protected void configureBindings()
  {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
     // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> {
        double xStart = SmartDashboard.getNumber("Start_X", OperatorConstants.xStartPos);
        double xMiddle = SmartDashboard.getNumber("Middle_X", OperatorConstants.xMiddlePos);
        double yStart = SmartDashboard.getNumber("Start_Y", OperatorConstants.yStartPos);
        double yMiddle = SmartDashboard.getNumber("Middle_Y", OperatorConstants.yMiddlePos);
        double yMax = SmartDashboard.getNumber("Max_Y", OperatorConstants.yMaxPos);

        boolean changed = xStartPos != xStart;
        if (changed)
          xStartPos = xStart;
        changed = changed || xMiddlePos != xMiddle;
        if (xMiddlePos != xMiddle)
          xMiddlePos = xMiddle;
        changed = changed || yStartPos != yStart;
        if (yStartPos != yStart)
          yStartPos = yStart;
        changed = changed || yMiddlePos != yMiddle;
        if (yMiddlePos != yMiddle)
          yMiddlePos = yMiddle;
        changed = changed || yMaxPos != yMax;
        if (yMaxPos != yMax)
          yMaxPos = yMax;
        if (changed)
          sensitivityPos.set(xStartPos, xMiddlePos, yStartPos, yMiddlePos, yMaxPos);

        SmartDashboard.putNumber("Joystick_X", driverController.getLeftY());
        SmartDashboard.putNumber("Joystick_Y", driverController.getLeftX());

        SmartDashboard.putNumber("Joystick_OutX", -sensitivityPos.transfer(driverController.getLeftY()));
        SmartDashboard.putNumber("Joystick_OutY", -sensitivityPos.transfer(driverController.getLeftX()));

        return drive.withVelocityX(
            // Drive forward with negative Y (forward)
           -MaxSpeed * sensitivityPos.transfer(driverController.getLeftY())
          )
          .withVelocityY(
            // Drive left with negative X (left)
            -MaxSpeed * sensitivityPos.transfer(driverController.getLeftX())
          )
          .withRotationalRate(
            MaxAngularRate * sensitivityRot.transfer(-driverController.getRightX())
          );
        }
      )
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.b().whileTrue(drivetrain.applyRequest(() ->
      point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
    ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on right bumper press
    driverController.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoDriveForward;
  }


  private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> createModuleConstants(SwerveModuleConfig constants)
  {
    return TunerConstants.ConstantCreator.createModuleConstants(
      constants.steerMotorId, constants.driveMotorId, constants.encoderId,
      constants.encoderOffset,
      constants.xPos, constants.yPos,
      constants.invertSide, constants.steerMotorInverted, constants.encoderInverted
    );
  }

  // Creates a CommandSwerveDrivetrain instance.
  // This should only be called once in your robot program,.
  private CommandSwerveDrivetrain createDrivetrain(RobotConfig config)
  {
    return new CommandSwerveDrivetrain(
        config, DrivetrainConstants,
        createModuleConstants(config.frontLeft),
        createModuleConstants(config.frontRight),
        createModuleConstants(config.backLeft),
        createModuleConstants(config.backRight)
    );
  }
}
