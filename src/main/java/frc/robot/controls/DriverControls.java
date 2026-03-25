package frc.robot.controls;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Robot;
import frc.robot.Simulation;
import frc.robot.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;

public class DriverControls {
  // kSpeedAt12Volts desired top speed
  private static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  private static double m_deadbandLimit = 0.01; // 1% deadband on joystick inputs

  // 3/4 of a rotation per second max angular velocity
  private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * m_deadbandLimit).withRotationalDeadband(MaxAngularRate * m_deadbandLimit)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // private static final SwerveRequest.PointWheelsAt point = new
  // SwerveRequest.PointWheelsAt();

  public static double scale(double input) {
    double numeratorFactor = (Math.abs(input) - m_deadbandLimit);
    double denominatorFactor = 1 - m_deadbandLimit;
    double sign = (input >= 0 ? 1 : -1);

    return sign * ((numeratorFactor * numeratorFactor) / (denominatorFactor * denominatorFactor));
  }

  public static void configure(int port, CommandSwerveDrivetrain drivetrain, Superstructure superstructure,
      Telemetry logger) {
    CommandXboxController controller = new CommandXboxController(port);

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> {
          double slowInput = controller.getLeftTriggerAxis(); // [0, 1]
          double boostInput = controller.getRightTriggerAxis(); // [0, 1]

          // Lerp: slow pulls from standardSpeed down to slowModeMin,
          // boost pulls from standardSpeed up to boostModeScaler
          double speedScaler = ControllerConstants.k_standardSpeed
              - slowInput * (ControllerConstants.k_standardSpeed - ControllerConstants.k_slowModeMin)
              + boostInput * (ControllerConstants.k_boostModeScaler - ControllerConstants.k_standardSpeed);

          return drive
              .withVelocityX(scale(controller.getLeftY()) * MaxSpeed * speedScaler)
              .withVelocityY(scale(controller.getLeftX()) * MaxSpeed * speedScaler)
              .withRotationalRate(scale(-controller.getRightX()) * MaxAngularRate);
        }));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    controller.x().whileTrue(drivetrain.applyRequest(() -> brake));
    // controller.b().whileTrue(drivetrain.applyRequest(
    // () -> point.withModuleDirection(new Rotation2d(-controller.getLeftY(),
    // -controller.getLeftX()))));

    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine should be run exactly once in a single log.
    // controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    // controller.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);

    // controller.povLeft().onTrue(superstructure.turretLeftCommand());
    // controller.povRight().onTrue(superstructure.turretRightCommand());

    // controller.povUp().onTrue(superstructure.turretCenterCommand());

    // controller.leftBumper().onTrue(superstructure.turretRezeroCommand().ignoringDisable(true));

    // controller.rightBumper().toggleOnTrue(
    // new ShootOnTheMoveCommand(drivetrain, superstructure, () ->
    // superstructure.getAimPoint())
    // .ignoringDisable(true)
    // .withName("OperatorControls.aimCommand"));

    if (Robot.isSimulation()) {
      // Fire fuel 10 times per second while button is held
      controller.a().whileTrue(
          Commands.repeatingSequence(
              Simulation.fireFuel(drivetrain, superstructure),
              Commands.waitSeconds(1.0)));
    }
  }
}
