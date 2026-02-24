// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  // kSpeedAt12Volts desired top speed
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  // 3/4 of a rotation per second max angular velocity
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // OLD
    // TODO: reimplement this
    // DriverControls.configure(Constants.ControllerConstants.kDriverControllerPort,
    // m_swerve);

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_swerve.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_swerve.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        m_swerve.applyRequest(() -> idle).ignoringDisable(true));

    joystick.a().whileTrue(m_swerve.applyRequest(() -> brake));
    joystick.b().whileTrue(m_swerve.applyRequest(
        () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(m_swerve.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(m_swerve.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(m_swerve.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(m_swerve.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    joystick.leftBumper().onTrue(m_swerve.runOnce(m_swerve::seedFieldCentric));

    m_swerve.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    // Simple drive forward auton
    final var idle = new SwerveRequest.Idle();
    return Commands.sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        m_swerve.runOnce(() -> m_swerve.seedFieldCentric(Rotation2d.kZero)),
        // Then slowly drive forward (away from us) for 5 seconds.
        m_swerve.applyRequest(() -> drive.withVelocityX(0.5)
            .withVelocityY(0)
            .withRotationalRate(0))
            .withTimeout(5.0),
        // Finally idle for the rest of auton
        m_swerve.applyRequest(() -> idle));
  }

  public CommandSwerveDrivetrain getSwerveSystem() {
    return m_swerve;
  }
}
