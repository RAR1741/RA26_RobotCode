// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.controls.DriverControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private final Telemetry logger = new Telemetry();

  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();

  public final SparkFlex hopperFloor; // ID 40
  public final SparkFlex kicker; // ID 41

  public RobotContainer() {
    hopperFloor = new SparkFlex(40, MotorType.kBrushless);
    // hopperFloor.configure(new SparkBaseConfig().inverted(true), null, null);
    hopperFloor.setInverted(true);
    kicker = new SparkFlex(41, MotorType.kBrushless);

    configureBindings();
  }

  private void configureBindings() {
    DriverControls.configure(Constants.ControllerConstants.kDriverControllerPort, m_swerve, logger);

    CommandXboxController oppController = new CommandXboxController(1);

    oppController.a().onTrue(Commands.runOnce(() -> {
      hopperFloor.set(0.5);
      kicker.set(0.5);
    })).onFalse(Commands.runOnce(() -> {
      hopperFloor.set(0);
      kicker.set(0);
    }));
  }

  public Command getAutonomousCommand() {
    return Commands.none();

    // Simple drive forward auton
    // final var idle = new SwerveRequest.Idle();
    // return Commands.sequence(
    // // Reset our field centric heading to match the robot
    // // facing away from our alliance station wall (0 deg).
    // m_swerve.runOnce(() -> m_swerve.seedFieldCentric(Rotation2d.kZero)),
    // // Then slowly drive forward (away from us) for 5 seconds.
    // m_swerve.applyRequest(() -> drive.withVelocityX(0.5)
    // .withVelocityY(0)
    // .withRotationalRate(0))
    // .withTimeout(5.0),
    // // Finally idle for the rest of auton
    // m_swerve.applyRequest(() -> idle));
  }

  public CommandSwerveDrivetrain getSwerveSystem() {
    return m_swerve;
  }
}
