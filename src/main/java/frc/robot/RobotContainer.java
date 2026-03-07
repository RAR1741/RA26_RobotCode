// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controls.DriverControls;
import frc.robot.controls.OperatorControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;

public class RobotContainer {
  private final Telemetry logger = new Telemetry();

  private final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
  private final Superstructure superstructure = new Superstructure();

  public RobotContainer() {
    configureBindings();
    buildNamedAutoCommands();
  }

  private void configureBindings() {
    DriverControls.configure(Constants.ControllerConstants.kDriverControllerPort, swerve, superstructure, logger);
    OperatorControls.configure(Constants.ControllerConstants.kOperatorControllerPort, swerve, superstructure);
  }

  private void buildNamedAutoCommands() {
    // Add any auto commands to the NamedCommands here
    // NamedCommands.registerCommand("driveForwards",
    // drivebase.driveForward().withTimeout(2)
    // .withName("Auto.driveForwards"));
  }

  public Command getAutonomousCommand() {
    return Commands.none();

    // Simple drive forward auton
    // final var idle = new SwerveRequest.Idle();
    // return Commands.sequence(
    // // Reset our field centric heading to match the robot
    // // facing away from our alliance station wall (0 deg).
    // swerve.runOnce(() -> swerve.seedFieldCentric(Rotation2d.kZero)),
    // // Then slowly drive forward (away from us) for 5 seconds.
    // swerve.applyRequest(() -> drive.withVelocityX(0.5)
    // .withVelocityY(0)
    // .withRotationalRate(0))
    // .withTimeout(5.0),
    // // Finally idle for the rest of auton
    // swerve.applyRequest(() -> idle));
  }

  public CommandSwerveDrivetrain getSwerveSystem() {
    return swerve;
  }
}
