// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controls.DriverControls;
import frc.robot.controls.OperatorControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;

import frc.robot.auto.CustomAutoFactory;
import frc.robot.auto.CustomAutoChooser;
import frc.robot.auto.Binder;

public class RobotContainer {
    private final Telemetry logger = new Telemetry();

    private final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
    private final Superstructure superstructure = new Superstructure();

    private AutoFactory m_factory;
    private AutoChooser m_chooser;
    private Binder m_binder;
  
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
      m_factory = new CustomAutoFactory(this.getSwerveSystem()).getFactory();
      m_chooser = new CustomAutoChooser().getAutoChooser();
      m_binder = new Binder(m_factory);

      Command command = Commands.print("printed");

      m_binder.bind("Print", command);
      m_chooser.addCmd("Print", () -> command);

      return m_factory.trajectoryCmd("Print");
    
    //return Commands.none();
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
