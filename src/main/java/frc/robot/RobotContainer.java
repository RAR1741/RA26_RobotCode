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
import frc.robot.auto.AutoMaker;

public class RobotContainer {
    private final Telemetry logger = new Telemetry();

    private final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
    private final Superstructure superstructure = new Superstructure();

    private final AutoMaker m_auto = new AutoMaker(swerve);
  
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
      var routine = m_auto.get().newRoutine("Routine");

      var traj = routine.trajectory("NewPath");

      routine.active().onTrue(
        Commands.sequence(
          traj.resetOdometry(),
          traj.cmd()
        )
      );

      traj.atTime("Event").onTrue(Commands.print("Event Fired"));
      traj.done().onTrue(Commands.print("Event Done"));

      return routine.cmd();
    }

  public CommandSwerveDrivetrain getSwerveSystem() {
    return swerve;
  }
}
