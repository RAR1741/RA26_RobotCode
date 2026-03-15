// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoMaker;
import frc.robot.controls.DriverControls;
import frc.robot.controls.OperatorControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;

public class RobotContainer {
  private final Telemetry logger = new Telemetry();

  private final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
  private final Superstructure superstructure = new Superstructure(swerve);

  // private final AutoMaker m_auto = new AutoMaker(swerve);
  private final AutoFactory autoFactory;

  public RobotContainer() {
    autoFactory = new AutoFactory(
        () -> swerve.getState().Pose,
        (pose) -> swerve.resetPose(pose),
        swerve::followTrajectory,
        true,
        swerve);

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
    // AutoFactory factory = m_auto.get();
    AutoRoutine routine = autoFactory.newRoutine("Routine");

    AutoTrajectory traj = routine.trajectory("TestPath");
    System.out.println("=================== Trajectory loaded: " + traj.toString());

    Logger.recordOutput("Auto/InitialPose", traj.getInitialPose().get());
    Logger.recordOutput("Auto/FinalPose", traj.getFinalPose().get());

    routine.active().onTrue(
        Commands.sequence(
            // traj.resetOdometry(),
            traj.cmd()));

    traj.atTime("Event").onTrue(Commands.print("=================== Event Fired ==================="));
    traj.done().onTrue(Commands.print("=================== Event Done ==================="));

    return routine.cmd();
  }

  public Command getHoodHomeCommand() {
    return superstructure.hoodHomeSequence();
  }

  public CommandSwerveDrivetrain getSwerveSystem() {
    return swerve;
  }
}
