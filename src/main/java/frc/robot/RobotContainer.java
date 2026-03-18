// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.auto.AutoChooser;
import frc.robot.auto.ChoreoTraj;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.controls.DriverControls;
import frc.robot.controls.OperatorControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;

public class RobotContainer {
  private final Telemetry logger = new Telemetry();

  private final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
  private final Superstructure superstructure = new Superstructure(swerve);

  private final AutoFactory autoFactory;
  private final AutoChooser autoChooser;

  public RobotContainer() {
    autoFactory = new AutoFactory(
        () -> swerve.getState().Pose,
        (pose) -> swerve.resetPose(pose),
        swerve::followTrajectory,
        true,
        swerve);

    configureBindings();
    buildNamedAutoCommands();

    // Build the auto chooser with all Choreo trajectories + DO NOTHING
    autoChooser = new AutoChooser("DO NOTHING");
    for (String name : ChoreoTraj.ALL_TRAJECTORIES.keySet()) {
      autoChooser.addRoutine(name, () -> buildTrajectoryRoutine(name));
    }

    // Publish to SmartDashboard so it appears on the driver dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Schedule the selected auto when autonomous mode starts
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  /**
   * Builds an AutoRoutine that resets odometry and follows the named trajectory.
   */
  private AutoRoutine buildTrajectoryRoutine(String trajectoryName) {
    AutoRoutine routine = autoFactory.newRoutine(trajectoryName);
    AutoTrajectory traj = routine.trajectory(trajectoryName);

    traj = addNamedEvents(traj);

    routine.active().onTrue(
        Commands.sequence(
            // DO NOT CALL THIS
            // traj.resetOdometry(),
            traj.cmd()));

    // // When the trajectory is done, score
    // traj.done().onTrue(superstructure.feedAllCommand());

    return routine;
  }

  private AutoTrajectory addNamedEvents(AutoTrajectory traj) {
    // Intake
    traj.atTime("intakeCommand").onTrue(superstructure.intakeCommand());
    traj.atTime("intakeStopCommand").onTrue(superstructure.intakeStopCommand());

    // Feeding
    traj.atTime("feedAllCommand").onTrue(superstructure.feedAllCommand());

    // Shooting
    traj.atTime("ShootOnTheMoveCommand")
        .toggleOnTrue(new ShootOnTheMoveCommand(swerve, superstructure, () -> superstructure.getAimPoint()));

    // // When the trajectory is done, start the next trajectory
    // pickupTraj.done().onTrue(scoreTraj.cmd());

    // // While the trajectory is active, prepare the scoring subsystem
    // scoreTraj.active().whileTrue(scoringSubsystem.getReady());

    // // When the trajectory is done, score
    // scoreTraj.done().onTrue(scoringSubsystem.score());

    return traj;
  }

  private void configureBindings() {
    DriverControls.configure(Constants.ControllerConstants.kDriverControllerPort, swerve, superstructure, logger);
    OperatorControls.configure(Constants.ControllerConstants.kOperatorControllerPort, swerve, superstructure);
  }

  private void buildNamedAutoCommands() {
    // Add any named auto commands here via autoFactory.bind()
  }

  public Command getHoodHomeCommand() {
    return superstructure.hoodHomeSequence();
  }

  public CommandSwerveDrivetrain getSwerveSystem() {
    return swerve;
  }

  /**
   * Resets the drivetrain pose to the starting pose of the currently selected
   * auto.
   * Intended for use in simulation only.
   */
  public void resetPoseToAutoStart() {
    String selected = autoChooser.getSelectedName();
    ChoreoTraj traj = ChoreoTraj.ALL_TRAJECTORIES.get(selected);
    if (traj != null) {
      Pose2d startPose = traj.initialPoseBlue();
      if (ChoreoAllianceFlipUtil.shouldFlip()) {
        startPose = ChoreoAllianceFlipUtil.flip(startPose);
      }
      swerve.resetPose(startPose);
      System.out.println("[Sim] Reset pose to start of auto: " + selected);
    }
  }
}
