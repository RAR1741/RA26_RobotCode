package frc.robot.auto;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoMaker {
  private final AutoFactory autoFactory;

  public AutoMaker(CommandSwerveDrivetrain swerve) {
    autoFactory = new AutoFactory(
        () -> swerve.getState().Pose,
        (pose) -> swerve.resetPose(pose),
        swerve::followTrajectory,
        true,
        swerve);
  }

  public Command useTrajectory(String TrajectoryName) {
    return autoFactory.trajectoryCmd(TrajectoryName);
  }

  public void bind(String name, Command command) {
    autoFactory.bind(name, command);
  }

  public AutoFactory get() {
    return autoFactory;
  }
}
