package frc.robot.auto;

import choreo.auto.AutoFactory;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoFactory {
    public final AutoFactory autofactory;

    public autoFactory(CommandSwerveDrivetrain swerve){
        autofactory = new AutoFactory(() -> swerve.getState().Pose, (pose) -> swerve.resetPose(pose), swerve::followTrajectory, false, swerve);
    }
}
