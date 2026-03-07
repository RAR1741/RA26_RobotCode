package frc.robot.auto;

import choreo.auto.AutoFactory;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CustomAutoFactory {
    private final AutoFactory autofactory;

    public CustomAutoFactory(CommandSwerveDrivetrain swerve){
        autofactory = new AutoFactory(() -> swerve.getState().Pose, (pose) -> swerve.resetPose(pose), swerve::followTrajectory, false, swerve);
    }

    public AutoFactory getFactory() {
        return autofactory;
    }
}
