package frc.robot.auto.tasks;

import frc.robot.subsystems.SwerveSystem;
import limelight.networktables.PoseEstimate;
import frc.robot.subsystems.LimelightSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class Rotate extends Tasks {
    private SwerveSystem m_swerve;
    private LimelightSystem m_limelight;
    private double m_rot;
    private Pose2d m_prevPose;
    private Pose2d m_curPose;

    public Rotate(double rotation){
        m_rot = rotation;
        m_swerve = new SwerveSystem();
        m_limelight = new LimelightSystem(m_swerve.getSwerveDrive());
    }

    @Override
    public void prepare() {
        m_limelight.getMeasurements().ifPresent((PoseEstimate pose) -> {
            m_prevPose = pose.pose.toPose2d();
        });
        m_prepared = true;
    }

    @Override
    public void update() {
        logIsRunning(m_isFinished);
        m_swerve.driveCommand(0, 0, Units.degreesToRadians(m_rot));

    }

    @Override
    public boolean isFinished() {
        m_limelight.getMeasurements().ifPresent((PoseEstimate pose) -> {
            m_curPose = pose.pose.toPose2d();
        });

        return m_prevPose.getRotation().getDegrees() - m_curPose.getRotation().getDegrees() == m_rot;
    }

    @Override
    public void done(){
        logIsRunning(false);
    }
    
}
