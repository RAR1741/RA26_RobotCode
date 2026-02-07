package frc.robot.auto.tasks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SwerveSystem;
import limelight.networktables.PoseEstimate;
import frc.robot.subsystems.LimelightSystem;
import frc.robot.Telemetry;

public class DriveForwardTask extends Tasks{
    private SwerveSystem m_swerve;
    private LimelightSystem m_limelight;
    private double m_targetDistance;
    private double m_speed;   
    private Pose2d m_startPose;
    private Pose2d m_curPose;
    private Pose2d m_checkerPose;

    private Timer m_runningTimer = new Timer();

    public DriveForwardTask(double distance, double speed){
        m_swerve = new SwerveSystem();
        m_limelight = new LimelightSystem(m_swerve.getSwerveDrive());
        m_targetDistance = distance;
        m_speed = speed;
    }

    @Override
    public void prepare() {
        m_runningTimer.reset();
        m_runningTimer.start();

        m_limelight.getMeasurements().ifPresent((PoseEstimate pose) -> {
            m_startPose = pose.pose.toPose2d();    
        });

        m_prepared = true;
    }

    @Override
    public void update() {
        logIsRunning(m_isFinished);
        
        m_limelight.getMeasurements().ifPresent((PoseEstimate pose) -> {
            m_curPose = pose.pose.toPose2d();    
        });

        // Vx = V * cos(theta)
        double xSpeed = m_speed * Math.cos(m_curPose.getRotation().getRadians());
        // Vy = V * sin(theta)
        double ySpeed = m_speed * Math.sin(m_curPose.getRotation().getRadians());

        m_swerve.driveCommand(xSpeed, ySpeed, 0);
    }

    @Override
    public boolean isFinished() {
        m_limelight.getMeasurements().ifPresent((PoseEstimate pose) -> {
            m_checkerPose = pose.pose.toPose2d();
        });

        Pose2d relativePose = m_startPose.relativeTo(m_checkerPose);
        return Math.hypot(relativePose.getX(), relativePose.getY()) >= m_targetDistance;
    }

    @Override
    public void done(){
        logIsRunning(false);

        Telemetry.logString("DriveForwardTask", "Auto driving done");
        m_swerve.driveCommand(0.0, 0.0, 0.0);
    }
}
