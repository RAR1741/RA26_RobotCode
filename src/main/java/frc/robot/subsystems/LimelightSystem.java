package frc.robot.subsystems;

import java.util.Optional;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightSettings.ImuMode;
import swervelib.SwerveDrive;
import frc.robot.Constants;

//https://github.com/Yet-Another-Software-Suite/YALL

/*
--coordinate system--
https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
--field dimensions (in inches)--
https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
*/

public class LimelightSystem extends SubsystemBase {

    private Limelight limelight;
    private SwerveDrive swerveDrive; 
    private Optional<PoseEstimate> visionEstimate;
    private boolean allowed;

    public LimelightSystem(SwerveDrive swerve){
        limelight = new Limelight("limelight");
        swerveDrive = swerve;

        limelight.getSettings()
        .withLimelightLEDMode(LEDMode.PipelineControl)
        .withCameraOffset(Pose3d.kZero)
        .withRobotOrientation(
            new Orientation3d(swerveDrive.getGyro().getRotation3d(),
                new AngularVelocity3d(
                    DegreesPerSecond.of(0),
                    DegreesPerSecond.of(0),
                    DegreesPerSecond.of(0))))
        .withImuMode(ImuMode.InternalImuExternalAssist)
        .save();

        visionEstimate = limelight.createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate();
    }

    @Override
    public void periodic() {
        //if the pose is there
        visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
            this.allowed = this.rejectUpdate(poseEstimate);
            if (!this.allowed) {
                swerveDrive.addVisionMeasurement(
                    poseEstimate.pose.toPose2d(),
                    poseEstimate.timestampSeconds);
            }
        });
    }

    public void onEnabled() {
        limelight.getSettings()
            // .withImuMode(ImuMode.InternalImuMT1Assist)
            .withImuAssistAlpha(0.01)
            .save();
    }
        
    public boolean rejectUpdate(PoseEstimate foo) {
        /*
        returns true if Pose didn't pass tests
        returns false if Pose passed tests
        */
        if (foo.tagCount <= 0) { return true; }
        if (foo.getAvgTagAmbiguity() > 0.7 ) { return true; }
        if (foo.pose.getX() <= 0 || foo.pose.getX() > Constants.FieldConstants.k_length) { return true; }
        if (foo.pose.getY() <= 0 || foo.pose.getY() > Constants.FieldConstants.k_width) { return true;}
        if (Math.abs(swerveDrive.getRobotVelocity().omegaRadiansPerSecond) > Math.PI * 2) { return true; }
        if (Math.abs(swerveDrive.getRobotVelocity().omegaRadiansPerSecond) > Math.PI * 2) { return true; }

        // TODO make sure the april tag area is legible

        return true;
        
    }
}