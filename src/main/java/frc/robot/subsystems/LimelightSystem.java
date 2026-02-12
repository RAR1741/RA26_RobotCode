package frc.robot.subsystems;

import java.util.Optional;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import frc.robot.Telemetry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
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
    private String lastRejectReason = "";

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
         .save();

        visionEstimate = limelight.createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate();
    }

    @Override
    public void periodic(){
        // if the pose is there
        visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
            this.allowed = this.exceptions(poseEstimate);

            // Publish numeric telemetry to SmartDashboard via Telemetry helper
            Telemetry.logNumber("Limelight/tagCount", (double) poseEstimate.tagCount);
            Telemetry.logNumber("Limelight/poseX_m", poseEstimate.pose.getX());
            Telemetry.logNumber("Limelight/poseY_m", poseEstimate.pose.getY());
            Telemetry.logNumber("Limelight/poseZ_m", poseEstimate.pose.getZ());
            Telemetry.logNumber("Limelight/poseTimestamp", poseEstimate.timestampSeconds);
            Telemetry.logNumber("Robot/vx_mps", swerveDrive.getRobotVelocity().vxMetersPerSecond);
            Telemetry.logNumber("Robot/vy_mps", swerveDrive.getRobotVelocity().vyMetersPerSecond);

            // Extra single telemetry: latency (seconds)
            double latency = Timer.getFPGATimestamp() - poseEstimate.timestampSeconds;
            Telemetry.logNumber("Limelight/latency_s", latency);

            Telemetry.logString("Limelight/accepted", this.allowed ? "true" : "false");

            if (this.allowed) {
                swerveDrive.addVisionMeasurement(
                    poseEstimate.pose.toPose2d(),
                    poseEstimate.timestampSeconds);
            } else {
                // always write reject reason to dashboard (human readable)
                Telemetry.logString("Limelight/rejectReason", lastRejectReason);
            }
        });
    }

    public void onEnabled(){
        limelight.getSettings()
            // .withImuMode(ImuMode.InternalImuMT1Assist)
            .withImuAssistAlpha(0.01)
            .save();
    }
        
    public boolean exceptions(PoseEstimate foo) {
        if (foo.tagCount <= 0) { 
            lastRejectReason = "no tags";
            return false; }
        if (foo.pose.getX() <= 0 || foo.pose.getX() > Constants.FieldConstants.k_length) { 
            lastRejectReason = "x out of bounds: " + foo.pose.getX();
            return false; }
        if (foo.pose.getY() <= 0 || foo.pose.getY() > Constants.FieldConstants.k_width) { 
            lastRejectReason = "y out of bounds: " + foo.pose.getY();
            return false;}
        if (Math.abs(swerveDrive.getRobotVelocity().vxMetersPerSecond) > 720) { 
            lastRejectReason = "vx too high: " + swerveDrive.getRobotVelocity().vxMetersPerSecond;
            return false; }
        if (Math.abs(swerveDrive.getRobotVelocity().vyMetersPerSecond) > 720) { 
            lastRejectReason = "vy too high: " + swerveDrive.getRobotVelocity().vyMetersPerSecond;
            return false; }

        // TODO make sure the april tag area is legibi
        lastRejectReason = "";

        return true;

    }
}