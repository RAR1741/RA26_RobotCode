package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.VisionConstants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

public class LimeLightSubsystem extends SubsystemBase {

  private Limelight limelight;
  private LimelightPoseEstimator poseEstimator;

  private final CommandSwerveDrivetrain drivetrain;

  private final boolean IS_LIMELIGHT_ENABLED = true;

  private double distanceToHub = 0.0;

  public double getDistanceToHub() {
    if (IS_LIMELIGHT_ENABLED) {
      return distanceToHub;
    }
    return 0.0;
  }

  public LimeLightSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    if (IS_LIMELIGHT_ENABLED) {
      limelight = new Limelight("limelight-front");

      limelight.getSettings()
          .withLimelightLEDMode(LEDMode.PipelineControl)
          .withCameraOffset(new Pose3d(
              // TODO: THIS
              // -2.75 in RIGHT
              // -12.75 in FORWARD
              // 20.375 in UP
              Inches.of(-12.75).in(Meters), // -0.32385
              Inches.of(-2.75).in(Meters), // -0.06985
              Inches.of(20.375).in(Meters), // 0.517525
              new Rotation3d(0, Degrees.of(20).in(Radians), Degrees.of(180).in(Radians))))
          // new Rotation3d(0, 21, 180)))
          .withImuMode(ImuMode.InternalImuMT1Assist)
          .withImuAssistAlpha(0.01)
          .save();

      RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
        System.out.println("Setting LL IMU Assist Alpha to 0.01");

        limelight.getSettings()
            .withImuMode(ImuMode.InternalImuMT1Assist)
            .withImuAssistAlpha(0.01)
            .save();
      }).ignoringDisable(true));

      Command onEnable = Commands.runOnce(() -> {
        System.out.println("Setting LL IMU Assist Alpha to 0.000001");

        limelight.getSettings()
            .withImuMode(ImuMode.ExternalImu)
            .withImuAssistAlpha(0.000001)
            .save();
      });

      RobotModeTriggers.teleop().onTrue(onEnable);
      RobotModeTriggers.autonomous().onTrue(onEnable);
      RobotModeTriggers.test().onTrue(onEnable);

      // Required for megatag2 in periodic() function before fetching pose.
      // limelight.getSettings()
      // .withRobotOrientation(
      // new Orientation3d(drivetrain.getRotation3d(),
      // new AngularVelocity3d(
      // DegreesPerSecond.of(0),
      // DegreesPerSecond.of(0),
      // DegreesPerSecond.of(0))))
      // .save();

      poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
    }
  }

  @Override
  public void periodic() {
    if (IS_LIMELIGHT_ENABLED) {
      if (DriverStation.isEnabled()) {
        AngularVelocity3d angularVel = new AngularVelocity3d(
            DegreesPerSecond.of(0),
            DegreesPerSecond.of(0),
            RadiansPerSecond.of(drivetrain.getState().Speeds.omegaRadiansPerSecond));

        Rotation3d robotRotation = new Rotation3d(
            0,
            0,
            drivetrain.getState().Pose.getRotation().getRadians());

        // Required for megatag2 in periodic() function before fetching pose.
        limelight.getSettings()
            .withImuMode(ImuMode.ExternalImu)
            .withRobotOrientation(new Orientation3d(robotRotation, angularVel))
            .save();
      }

      // Get MegaTag2 pose
      Optional<PoseEstimate> visionEstimate = poseEstimator.getPoseEstimate();

      Logger.recordOutput("FieldSimulation/heading/pigeon", drivetrain.getState().RawHeading.getDegrees());
      Logger.recordOutput("FieldSimulation/heading/pose", drivetrain.getState().Pose.getRotation().getDegrees());

      // If the pose is present
      visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
        Logger.recordOutput("Limelight/Megatag2Count", poseEstimate.tagCount);
        Logger.recordOutput("FieldSimulation/LLPose", poseEstimate.pose);

        if (poseEstimate.tagCount > 0) {
          Pose3d redHub = new Pose3d(
              Meter.of(11.902),
              Meter.of(4.031),
              Meter.of(0.0),
              new Rotation3d(0, 0, 0));

          distanceToHub = poseEstimate.pose.toPose2d().minus(redHub.toPose2d()).getTranslation().getNorm();

          Logger.recordOutput("FieldSimulation/hubDiff", distanceToHub);

          double avgDistance = poseEstimate.avgTagDist;
          double xyStdDev = VisionConstants.xyStdDevCoefficient
              * Math.pow(avgDistance, 2.0)
              / poseEstimate.tagCount
              * VisionConstants.stdDevFactor
              * (DriverStation.isAutonomous() ? VisionConstants.autoStdDevScale : 1.0);

          double thetaStdDev = !DriverStation.isEnabled()
              ? VisionConstants.thetaStdDevCoefficient
                  * Math.pow(avgDistance, 2.0)
                  / poseEstimate.tagCount
                  * VisionConstants.stdDevFactor
                  * (DriverStation.isAutonomous() ? VisionConstants.autoStdDevScale : 1.0)
              : Double.POSITIVE_INFINITY;

          // Add it to the pose estimator.
          drivetrain.addVisionMeasurement(
              poseEstimate.pose.toPose2d(),
              poseEstimate.timestampSeconds,
              VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));

          // TODO: possibly add stddevs here
          // TODO: Instead of providing the limelight's pose as is, replace the rotation
          // component with the current pose rotation so the process doesn't take it into
          // account?
        }
      });
    }
  }

  @Override
  public void simulationPeriodic() {
  }
}
