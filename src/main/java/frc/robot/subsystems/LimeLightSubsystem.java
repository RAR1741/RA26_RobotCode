package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.VisionConstants;
import frc.robot.FieldConstants;
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

  private CommandSwerveDrivetrain drivetrain;

  private boolean IS_LIMELIGHT_ENABLED = true;

  private static final int MAX_INIT_RETRIES = 5;
  private static final double RETRY_INTERVAL_SECONDS = 5.0;
  private String limelightName;
  private Pose3d cameraOffset;
  private int initRetryCount = 0;
  private boolean initialized = false;
  private Timer retryTimer = new Timer();

  private boolean hasEnabled = false;

  public LimeLightSubsystem(CommandSwerveDrivetrain drivetrain, String name, Pose3d cameraOffset) {
    this.drivetrain = drivetrain;
    this.limelightName = name;
    this.cameraOffset = cameraOffset;

    if (IS_LIMELIGHT_ENABLED) {
      tryInitializeLimelight();
    }
  }

  private void tryInitializeLimelight() {
    // Quick non-blocking check before constructing Limelight to avoid the library's
    // built-in warning
    boolean isAvailable = NetworkTableInstance.getDefault()
        .getTable(limelightName).containsKey("getpipe");

    if (!isAvailable && !RobotBase.isSimulation()) {
      initRetryCount++;
      if (initRetryCount <= MAX_INIT_RETRIES) {
        System.out.println("Limelight " + limelightName + " not available, retry "
            + initRetryCount + "/" + MAX_INIT_RETRIES + " in " + RETRY_INTERVAL_SECONDS + "s");
        retryTimer.restart();
      } else {
        System.err.println("Limelight " + limelightName + " failed to initialize after "
            + MAX_INIT_RETRIES + " retries. Giving up.");
      }
      return;
    }

    try {
      limelight = new Limelight(limelightName);
      configureLimelight();
      initialized = true;
      System.out.println("Limelight " + limelightName + " initialized successfully"
          + (initRetryCount > 0 ? " (after " + initRetryCount + " retries)" : ""));
    } catch (Exception e) {
      limelight = null;
      poseEstimator = null;
      initRetryCount++;
      if (initRetryCount <= MAX_INIT_RETRIES) {
        System.out.println("Limelight " + limelightName + " initialization error: " + e.getMessage()
            + ", retry " + initRetryCount + "/" + MAX_INIT_RETRIES + " in " + RETRY_INTERVAL_SECONDS + "s");
        retryTimer.restart();
      } else {
        System.err.println("Limelight " + limelightName + " failed to initialize after "
            + MAX_INIT_RETRIES + " retries. Giving up.");
      }
    }
  }

  private void configureLimelight() {
    limelight.getSettings()
        .withLimelightLEDMode(LEDMode.PipelineControl)
        .withCameraOffset(cameraOffset)
        // new Rotation3d(0, 21, 180)))
        .withImuMode(ImuMode.InternalImuMT1Assist)
        .withImuAssistAlpha(0.01)
        .save();

    RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
      if (limelight == null)
        return;

      if (DriverStation.getMatchType() != MatchType.None)
        return;

      System.out.println("Setting LL (" + limelight.limelightName + ") IMU Assist Alpha to 0.01");

      limelight.getSettings()
          .withImuMode(ImuMode.InternalImuMT1Assist)
          .withImuAssistAlpha(0.01)
          .save();
    }).ignoringDisable(true));

    Command onEnable = Commands.runOnce(() -> {
      hasEnabled = true;

      if (limelight == null)
        return;
      System.out.println("Setting LL (" + limelight.limelightName + ") IMU Assist Alpha to 0.000001");

      limelight.getSettings()
          .withImuMode(ImuMode.ExternalImu)
          .withImuAssistAlpha(0.000001)
          .save();
    });

    RobotModeTriggers.teleop().onTrue(onEnable);
    RobotModeTriggers.autonomous().onTrue(onEnable);
    RobotModeTriggers.test().onTrue(onEnable);

    poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
  }

  @Override
  public void periodic() {
    if (IS_LIMELIGHT_ENABLED) {
      // If not yet initialized, check if it's time to retry
      if (!initialized) {
        if (initRetryCount <= MAX_INIT_RETRIES && retryTimer.hasElapsed(RETRY_INTERVAL_SECONDS)) {
          tryInitializeLimelight();
        }
        return;
      }

      // TODO: this might be needed?
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
        Logger.recordOutput("Limelight/" + limelight.limelightName + "/Megatag2Count", poseEstimate.tagCount);
        Logger.recordOutput("Limelight/" + limelight.limelightName + "/LLPose", poseEstimate.pose);

        if (checkPose(poseEstimate)) {
          updatePoseWithStdDev(poseEstimate);
        }
      });
    }
  }

  private void updatePoseWithStdDev(PoseEstimate estimate) {
    Logger.recordOutput("FieldSimulation/drivetrainAngularVelocity",
        drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble());

    double avgDistance = estimate.avgTagDist;
    double xyStdDev = VisionConstants.xyStdDevCoefficient
        * Math.pow(avgDistance, 2.0)
        / estimate.tagCount
        * VisionConstants.stdDevFactor
        * (DriverStation.isAutonomous() ? VisionConstants.autoStdDevScale : 1.0);

    // If enabled, don't trust. If disbled, trust but with a large std dev
    double thetaStdDev = DriverStation.isEnabled()
        ? Double.POSITIVE_INFINITY
        : VisionConstants.thetaStdDevCoefficient
            * Math.pow(avgDistance, 2.0)
            / estimate.tagCount
            * VisionConstants.stdDevFactor;

    // Add it to the pose estimator.
    drivetrain.addVisionMeasurement(
        estimate.pose.toPose2d(),
        estimate.timestampSeconds,
        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
  }

  private boolean checkPose(PoseEstimate estimate) {
    if (estimate == null) {
      return false;
    }

    if (estimate.tagCount <= VisionConstants.minTagCount) {
      return false;
    }

    if (estimate.pose.equals(new Pose3d())) {
      return false;
    }

    if (estimate.pose.getX() <= 0 || estimate.pose.getX() > FieldConstants.fieldLength) {
      return false;
    }

    if (estimate.pose.getY() <= 0 || estimate.pose.getY() > FieldConstants.fieldWidth) {
      return false;
    }

    if (Math.abs(drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) > 75.0) {
      return false;
    }

    return true;
  }

  @Override
  public void simulationPeriodic() {
  }

  public String[] getConnections() {
    String[] out = {null};

    try {
      poseEstimator.getPoseEstimate();
    } catch (Exception e) {
      out[0] = e.getClass().toString().substring(16);
    }

    return out;
  }
}
