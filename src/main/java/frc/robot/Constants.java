package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

public class Constants {
  public static final CANBus ctreCANBus = new CANBus("Drivetrain");

  public static class LimelightConstants {
    public static final boolean IS_LIMELIGHT_ENABLED = true;

    public static final String upName = "limelight-up";
    public static final Pose3d upCameraOffset = new Pose3d(
        Inches.of(-12.75).in(Meters), // FORWARD
        Inches.of(-2.75).in(Meters), // RIGHT
        Inches.of(20.25).in(Meters), // UP
        new Rotation3d(
            0,
            Degrees.of(20).in(Radians),
            Degrees.of(180).in(Radians)));

    public static final String downName = "limelight-down";
    public static final Pose3d downCameraOffset = new Pose3d(
        Inches.of(-12.75).in(Meters), // FORWARD
        Inches.of(-5.878).in(Meters), // RIGHT
        Inches.of(16.875).in(Meters), // UP
        new Rotation3d(
            0,
            Degrees.of(0).in(Radians),
            Degrees.of(180).in(Radians)));
  }

  public static class SwerveDriveConstants {
    private final static Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final static Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final static Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final static Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    public static final SwerveDriveKinematics k_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation); // get this in when
                                                                                             // we know bot dims
  }

  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double k_slowSpeed = 0.25; // Scales down max speed when slow mode is active
    public static final double k_standardSpeed = 0.5; // Normal max speed
    public static final double k_boostSpeed = 1.0; // Scales up max speed when boost mode is active

    public static final double k_slowRot = 0.10;
    public static final double k_standardRot = 0.75;
    public static final double k_boostRot = 0.75;

    // Joystick Deadband
    public static final double k_DEADBAND = 0.01;
  }

  public static class IntakeConstants {
    public static final int k_pivotPrimaryMotorId = 30;
    public static final int k_pivotSecondaryMotorId = 31;
    public static final int k_rollerMotorId = 32;
    public static final int k_rollerMotorSecondaryId = 33;

    public static final Angle k_IntakeStow = Degrees.of(0);
    public static final Angle k_IntakeFeed = Degrees.of(80);
    public static final Angle k_IntakeMaxWhileRoller = Degrees.of(103.5);
    // public static final Angle k_IntakeDeployed = Degrees.of(148);
    public static final Angle k_IntakeDeployed = Degrees.of(110);

    // WARNING: make sure this doesn't loop over 0.0/1.0!
    // return (getAbsAngle() - IntakeConstants.k_pivotAbsEncoderOffset + 1.0) % 1.0;
    public static final double k_pivotAbsEncoderOffset = 0.352112;

    public static final Time k_feedUpTime = Seconds.of(0.5);
    public static final Time k_feedDownTime = Seconds.of(2.0);

    /** Current threshold (amps) to detect a stall when deploying. */
    public static final double k_deployStallCurrentThreshold = 30.0;
    /**
     * How long (seconds) current must stay above the threshold to count as a stall.
     */
    public static final double k_deployStallDebounce = 0.5;
    /** How far to back off (in degrees) when a deploy stall is detected. */
    public static final Angle k_deployBackoffAngle = Degrees.of(15);
    /** How long to wait (seconds) after backing off before retrying deploy. */
    public static final Time k_deployRetryDelay = Seconds.of(1.0);
  }

  public static class HopperConstants {
    public static final int kHopperMotorId = 40;
  }

  public static class KickerConstants {
    public static final int kKickerMotorId = 41;
  }

  public static class TurretConstants {
    public static final int k_turretMotorId = 50;

    public static final Angle MAX_ONE_DIR_FOV = Degrees.of(160); // degrees

    // Mechanical lash:
    // - Old m12: (0.935568 - 0.927775) 0.007793 * 360 = 2.80548 deg
    // - New m12: (0.718129 - 0.712218) 0.005911 * 360 = 2.12796 deg
    // - Old m13: (0.593744 - 0.587949) 0.005795 * 360 = 2.0862 deg
    // - New m13: (0.649539 - 0.64658) 0.002959 * 360 = 1.06524 deg

    public static final double M12_OFFSET = 0.684788;
    public static final double M13_OFFSET = 0.623915;

    public static final double m12Frequency = 965.0; // Hz
    public static final double m13Frequency = 978.0; // Hz

    public static final Translation3d turretTranslation = new Translation3d(
        Inches.of(-6.25),
        Inches.of(-6.75),
        Inches.of(20.0));

    // feet (NOT INCHES), seconds, degrees, pounds (mass), pound*ft/s^2 (force)
    public static final double k_gravitationalAcceleration = 32.174;
    public static final double k_turretHeight = 2.0;
    public static final double k_hubHeight = 6.0;
    public static final double k_ceilingHeight = 15.0;
    public static final double k_fuelRadius = 0.246063;
    public static final double k_fuelMass = 0.474; // estimate
    public static final double k_minYHeightToHub = 0.5 + k_fuelRadius + k_hubHeight - k_turretHeight;
    public static final double k_minYVelocityToHub = Math
        .sqrt(2.0 * k_gravitationalAcceleration * k_minYHeightToHub);

  }

  public static class HoodConstants {
    public static final int k_hoodMotorId = 51;

    // TODO: Update this after testing
    public static final double boxXMultiplier = 0.25;
    public static final double boxYMultiplier = 0.075;
  }

  public static class ShooterConstants {
    public static final int k_leaderMotorId = 52;
    public static final int k_followerMotorId = 53;
  }

  public static class SuperstructureConstants {
    public static final AngularVelocity k_shooterRPMTolerance = RPM.of(100);
    public static final Angle k_turretTolerance = Degrees.of(2);
    public static final Angle k_hoodTolerance = Degrees.of(2);
  }

  public static class VisionConstants {
    public static final double xyStdDevCoefficient = 0.005;
    public static final double thetaStdDevCoefficient = 0.01;
    public static final double stdDevFactor = 0.5;
    // public static final boolean useVisionRotation = false;

    public static final int minTagCount = 1;
    public static final double maxAvgDistance = 100.0;
    public static final double autoStdDevScale = 10.0; // TODO: maybe make this bigger, for less trust
  }

  /**
   * Logging-related global constants. Toggle features here so the rest of the
   * code
   */
  public static class LoggingConstants {
    /** Enable simple Telemetry (Telemetry.log / SmartDashboard) */
    public static final boolean k_enableTelemetry = true;

    /**
     * Enable AutoLog / recording (if you add an AutoLog/recorder integration
     * elsewhere)
     */
    public static final boolean k_enableAutolog = false;

    /** Enable junction Logger recordings (org.littletonrobotics.junction.Logger) */
    public static final boolean k_enableJunctionLogger = true;

    /**
     * Directory on the robot (or host) where logs should be written if recording is
     * enabled.
     */
    public static final String k_logDirectory = "/home/lvuser/logs";

    /**
     * Base filename to use for recordings (timestamp/extension may be added by
     * recorder).
     */
    public static final String k_logFileBaseName = "robotLog";

    /**
     * Target logging period in seconds for periodic recording (0 disables periodic
     * flush).
     */
    public static final double k_logPeriodSeconds = 0.02; // 20 ms
  }

  public static class SimulationConstants {
    public final static boolean k_isInSimulation = true;
    public final static DriveTrainSimulationConfig k_config = DriveTrainSimulationConfig.Default()
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(COTS.ofMark4(
            DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
            DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
            COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
            3))
        .withBumperSize(Inches.of(28.5), Inches.of(33.5));
  }

  public static class StateConstants {
    public static Translation2d blueLeftTrench = FieldConstants.FIELD_LAYOUT.getTagPose(23).get().getTranslation()
        .toTranslation2d();
    public static Translation2d blueRightTrench = FieldConstants.FIELD_LAYOUT.getTagPose(28).get().getTranslation()
        .toTranslation2d();
    public static Translation2d redLeftTrench = FieldConstants.FIELD_LAYOUT.getTagPose(7).get().getTranslation()
        .toTranslation2d();
    public static Translation2d redRightTrench = FieldConstants.FIELD_LAYOUT.getTagPose(12).get().getTranslation()
        .toTranslation2d();

    public static double fieldLength = FieldConstants.FIELD_LAYOUT.getFieldLength();
    public static double fieldWidth = FieldConstants.FIELD_LAYOUT.getFieldWidth();
    public static double trenchWidth = Units.inchesToMeters(50);
    public static Translation2d centerField = new Translation2d(fieldLength / 2.0, fieldWidth / 2.0);

    public static ArrayList<Translation2d> trenchList = new ArrayList<Translation2d>();

    public static Translation2d[] passZoneOne = new Translation2d[4];
    public static Translation2d[] passZoneTwo = new Translation2d[4];
    public static Translation2d[] passZoneThree = new Translation2d[4];
    public static Translation2d[] passZoneFour = new Translation2d[4];
    public static Translation2d[] passZoneFive = new Translation2d[4];

    public static final Pose2d hub = new Pose2d(
        FieldConstants.Hub.innerCenterPoint.toTranslation2d(),
        new Rotation2d());

    public static final Pose2d passLeftTarget = new Pose2d(
        blueLeftTrench.getX() / 2.0,
        FieldConstants.fieldWidth * 0.75,
        new Rotation2d());

    public static final Pose2d passRightTarget = new Pose2d(
        blueRightTrench.getX() / 2.0,
        FieldConstants.fieldWidth * 0.25,
        new Rotation2d());

    public static void initConstants() {
      double centerFieldY = centerField.getY();

      double passDeadzone = 0.5;

      trenchList.add(redLeftTrench);
      trenchList.add(redRightTrench);
      trenchList.add(blueLeftTrench);
      trenchList.add(blueRightTrench);

      // Zone One (Mid right)
      passZoneOne[0] = new Translation2d(blueLeftTrench.getX(), centerFieldY);
      passZoneOne[1] = new Translation2d(redLeftTrench.getX(), centerFieldY);
      passZoneOne[2] = new Translation2d(blueLeftTrench.getX(), 0);
      passZoneOne[3] = new Translation2d(redLeftTrench.getX(), 0);

      // Zone Two (Mid left)
      for (int i = 0; i < 4; i++) {
        passZoneTwo[i] = passZoneOne[i].plus(new Translation2d(0, centerFieldY));
      }

      // Zone Three (Far right)
      passZoneThree[0] = new Translation2d(redLeftTrench.getX(), 0);
      passZoneThree[1] = new Translation2d(fieldLength, 0);
      passZoneThree[2] = new Translation2d(fieldLength, centerFieldY - passDeadzone);
      passZoneThree[3] = new Translation2d(redLeftTrench.getX(), centerFieldY - passDeadzone);

      // Zone Four (Far left)
      for (int i = 0; i < 4; i++) {
        passZoneFour[i] = passZoneThree[i].plus(new Translation2d(0, centerFieldY + passDeadzone));
      }

      // Zone Five (Far center)
      passZoneFive[0] = new Translation2d(redLeftTrench.getX(), centerFieldY - passDeadzone);
      passZoneFive[1] = new Translation2d(fieldLength, centerFieldY - passDeadzone);
      passZoneFive[2] = new Translation2d(fieldLength, centerFieldY + passDeadzone);
      passZoneFive[3] = new Translation2d(redLeftTrench.getX(), centerFieldY + passDeadzone);
    }
  }
}
