package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;

public class Constants {
  public static final CANBus ctreCANBus = new CANBus("Drivetrain");

  public static class SwerveDriveConstants {
    private final static Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final static Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final static Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final static Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    public static final SwerveDriveKinematics k_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation); // get this in when
                                                                                             // we know bot dims

    public static final double k_maxSpeed = Units.feetToMeters(14.5);

    public static final double k_maxDriverSpeed = 1.0; // Meters per second
    public static final double k_maxDriverBoostSpeed = 4.5;

    public static final double k_boostScaler = k_maxDriverBoostSpeed / k_maxDriverSpeed;

    public static final double k_maxAcceleration = 999999999.0; // TODO
  }

  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double k_slowModeMin = 0.25; // Scales down max speed when slow mode is active
    public static final double k_standardSpeed = 0.5; // Normal max speed
    public static final double k_boostModeScaler = 1.0; // Scales up max speed when boost mode is active

    // Joystick Deadband
    public static final double k_DEADBAND = 0.1;
  }

  public static class IntakeConstants {
    public static final int k_pivotPrimaryMotorId = 30;
    public static final int k_pivotSecondaryMotorId = 31;
    public static final int k_rollerMotorId = 32;

    public static final Angle k_IntakeStow = Degrees.of(0);
    public static final Angle k_IntakeFeed = Degrees.of(80);
    // public static final Angle k_IntakeHold = Degrees.of(115);
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

    public static final double MAX_ONE_DIR_FOV = 110; // degrees

    public static final double M12_OFFSET = 0.85376;
    public static final double M13_OFFSET = 0.597265;

    public static final Translation3d turretTranslation = new Translation3d(
        Inches.of(-6.25),
        Inches.of(-6.75),
        Inches.of(20.0));

    // feet (NOT INCHES), seconds, degrees, pounds (mass)

    public static final double k_gravity = 32.174;
    public static final double k_turretDistToRobotCenter = turretTranslation.toTranslation2d().getNorm();
    public static final double k_turretHeight = 20.0 / 12.0; // either 20 or 21 inches
    public static final double k_turretMaxHorizontalRadius = 9.0 / 12.0;
    public static final double k_extraTimeToPassSensor = 1.5; // FIRST gave a shot distribution for this but it has a wide spread, so we should decide either more or less
    public static final double k_targetLaunchVY = 20.0;
    public static final double k_maxRPM = 6000.0; 
    public static final double k_maxLaunchVelocity = 50.0; //TODO: TEST VALUES FOR THIS
    public static final double k_minLaunchAngle = 45.0; 
    public static final double k_maxLaunchAngle = 80.0;
    public static final double k_maxHubLaunchDistance = Math.hypot(FieldConstants.k_allianceZoneDepth, FieldConstants.k_hubY);
    public static final double k_maxZoneLaunchDistance = Math.hypot(FieldConstants.k_fieldLength - FieldConstants.k_allianceZoneDepth, FieldConstants.k_hubY);
    public static final double k_minZoneLaunchVelocity = 5.0;
    public static final double k_minAngleUnderTrench = 70.5;
    public static final double k_maxAngleChangingRate = 9999999999.0; // TODO
    public static final double k_wheelRadius = 2.0 / 12.0;
    public static final double k_shootingVelocityTransferEfficiency = 1.0; // forget about it
    public static final int k_timeSolvingIterations = 0;
}

public static class FieldConstants { // feet (NOT INCHES)
    public final static double k_width = Units.feetToMeters(26.0) + Units.inchesToMeters(5); // don't know if these two are still needed
    public final static double k_length = Units.feetToMeters(57.0) + Units.inchesToMeters(6.0 + (7.0 / 8.0));
    public static final double k_fieldWidth = 26.0 + 5.7 / 12.0;
    public static final double k_fieldLength = 57.0 + (6.0 + (7.0 / 8.0)) / 12.0;
    public static final double k_fieldCenterX = k_fieldLength / 2.0;
    public static final double k_fieldCenterY = k_fieldWidth / 2.0;
    public static final double k_trenchWidth = 50.34 / 12.0;
    public static final double k_trenchBumpBarrierWidth = (65.65 - 50.34) / 12.0;
    public static final double k_bumpWidth = 73.0 / 12.0;
    public static final double k_hubBodyWidth = 47.0 / 12.0;
    public static final double k_hubBodyDepth = 47.0 / 12.0;
    public static final double k_allianceZoneDepth = 158.6 / 12.0;
    public static final double k_hubZoneDepth = k_hubBodyDepth;
    public static final double k_trenchBarDepth = 3.5 / 12.0;
    public static final double k_neutralZoneDepth = k_fieldLength - 2.0 * (k_allianceZoneDepth + k_hubZoneDepth);
    public static final double k_hubWidth = 41.7 / 12.0;
    public static final double k_hubRadius = k_hubWidth / Math.sqrt(3.0);
    public static final double k_hubHeight = 6.0;
    public static final double k_hubNetOverhang = 6.0 / 12.0;
    public static final double k_blueHubX = k_allianceZoneDepth + k_hubZoneDepth / 2.0;
    public static final double k_redHubX = k_fieldLength - (k_allianceZoneDepth + k_hubZoneDepth / 2.0);
    public static final double k_hubY = k_fieldCenterY;
    public static final double k_fuelRadius = 5.91 / 2.0 / 12.0;
    public static final double k_fuelMass = 0.474; // average
    public static final double k_ceilingHeight = 15.0; // estimated
} // check and adjust constants

  public static class HoodConstants {
    public static final int k_hoodMotorId = 51;
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
    public static final double autoStdDevScale = 0.0;
    public static final double autoTranslationMax = 100.0;
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

  public static enum AimPoints {
    RED_HUB(new Translation3d(11.938, 4.034536, 1.5748)),
    RED_OUTPOST(new Translation3d(16.0, 7.0, 0)),
    RED_FAR_SIDE(new Translation3d(16.0, 1.0, 0)),

    BLUE_HUB(new Translation3d(4.5974, 4.034536, 1.5748)),
    BLUE_OUTPOST(new Translation3d(0.0, 1.0, 0)),
    BLUE_FAR_SIDE(new Translation3d(0.0, 7.0, 0));

    public final Translation3d value;

    private AimPoints(Translation3d value) {
      this.value = value;
    }

    public static final Translation3d getAllianceHubPosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_HUB.value : BLUE_HUB.value;
    }

    public static final Translation3d getAllianceOutpostPosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_OUTPOST.value : BLUE_OUTPOST.value;
    }

    public static final Translation3d getAllianceFarSidePosition() {
      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? RED_FAR_SIDE.value : BLUE_FAR_SIDE.value;
    }
  }
}