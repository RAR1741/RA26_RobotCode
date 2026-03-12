package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.RPM;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.turret.AngleChangerSystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;

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
    public static final double k_maxAcceleration = 999999999999999.0;

    public static final double k_maxDriverSpeed = 1.0; // Meters per second
    public static final double k_maxDriverBoostSpeed = 4.5;

    public static final double k_boostScaler = k_maxDriverBoostSpeed / k_maxDriverSpeed;
  }

  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    // Joystick Deadband
    public static final double k_DEADBAND = 0.1;
  }

  public static class TurretConstants { // feet (NOT INCHES), seconds, degrees, pounds (mass), pound*ft/s^2 (force)
    public static final double k_gravity = 32.174;
    public static final double k_turretRelativeX = 9999999999.0;
    public static final double k_turretRelativeY = 9999999999.0;
    public static final double k_turretDistToRobotCenter = Math.hypot(k_turretRelativeX, k_turretRelativeY);
    public static final double k_turretHeight = 21.0 / 12.0;
    public static final double k_turretMaxHorizontalRadius = 9999999.0;
    public static final double k_extraTimeToPassSensor = 1.5; // FIRST gave a shot distribution for this but it has a wide spread, so we should decide either more or less
    public static final double k_targetLaunchVY = 20.0;
    public static final double k_maxRPM = 99999999999999.0;
    public static final double k_maxLaunchVelocity = 5000000000000000.0; // whatever is 67% motor power   --_(ツ)_/¯
    public static final double k_minLaunchAngle = 45.0;                  //                               ¯\_(ツ)_--
    public static final double k_maxLaunchAngle = 80.0;
    public static final double k_maxHubLaunchDistance = Math.hypot(FieldConstants.k_allianceZoneDepth, FieldConstants.k_hubY);
    public static final double k_maxZoneLaunchDistance = Math.hypot(FieldConstants.k_fieldLength - FieldConstants.k_allianceZoneDepth, FieldConstants.k_hubY);
    public static final double k_minZoneLaunchVelocity = 5.0; // idk if this even matters but it could be checked
    public static final double k_minAngleUnderTrench = 70.0;
    public static final double k_maxTrenchPitchMotorPos = AngleChangerSystem.launchPitchToMotorPos(k_minAngleUnderTrench);
    public static final double k_maxPitchMotorSpeed = 9999999999.0;
    public static final double k_RPMTolerance = 2000000000000000.0;
    public static final double k_launchVelocityTolerance = 20000000000.0;
    public static final double k_launchDirectionTolerance = 2000000000000.0;
    public static final double k_launchAngleTolerance = 200000000000.0;
    public static final double k_wheelRadius = 2.0 / 12.0;
    public static final double k_shootingVelocityTransferEfficiency = 0.99999999999999999999;
    public static final int k_yawMotorId = 9999;
    public static final int k_pitchMotorId = 9999;
    public static final int k_flywheelMotorIdA = 9999;
    public static final int k_flywheelMotorIdB = 9999;
    public static final int k_turretMotorId = 50; // which motor is this??
    public static final int k_timeSolvingIterations = 10;
  }

  public static class FieldConstants { // feet (NOT INCHES)
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
    public static final double k_blueHubX = k_allianceZoneDepth + k_hubZoneDepth / 2.0;
    public static final double k_redHubX = k_fieldLength - (k_allianceZoneDepth + k_hubZoneDepth / 2.0);
    public static final double k_hubY = k_fieldCenterY;
    public static final double k_fuelRadius = 5.91 / 2.0 / 12.0;
    public static final double k_fuelMass = 0.474; // average
    public static final double k_ceilingHeight = 15.0; // estimated
    public static final double k_hubNetOverhang = 0.5; // need to measure
  } // check and adjust constants
    
  public static class IntakeConstants { // feet (NOT INCHES), seconds, degrees, pounds (mass), pound*ft/s^2 (force)
    public static final int k_pivotMotorId = 30;
    public static final int k_rollerMotorId = 31;
    public static final Angle k_IntakeStow = Degrees.of(0);
    public static final Angle k_IntakeFeed = Degrees.of(59);
    public static final Angle k_IntakeHold = Degrees.of(115);
    public static final Angle k_IntakeDeployed = Degrees.of(148);
  }

  public static class HopperConstants {
    public static final int kHopperMotorId = 40;
  }

  public static class KickerConstants {
    public static final int kKickerMotorId = 41;
  }

  public static class ShooterConstants {
    public static final int k_leaderMotorId = 52;
    public static final int k_followerMotorId = 53;
  }

  // public static class SuperstructureConstants {
  //   private final static AngularVelocity targetShooterSpeed = RPM.of(0);
  //   private final static Angle targetTurretAngle = Degrees.of(0);
  // }

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
}
