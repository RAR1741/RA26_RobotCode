package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import static edu.wpi.first.units.Units.Inches;

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
  }

  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    // Joystick Deadband
    public static final double k_DEADBAND = 0.1;
  }

  public static class IntakeConstants {
    public static final int k_pivotPrimaryMotorId = 30;
    public static final int k_pivotSecondaryMotorId = 31;
    public static final int k_rollerMotorId = 32;

    public static final Angle k_IntakeStow = Degrees.of(0);
    public static final Angle k_IntakeFeed = Degrees.of(90);
    // public static final Angle k_IntakeHold = Degrees.of(115);
    // public static final Angle k_IntakeDeployed = Degrees.of(148);
    public static final Angle k_IntakeDeployed = Degrees.of(113);

    // WARNING: make sure this doesn't loop over 0.0/1.0!
    // return (getAbsAngle() - IntakeConstants.k_pivotAbsEncoderOffset + 1.0) % 1.0;
    public static final double k_pivotAbsEncoderOffset = 0.352112;
  }

  public static class HopperConstants {
    public static final int kHopperMotorId = 40;
  }

  public static class KickerConstants {
    public static final int kKickerMotorId = 41;
  }

  public static class TurretConstants {
    public static final int k_turretMotorId = 50;

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
  }

  public static class ShooterConstants {
    public static final int k_leaderMotorId = 52;
    public static final int k_followerMotorId = 53;

    public static final AngularVelocity k_shooterRPMTolerance = RPM.of(100); // RPM tolerance for "at speed" condition
  }

  public static class SuperstructureConstants {
    private final static AngularVelocity targetShooterSpeed = RPM.of(0);
    private final static Angle targetTurretAngle = Degrees.of(0);
  }

  public static class FieldConstants {
    public final static double k_width = Units.feetToMeters(26.0) + Units.inchesToMeters(5);
    public final static double k_length = Units.feetToMeters(57.0) + Units.inchesToMeters(6.0 + (7.0 / 8.0));
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
