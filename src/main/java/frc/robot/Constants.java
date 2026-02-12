package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class SwerveDriveConstants {
        public static final SwerveDriveKinematics k_kinematics = new SwerveDriveKinematics(); //get this in when we know bot dims
        public static final double k_maxSpeed = Units.feetToMeters(14.5); 
    }

    public static class ControllerConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kPoseControllerPort = 2;

        // Joystick Deadband
        public static final double k_DEADBAND = 0.1;
    }

    public static class TurretConstants { // feet (NOT INCHES), seconds, degrees, pounds (mass), pound*ft/s^2 (force)
        public static final double k_gravitationalAcceleration = 32.174;
        public static final double k_turretHeight = 2.0;
        public static final double k_hubHeight = 6.0;
        public static final double k_ceilingHeight = 15.0;
        public static final double k_fuelRadius = 0.246063;
        public static final double k_fuelMass = 0.474; // estimate
        public static final double k_minYHeightToHub = 0.5 + k_fuelRadius + k_hubHeight - k_turretHeight; 
        public static final double k_minYVelocityToHub = Math.sqrt(2.0 * k_gravitationalAcceleration * k_minYHeightToHub);
        
    } // TODO check and adjust constants

    
    public static class FieldConstants {
        public final static double k_width = Units.feetToMeters(26.0) + Units.inchesToMeters(5);
        public final static double k_length = Units.feetToMeters(57.0) + Units.inchesToMeters(6.0 + (7.0 / 8.0));
    }

    /**
     * Logging-related global constants. Toggle features here so the rest of the code
     * can read a single source of truth for telemetry / autolog / recorder configuration.
     */
    public static class LoggingConstants {
        /** Enable simple Telemetry (Telemetry.log / SmartDashboard) */
        public static final boolean k_enableTelemetry = true;

        /** Enable AutoLog / recording (if you add an AutoLog/recorder integration elsewhere) */
        public static final boolean k_enableAutolog = false;

        /** Enable junction Logger recordings (org.littletonrobotics.junction.Logger) */
        public static final boolean k_enableJunctionLogger = true;

        /** Directory on the robot (or host) where logs should be written if recording is enabled. */
        public static final String k_logDirectory = "/home/lvuser/logs";

        /** Base filename to use for recordings (timestamp/extension may be added by recorder). */
        public static final String k_logFileBaseName = "robotLog";

        /** Target logging period in seconds for periodic recording (0 disables periodic flush). */
        public static final double k_logPeriodSeconds = 0.02; // 20 ms
    }
}
