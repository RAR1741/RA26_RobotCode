package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class SwerveDriveConstants {
        public static final SwerveDriveKinematics k_kinematics = new SwerveDriveKinematics();
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
        public static final double k_turretHeight = 2.0; // adjust to real design
        public static final double k_extraTimeToPassSensor = 1.0; // test on field
    }

    public static class FieldConstants { // feet (NOT INCHES)
        public static final double k_hubWidth = 41.7 / 12.0;
        public static final double k_hubRadius = hubWidth / Math.sqrt(3.0);
        public static final double k_hubHeight = 6.0;
        public static final double k_hubX = 0.0; // adjust to field coordinate convention
        public static final double k_hubY = (158.6 + (47.0 / 2)) / 12.0; // same as above
        public static final double k_fuelRadius = 5.91 / 12.0 / 2.0;
        public static final double k_fuelMass = 0.474; // average
        public static final double k_ceilingHeight = 15.0; // estimated
    }
} // TODO check and adjust constants
