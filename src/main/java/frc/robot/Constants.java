package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class SwerveDriveConstants {
        private final static Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
        private final static Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
        private final static Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
        private final static Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

        public static final SwerveDriveKinematics k_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        ); //get this in when we know bot dims
        
        public static final double k_maxSpeed = Units.feetToMeters(14.5); 

        public static final double k_maxDriverSpeed = 1.0; // Meters per second
        public static final double k_maxDriverBoostSpeed = 4.5;

        public static final double k_boostScaler = k_maxDriverBoostSpeed / k_maxDriverSpeed;
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

    public static class SimulationConstants{
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
