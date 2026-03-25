package frc.robot.subsystems;

import java.util.function.Supplier;

// import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.Telemetry;
// import frc.robot.subsystems.SwerveSystem;

public class ShooterInstruction {

    public static String k_alliance = DriverStation.getAlliance().toString();
    public static Boolean isBlueTeam = k_alliance == "Blue";
    public static Boolean isRedTeam = k_alliance == "Red";
    public static Boolean wonAuto = false; // must be updated

    public boolean doShoot;
    public Angle targetYaw;
    public Angle targetPitch;
    public AngularVelocity targetVelocity;

    public ShooterInstruction(boolean doShoot, Angle targetYaw, Angle targetPitch, AngularVelocity targetVelocity) {
        this.doShoot = doShoot;
        this.targetYaw = targetYaw;
        this.targetPitch = targetPitch;
        this.targetVelocity = targetVelocity;
    }

    public static ShooterInstruction HoldStateDontShoot(Rotation3d rot3d, AngularVelocity angVel) {
        // return null; // definitely todo
        // and be sure that the angle goes above the minimum trench angle by default here
        return new ShooterInstruction(false, rot3d.getMeasureZ(), Degrees.of(90.0).minus(rot3d.getMeasureY()), angVel);
    }

    public static boolean hubIsActive(double time) {
        return time < 30.0 || time >= 130.0 || wonAuto && (time >= 55.0 && time < 80.0 || time >= 105.0) || !wonAuto && (time < 55.0 || time >= 80.0 && time < 105.0);
    }

    public static double gameTime() {
        return (DriverStation.isAutonomous()? 20.0 : 160.0) - DriverStation.getMatchTime();
    }

    public static int getZone(double turretX) {
        return (turretX < 0.0 || turretX > FieldConstants.k_fieldLength)? -1 : 
            (turretX < FieldConstants.k_blueHubX - FieldConstants.k_trenchBarDepth / 2.0 - TurretConstants.k_turretMaxHorizontalRadius)? 1 : 
            (turretX <= FieldConstants.k_blueHubX + FieldConstants.k_trenchBarDepth / 2.0 + TurretConstants.k_turretMaxHorizontalRadius)? 2 : 
            (turretX < FieldConstants.k_redHubX - FieldConstants.k_trenchBarDepth / 2.0 - TurretConstants.k_turretMaxHorizontalRadius)? 3 : 
            (turretX <= FieldConstants.k_redHubX + FieldConstants.k_trenchBarDepth / 2.0 + TurretConstants.k_turretMaxHorizontalRadius)? 4 : 5;
    }

    public static boolean getYInTrench(double turretY) {
        return turretY <= FieldConstants.k_trenchWidth || turretY >= FieldConstants.k_fieldWidth - FieldConstants.k_trenchWidth;
    }

    public static double getMinAllowedAngleToHub(double robotX, double robotY, double robotVX, double robotVY) {
        boolean yInTrench = getYInTrench(robotY);

        // graph link: https://www.desmos.com/3d/k1wjvps1p3
        double trenchDistance = Math.abs(robotX - (FieldConstants.k_fieldLength + 
            (FieldConstants.k_neutralZoneDepth + FieldConstants.k_hubZoneDepth) * 
                ((robotX > FieldConstants.k_fieldLength / 2.0)? 1.0 : -1.0)) / 2.0)
             - FieldConstants.k_hubZoneDepth / 2.0 - TurretConstants.k_turretDistToRobotCenter;

        if (!yInTrench) {
            trenchDistance = Math.hypot(trenchDistance, 
                FieldConstants.k_fieldWidth / 2.0 - FieldConstants.k_trenchWidth - 
                    Math.abs(robotY - FieldConstants.k_hubY));
        } else if (trenchDistance < 0.0) {
            return TurretConstants.k_minAngleUnderTrench;
        }

        double trenchDistanceGradientX; // :)  :D  :P  we love gradients  :>  C:  'v'  yaaaay  :]  :3  'u'
        double trenchDistanceGradientY;
        if (yInTrench) {
            trenchDistanceGradientX = (robotX < FieldConstants.k_blueHubX || (robotX > FieldConstants.k_fieldLength / 2.0 && robotX < FieldConstants.k_redHubX))? 1.0 : -1.0;
            trenchDistanceGradientY = 0.0;
        } else {
            double nearestY = robotY > FieldConstants.k_hubY? FieldConstants.k_fieldWidth - FieldConstants.k_trenchWidth : FieldConstants.k_trenchWidth;
            double nearestX = FieldConstants.k_allianceZoneDepth + 
                ((robotX < FieldConstants.k_blueHubX || robotX > FieldConstants.k_redHubX)? 0.0 : FieldConstants.k_hubZoneDepth);
            if (robotX > FieldConstants.k_fieldLength / 2.0) {
                nearestX = FieldConstants.k_fieldLength - nearestX;
            }
            double trenchDistanceAngle = Math.atan2(nearestY - robotY, nearestX - robotX);
            trenchDistanceGradientX = Math.cos(trenchDistanceAngle);
            trenchDistanceGradientY = Math.sin(trenchDistanceAngle);
        }
        double vIntoTrench = robotVX * trenchDistanceGradientX + robotVY * trenchDistanceGradientY; // dot product
        double maxAcceleration = Constants.SwerveDriveConstants.k_maxAcceleration;
        // x = dx - v * t - a/2 * t^2 = 0
        // double availableTime = (Math.sqrt(vIntoTrench * vIntoTrench + 2.0 * maxAcceleration * trenchDistance) - vIntoTrench) / maxAcceleration;
        double availableTime = ParabolicTrajectory.qFormulaGreater(-maxAcceleration / 2.0, -vIntoTrench, trenchDistance);
        return TurretConstants.k_minAngleUnderTrench - availableTime * TurretConstants.k_maxAngleChangingRate;
        // AngleChangerSystem.pitchMotorToLaunchPitch(TurretConstants.k_maxTrenchPitchMotorPos + availableTime * TurretConstants.k_maxPitchMotorSpeed);
    }

    public static boolean activeHubOnShot(ParabolicTrajectory testTrajectory) {
        return hubIsActive(gameTime() + testTrajectory.timeToHubScoring());
    }

    // see if these parameters need to be listed as suppliers
    public static ShooterInstruction generateInstruction(Pose2d turretPosition, Translation2d turretVelocity, Rotation3d currentTurretRotation, AngularVelocity currentShooterVelocity) {
        double turretX = turretPosition.getX();
        double turretY = turretPosition.getY();
        double turretVX = turretVelocity.getX();
        double turretVY = turretVelocity.getY();

        // if (isBlueTeam && getZone(turretX) == 2 || isRedTeam && getZone(turretX) == 4) {
        //     ParabolicTrajectory testTrajectory = ParabolicTrajectory.toHubFromXYWhileDriving(turretX, turretY, turretVX, turretVY);
        //     return new ShooterInstruction(false, 
        //         Degrees.of(testTrajectory.launchDirection), 
        //         Degrees.of(Math.max(testTrajectory.launchAngle, TurretConstants.k_minAngleUnderTrench)), 
        //         launchVelocityToAngular(testTrajectory.launchVelocity));
        // } else {
        ParabolicTrajectory testTrajectory;
        int zone = getZone(turretX);
        boolean aimingToHub = isBlueTeam && zone <= 2 || isRedTeam && zone >= 4;
        boolean underTrenchBar = isBlueTeam && zone == 2 || isRedTeam && zone == 4;
        
        if (aimingToHub) {
            testTrajectory = ParabolicTrajectory.toHubFromXYWhileDriving(turretX, turretY, turretVX, turretVY);
        } else {
            testTrajectory = ParabolicTrajectory.toZoneFromXYWhileDriving(turretX, turretY, turretVX, turretVY);
        }
        if (testTrajectory == null) {
            return ShooterInstruction.HoldStateDontShoot(currentTurretRotation, currentShooterVelocity);
        }
        ShooterInstruction testInstruction = new ShooterInstruction(!underTrenchBar && (!aimingToHub || activeHubOnShot(testTrajectory)), 
            Degrees.of(testTrajectory.launchDirection), 
            Degrees.of(testTrajectory.launchAngle), 
            ShooterSubsystem.launchVelocityToAngular(testTrajectory.launchVelocity));

        double minAllowedAngle = getMinAllowedAngleToHub(turretX, turretY, turretVX, turretVY);
        double maxAllowedAngle = 80.0; // getMaxAllowedAngleToHub(turretX, turretY, turretVX, turretVY);
        if (aimingToHub) {
            if (testTrajectory.launchAngle < minAllowedAngle) {
                testTrajectory = ParabolicTrajectory.toHubFromAXYWhileDriving(minAllowedAngle, turretX, turretY, turretVX, turretVY);
                if (testTrajectory == null) {
                    return ShooterInstruction.HoldStateDontShoot(currentTurretRotation, currentShooterVelocity);
                }
                testInstruction = new ShooterInstruction(!underTrenchBar && activeHubOnShot(testTrajectory), 
                    Degrees.of(testTrajectory.launchDirection), 
                    Degrees.of(minAllowedAngle), 
                    ShooterSubsystem.launchVelocityToAngular(testTrajectory.launchVelocity));
            } else if (testTrajectory.launchAngle > maxAllowedAngle) {
                testInstruction.targetPitch = Degrees.of(maxAllowedAngle);
                testInstruction.doShoot = false;
            }
        } else {
            if (testTrajectory.launchAngle < minAllowedAngle) {
                testInstruction.targetPitch = Degrees.of(minAllowedAngle);
            } else if (testTrajectory.launchAngle > maxAllowedAngle) {
                testInstruction.targetPitch = Degrees.of(maxAllowedAngle);
            }
        }

        if (!testTrajectory.testValid()) {
            testInstruction.doShoot = false;
        }
        return testInstruction;
    }

    // 70 degree launch to hub
    public static ShooterInstruction generateInstructionJordanMode(Translation2d turretPosition, Translation2d turretVelocity, Rotation3d currentTurretRotation, AngularVelocity currentShooterVelocity) {
        double turretX = turretPosition.getX();
        double turretY = turretPosition.getY();
        double turretVX = turretVelocity.getX();
        double turretVY = turretVelocity.getY();

        ParabolicTrajectory testTrajectory;
        int zone = getZone(turretX);
        boolean aimingToHub = isBlueTeam && zone <= 2 || isRedTeam && zone >= 4;
        boolean underTrenchBar = isBlueTeam && zone == 2 || isRedTeam && zone == 4;
        if (aimingToHub) {
            testTrajectory = ParabolicTrajectory.toHubFromAXYWhileDriving(70.0, turretX, turretY, turretVX, turretVY);
        } else {
            testTrajectory = ParabolicTrajectory.toZoneFromXYWhileDriving(turretX, turretY, turretVX, turretVY);
        }
        if (testTrajectory == null) {
            return ShooterInstruction.HoldStateDontShoot(currentTurretRotation, currentShooterVelocity);
        }
        ShooterInstruction testInstruction = new ShooterInstruction(!underTrenchBar && (!aimingToHub || activeHubOnShot(testTrajectory)), 
            Degrees.of(testTrajectory.launchDirection), 
            Degrees.of(testTrajectory.launchAngle), 
            ShooterSubsystem.launchVelocityToAngular(testTrajectory.launchVelocity));

        double minAllowedAngle = getMinAllowedAngleToHub(turretX, turretY, turretVX, turretVY); // min allowed angle is always at or below 70 degrees
        if (testTrajectory.launchAngle < minAllowedAngle) {
            testInstruction.targetPitch = Degrees.of(minAllowedAngle); // just assumes zone shot behavior
        }

        if (!testTrajectory.testValid()) {
            testInstruction.doShoot = false;
        }
        return testInstruction;
    }

    public static ShooterInstruction generateInstructionBareMinimumFunctional(Translation2d turretPosition, Translation2d turretVelocity, Rotation3d currentTurretRotation, AngularVelocity currentShooterVelocity) {
        double turretX = turretPosition.getX();
        double turretY = turretPosition.getY();
        double turretVX = turretVelocity.getX();
        double turretVY = turretVelocity.getY();

        ParabolicTrajectory testTrajectory = null;
        int zone = getZone(turretX);
        boolean aimingToHub = isBlueTeam && zone <= 2 || isRedTeam && zone >= 4;
        boolean underTrenchBar = isBlueTeam && zone == 2 || isRedTeam && zone == 4;
        if (aimingToHub) {
            testTrajectory = ParabolicTrajectory.toHubFromAXYWhileDriving(70.0, turretX, turretY, turretVX, turretVY);
        }
        if (testTrajectory == null) {
            return HoldStateDontShoot(currentTurretRotation, currentShooterVelocity);
        }
        ShooterInstruction testInstruction = new ShooterInstruction(!underTrenchBar && (!aimingToHub || activeHubOnShot(testTrajectory)), 
            Degrees.of(testTrajectory.launchDirection), 
            Degrees.of(testTrajectory.launchAngle), 
            ShooterSubsystem.launchVelocityToAngular(testTrajectory.launchVelocity));

        if (!testTrajectory.testValid()) {
            testInstruction.doShoot = false;
        }
        return testInstruction;
    }
}