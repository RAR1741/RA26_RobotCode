package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;

// import edu.wpi.first.math.util.Units;
// import static edu.wpi.first.units.Units.Amps;
// import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
// import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.DegreesPerSecond;
// import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.ParabolicTrajectory;
// import frc.robot.Telemetry;
// import frc.robot.subsystems.SwerveSystem;

public class ShooterInstruction {

    public static String k_alliance = DriverStation.getAlliance().toString();
    public static Boolean isBlueTeam = k_alliance == "Blue";
    public static Boolean isRedTeam = k_alliance == "Red";
    public static Boolean wonAuto = false; // must be updated

    public final Translation3d turretTranslation = new Translation3d(
        TurretConstants.k_turretRelativeX, 
        TurretConstants.k_turretRelativeY, 
        TurretConstants.k_turretHeight);

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

    static ShooterInstruction HoldStateDontShoot() {
        return null; // definitely todo
        // and be sure that the angle goes above the minimum trench angle by default here
    }

    public static boolean hubIsActive(double time) {
        return time < 30.0 || time >= 130.0 || wonAuto && (time >= 55.0 && time < 80.0 || time >= 105.0) || !wonAuto && (time < 55.0 || time >= 80.0 && time < 105.0);
    }

    public static double gameTime() {
        return (DriverStation.isAutonomous()? 20.0 : 140.0) - DriverStation.getMatchTime();
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

    public double getMinAllowedAngle(double robotX, double robotY, double robotVX, double robotVY) {
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
        } else if (trenchDistance < 0.0) {return TurretConstants.k_minAngleUnderTrench;}

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
        return pitchMotorToLaunchPitch(TurretConstants.k_maxTrenchPitchMotorPos + availableTime * TurretConstants.k_maxPitchMotorSpeed);
    }

    public Pose3d getTurretPose() {
        return new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)); // tOdO gibb dis
    }
    public Translation2d getTurretVelocity() { // tOdO gib mehr
        return new Translation2d(0, 0);
    }

    // see if these parameters need to be listed as suppliers
    public static ShooterInstruction generateInstruction(Supplier<Pose2d> turretPositionSupplier, Supplier<Translation2d> turretVelocitySupplier) {
        Pose2d turretPosition = turretPositionSupplier.get();
        double turretX = turretPosition.getX();
        double turretY = turretPosition.getY();
        Translation2d turretVelocity = turretVelocitySupplier.get();
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
            return ShooterInstruction.HoldStateDontShoot();
        }
        ShooterInstruction testInstruction = new ShooterInstruction(true, 
            Degrees.of(testTrajectory.launchDirection), 
            Degrees.of(testTrajectory.launchAngle), 
            launchVelocityToAngular(testTrajectory.launchVelocity));

        double minAllowedAngle = getMinAllowedAngleToHub(turretX, turretY, turretVX, turretVY);
        if (aimingToHub) {
            if (testTrajectory.launchAngle < minAllowedAngle) {
                testTrajectory = ParabolicTrajectory.toHubFromAXYWhileDriving(minAllowedAngle, turretX, turretY, turretVX, turretVY);
                if (testTrajectory == null) {
                    return ShooterInstruction.HoldStateDontShoot();
                }
                testInstruction = new ShooterInstruction(!underTrenchBar && hubIsActive(gameTime() + testTrajectory.timeToHubScoring()), 
                    Degrees.of(testTrajectory.launchDirection), 
                    Degrees.of(minAllowedAngle), 
                    launchVelocityToAngular(testTrajectory.launchVelocity));
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
    public static ShooterInstruction generateInstructionJordanMode() {
        Translation2d turretPosition = getTurretPose().getTranslation();
        double turretX = turretPosition.getX();
        double turretY = turretPosition.getY();
        Translation2d turretVelocity = getTurretVelocity();
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
            return ShooterInstruction.HoldStateDontShoot();
        }
        ShooterInstruction testInstruction = new ShooterInstruction(!underTrenchBar, 
            Degrees.of(testTrajectory.launchDirection), 
            Degrees.of(testTrajectory.launchAngle), 
            launchVelocityToAngular(testTrajectory.launchVelocity));

        double minAllowedAngle = getMinAllowedAngle(turretX, turretY, turretVX, turretVY); // min allowed angle is always at or below 70 degrees
        if (testTrajectory.launchAngle < minAllowedAngle) {
            testInstruction.targetPitch = Degrees.of(minAllowedAngle); // just assumes zone shot behavior
        }

        if (!testTrajectory.testValid()) {
            testInstruction.doShoot = false;
        }
        return testInstruction;
    }
}
//     public class ShootOnTheMoveCommand extends Command {
//         public ShootOnTheMoveCommand(SwerveSubsystem drivetrain, Superstructure superstructure,
//       Supplier<Translation3d> aimPointSupplier) {
//     this.drivetrain = drivetrain;
//     this.superstructure = superstructure;
//     this.aimPointSupplier = aimPointSupplier;

//     // TODO: figure out if the above is actually required. Right now, when you start
//     // some other command, the auto aim can't start back up again
//   }

//   @Override
//   public void initialize() {
//     super.initialize();

//     latestHoodAngle = superstructure.getHoodAngle();
//     latestTurretAngle = superstructure.getTurretAngle();
//     latestShootSpeed = superstructure.getShooterSpeed();

//     // TODO: when this current command ends, we should probably cancel the dynamic
//     // aim command
//     superstructure.aimDynamicCommand(() -> this.latestShootSpeed, () -> this.latestTurretAngle, () -> this.latestHoodAngle).schedule();
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   @Override
//   public void execute() {
//     // Calculate trajectory to aimPoint
//     Translation3d target = aimPointSupplier.get();

//     Translation3d shooterLocation = drivetrain.getPose3d().getTranslation()
//         .plus(superstructure.getShooterPose().getTranslation());

//     Translation2d shooterOnGround = new Translation2d(shooterLocation.getX(), shooterLocation.getY());
//     Translation2d targetOnGround = new Translation2d(target.getX(), target.getY());

//     Meters distanceToTarget = Meters.of(shooterOnGround.getDistance(targetOnGround));

//     // Get time of flight. We could try to do this analytically but for now it's
//     // easier and more realistic
//     // to use a simple linear approximation based on empirical data.
//     double timeOfFlight = getFlightTime(distanceToTarget);

//     // Calculate corrective vector based on our current velocity multiplied by time
//     // of flight.
//     // If we're stationary, this should be zero. If we're backing up, this will be
//     // "ahead" of the target, etc.
//     var updatedPosition = drivetrain.getFieldVelocity().times(timeOfFlight);
//     Translation2d correctiveVector = new Translation2d(updatedPosition.vxMetersPerSecond, updatedPosition.vyMetersPerSecond)
//         .unaryMinus();
//     Translation3d correctiveVector3d = new Translation3d(correctiveVector.getX(), correctiveVector.getY(), 0);

//     Logger.recordOutput("FieldSimulation/AimTargetCorrected",
//         new Pose3d(target.plus(correctiveVector3d), Rotation3d.kZero));

//     Translation2d correctedTarget = targetOnGround.plus(correctiveVector);

//     Translation2d vectorToTarget = drivetrain.getPose().getTranslation().minus(correctedTarget);

//     Meters correctedDistance = Meters.of(vectorToTarget.getNorm());
//     var calculatedHeading = vectorToTarget.getAngle()
//         .rotateBy(drivetrain.getHeading().unaryMinus())
//         .getMeasure();

//     Logger.recordOutput("ShootOnTheMove/RobotHeading", drivetrain.getHeading());
//     Logger.recordOutput("ShootOnTheMove/CalculatedHeading", calculatedHeading);
//     Logger.recordOutput("ShootOnTheMove/distanceToTarget", distanceToTarget);

//     latestTurretAngle = calculatedHeading;
//     latestShootSpeed = calculateRequiredShooterSpeed(correctedDistance);

//     latestHoodAngle = calculateRequiredHoodAngle(correctedDistance);

//     superstructure.setShooterSetpoints(
//         latestShootSpeed,
//         latestTurretAngle,
//         latestHoodAngle);

//     // System.out.println("Shooting at distance: " + correctedDistance + " requires
//     // speed: " + latestShootSpeed
//     // + ", hood angle: " + latestHoodAngle + ", turret angle: " +
//     // latestTurretAngle);
//   }
//     }