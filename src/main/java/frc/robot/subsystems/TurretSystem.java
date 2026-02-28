package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

//import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

//import edu.wpi.first.math.Pair;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;

// import edu.wpi.first.math.util.Units;
//import static edu.wpi.first.units.Units.Amps;
//import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
//import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Degrees;
//import static edu.wpi.first.units.Units.DegreesPerSecond;
//import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.ParabolicTrajectory;
// import frc.robot.Telemetry;
// import frc.robot.subsystems.SwerveSystem;

import yams.mechanisms.velocity.FlyWheel;
import yams.mechanisms.positional.Pivot;

public class TurretSystem extends SubsystemBase {

    public static String k_alliance = DriverStation.getAlliance().toString();
    public static Boolean isBlueTeam = k_alliance == "Blue";
    public static Boolean isRedTeam = k_alliance == "Red";
    public static Boolean wonAuto = false; // must be updated

    public static SparkMax flywheelSparkA = null;
    public static SparkMax flywheelSparkB = null;
    public static Pivot turretPitch = null;
    public static Pivot turretYaw = null;
    public static FlyWheel shooter = null;
    public static SparkMax yawMotorSpark = null;
    public class KickerSystem {KickerSystem() {} static KickerSystem getInstance() {return null;} void setSpeed(double a) {}}

    public final Translation3d turretTranslation = new Translation3d(
        TurretConstants.k_turretRelativeX, 
        TurretConstants.k_turretRelativeY, 
        TurretConstants.k_turretHeight);

    public TurretSystem() {

    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/FlywheelVelocityA", flywheelSparkA.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/FlywheelVelocityB", flywheelSparkB.getEncoder().getVelocity());

        turretYaw.updateTelemetry();

        Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
            new Pose3d(
                turretTranslation,
                new Rotation3d(0, turretPitch.getAngle().in(Radians), turretYaw.getAngle().in(Radians)))
        });
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

        // add desmos graph link
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
    public TurretInstruction generateInstruction(Supplier<Pose2d> turretPositionSupplier, Supplier<Translation2d> turretVelocitySupplier) {
        Pose2d turretPosition = turretPositionSupplier.get();
        double turretX = turretPosition.getX();
        double turretY = turretPosition.getY();
        Translation2d turretVelocity = turretVelocitySupplier.get();
        double turretVX = turretVelocity.getX();
        double turretVY = turretVelocity.getY();

        // if (isBlueTeam && getZone(turretX) == 2 || isRedTeam && getZone(turretX) == 4) {
        //     ParabolicTrajectory testTrajectory = ParabolicTrajectory.toHubFromXYWhileDriving(turretX, turretY, turretVX, turretVY);
        //     return new TurretInstruction(false, 
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
            return TurretInstruction.HoldStateDontShoot();
        }
        TurretInstruction testInstruction = new TurretInstruction(true, 
            Degrees.of(testTrajectory.launchDirection), 
            Degrees.of(testTrajectory.launchAngle), 
            launchVelocityToAngular(testTrajectory.launchVelocity));

        double minAllowedAngle = getMinAllowedAngle(turretX, turretY, turretVX, turretVY);
        if (testTrajectory.launchAngle < minAllowedAngle) {
            if (aimingToHub) {
                testTrajectory = ParabolicTrajectory.toHubFromAXYWhileDriving(minAllowedAngle, turretX, turretY, turretVX, turretVY);
                if (testTrajectory == null) {
                    return TurretInstruction.HoldStateDontShoot();
                }
                testInstruction = new TurretInstruction(!underTrenchBar && hubIsActive(gameTime() + testTrajectory.timeToHubScoring()), 
                    Degrees.of(testTrajectory.launchDirection), 
                    Degrees.of(minAllowedAngle), 
                    launchVelocityToAngular(testTrajectory.launchVelocity));
            } else {
                testInstruction.targetPitch = Degrees.of(minAllowedAngle);
            }
        }

        if (!testTrajectory.testValid()) {
            testInstruction.doShoot = false;
        }
        return testInstruction;
    }

    // 70 degree launch to hub
    public TurretInstruction generateInstructionJordanMode(Supplier<Pose2d> turretPositionSupplier, Supplier<Translation2d> turretVelocitySupplier) {
        Pose2d turretPosition = turretPositionSupplier.get();
        double turretX = turretPosition.getX();
        double turretY = turretPosition.getY();
        Translation2d turretVelocity = turretVelocitySupplier.get();
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
            return TurretInstruction.HoldStateDontShoot();
        }
        TurretInstruction testInstruction = new TurretInstruction(!underTrenchBar, 
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

    // aimToTrajectoryFunction(() -> turretPos, () -> turretVel, () -> robotDir, ParabolicTrajectory.toHubFromXYWhileDriving)
    public Command aimToHub(Supplier<Pose2d> turretPositionSupplier, Supplier<Translation2d> turretVelocitySupplier, Supplier<Angle> robotOrientationSupplier) {
        return aimToInstruction(() -> generateInstruction(turretPositionSupplier, turretVelocitySupplier), robotOrientationSupplier);
    }

    public Command aimToInstruction(Supplier<TurretInstruction> instructionSupplier, Supplier<Angle> robotOrientationSupplier) {
        return Commands.parallel(
            setAnglesDynamic(() -> instructionSupplier.get().targetYaw.minus(robotOrientationSupplier.get()), 
                            () -> instructionSupplier.get().targetPitch),
            setSpeedDynamic(() -> instructionSupplier.get().targetVelocity),
            setKickerActivity(() -> instructionSupplier.get().doShoot));
    }

    public Command setSpeed(AngularVelocity speed) {
        return shooter.setSpeed(speed);
    }

    public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
        return shooter.setSpeed(speedSupplier);
    }

    public Command spinUp() {
        return setSpeed(RPM.of(5500));
    }

    public Command stop() {
        return setSpeed(RPM.of(0));
    }

    public AngularVelocity getSpeed() {
        return shooter.getSpeed();
    }

    public Command sysId() {
        return Commands.sequence(
            shooter.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10)),
            turretPitch.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10))
        );
    }

    @Override
    public void simulationPeriodic() {
        shooter.simIterate();
        turretYaw.simIterate();
        turretPitch.simIterate();
    }

    public static AngularVelocity launchVelocityToAngular(double launchVelocity) {
        return RadiansPerSecond.of(launchVelocity / TurretConstants.k_wheelRadius);
    }

    public LinearVelocity getTangentialVelocity() {
        // Calculate tangential velocity at the edge of the wheel and convert to LinearVelocity
        return FeetPerSecond.of(getSpeed().in(RadiansPerSecond) * TurretConstants.k_wheelRadius);
    }



    public Command setAngles(Angle yaw, Angle pitch) {
        return Commands.parallel(
            turretYaw.setAngle(yaw),
            turretPitch.setAngle(pitch));
    }

    public Command setAnglesDynamic(Supplier<Angle> yawSupplier, Supplier<Angle> pitchSupplier) {
        return Commands.parallel(
            turretYaw.setAngle(yawSupplier),
            turretPitch.setAngle(pitchSupplier));
    }

    public Command center() {
        return Commands.parallel(
            turretYaw.setAngle(Degrees.of(0)),
            turretPitch.setAngle(Degrees.of(80)));
    }

    public Angle getRobotRelativeYaw() {
        return turretYaw.getAngle();
    }

    public Angle getFieldRelativeYaw(Angle robotOrientation) {
        return turretYaw.getAngle().minus(robotOrientation);
    }

    public Angle getRawPitch() {
        return turretPitch.getAngle(); // this one does need to be converted
    }

    public Command setYawDutyCycle(double dutyCycle) {
        return turretYaw.set(dutyCycle);
    }

    public Command setPitchDutyCycle(double dutyCycle) {
        return turretPitch.set(dutyCycle);
    }

    public Command rezeroYaw() {
        return Commands.runOnce(() -> yawMotorSpark.getEncoder().setPosition(0), this).withName("TurretYaw.Rezero");
    }

    public static Command setKickerActivity(Supplier<Boolean> on) {return null;}

    public static double launchPitchToMotorPos(double launchPitch) {
        return 0.0; // BRUUUUHHHH
    }
    public static double pitchMotorToLaunchPitch(double motorPos) {
        return 0.0; // tragic
    }


    public class TurretInstruction {
        public boolean doShoot;
        public Angle targetYaw;
        public Angle targetPitch;
        public AngularVelocity targetVelocity;

        public TurretInstruction(boolean doShoot, Angle targetYaw, Angle targetPitch, AngularVelocity targetVelocity) {
            this.doShoot = doShoot;
            this.targetYaw = targetYaw;
            this.targetPitch = targetPitch;
            this.targetVelocity = targetVelocity;
        }

        static TurretInstruction HoldStateDontShoot() {
            return null; // TODOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
            // and be sure that the angle goes above the minimum trench angle by default here
        }
    }
}
//     public class ShootOnTheMoveCommand extends Command {
//         public ShootOnTheMoveCommand(SwerveSubsystem drivetrain, Superstructure superstructure,
//       Supplier<Translation3d> aimPointSupplier) {
//     this.drivetrain = drivetrain;
//     this.superstructure = superstructure;
//     this.aimPointSupplier = aimPointSupplier;

//     // We use the drivetrain to determine linear velocity, but don't require it for
//     // control. We
//     // will be using the superstructure to control the shooting mechanism so it's a
//     // requirement.
//     // addRequirements(superstructure);

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
//     superstructure.aimDynamicCommand(
//         () -> {
//           return this.latestShootSpeed;
//         },
//         () -> {
//           return this.latestTurretAngle;
//         },
//         () -> {
//           return this.latestHoodAngle;
//         }).schedule();
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   @Override
//   public void execute() {
//     // Calculate trajectory to aimPoint
//     var target = aimPointSupplier.get();

//     var shooterLocation = drivetrain.getPose3d().getTranslation()
//         .plus(superstructure.getShooterPose().getTranslation());

//     // Ignore this parameter for now, the range tables will account for it :/
//     // var deltaH = target.getMeasureZ().minus(shooterLocation.getMeasureZ());
//     var shooterOnGround = new Translation2d(shooterLocation.getX(), shooterLocation.getY());
//     var targetOnGround = new Translation2d(target.getX(), target.getY());

//     var distanceToTarget = Meters.of(shooterOnGround.getDistance(targetOnGround));

//     // Get time of flight. We could try to do this analytically but for now it's
//     // easier and more realistic
//     // to use a simple linear approximation based on empirical data.
//     double timeOfFlight = getFlightTime(distanceToTarget);

//     // Calculate corrective vector based on our current velocity multiplied by time
//     // of flight.
//     // If we're stationary, this should be zero. If we're backing up, this will be
//     // "ahead" of the target, etc.
//     var updatedPosition = drivetrain.getFieldVelocity().times(timeOfFlight);
//     var correctiveVector = new Translation2d(updatedPosition.vxMetersPerSecond, updatedPosition.vyMetersPerSecond)
//         .unaryMinus();
//     var correctiveVector3d = new Translation3d(correctiveVector.getX(), correctiveVector.getY(), 0);

//     Logger.recordOutput("FieldSimulation/AimTargetCorrected",
//         new Pose3d(target.plus(correctiveVector3d), Rotation3d.kZero));

//     var correctedTarget = targetOnGround.plus(correctiveVector);

//     var vectorToTarget = drivetrain.getPose().getTranslation().minus(correctedTarget);

//     var correctedDistance = Meters.of(vectorToTarget.getNorm());
//     var calculatedHeading = vectorToTarget.getAngle()
//         .rotateBy(drivetrain.getHeading().unaryMinus())
//         .getMeasure();

//     Logger.recordOutput("ShootOnTheMove/RobotHeading", drivetrain.getHeading());
//     Logger.recordOutput("ShootOnTheMove/CalculatedHeading", calculatedHeading);
//     Logger.recordOutput("ShootOnTheMove/distanceToTarget", distanceToTarget);

//     latestTurretAngle = calculatedHeading;
//     latestShootSpeed = calculateRequiredShooterSpeed(correctedDistance);

//     // TODO: add this back if/when we have a real hood, for now, just set it to the
//     // current angle
//     // latestHoodAngle = calculateRequiredHoodAngle(correctedDistance);
//     latestHoodAngle = superstructure.getHoodAngle();

//     superstructure.setShooterSetpoints(
//         latestShootSpeed,
//         latestTurretAngle,
//         latestHoodAngle);

//     // System.out.println("Shooting at distance: " + correctedDistance + " requires
//     // speed: " + latestShootSpeed
//     // + ", hood angle: " + latestHoodAngle + ", turret angle: " +
//     // latestTurretAngle);
//   }

//   private double getFlightTime(Distance distanceToTarget) {
//     // Simple linear approximation based on empirical data.
//     return TIME_OF_FLIGHT_BY_DISTANCE.get(distanceToTarget.in(Meters));
//   }
//     }