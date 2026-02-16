package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.ParabolicTrajectory;
// import frc.robot.Telemetry;
import frc.robot.subsystems.SwerveSystem;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;

import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;

import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class TurretSystem extends SubsystemBase {

    public final Translation3d turretTranslation = new Translation3d(
        TurretConstants.k_turretRelativeX, 
        TurretConstants.k_turretRelativeY, 
        TurretConstants.k_turretHeight);

    private final SparkMax yawMotorSpark = new SparkMax(TurretConstants.k_yawMotorId, MotorType.kBrushless);

    private final SparkMax pitchMotorSpark = new SparkMax(TurretConstants.k_pitchMotorId, MotorType.kBrushless);

    private SparkMax flywheelSparkA = new SparkMax(TurretConstants.k_flywheelMotorIdA, MotorType.kBrushless);
    private SparkMax flywheelSparkB = new SparkMax(TurretConstants.k_flywheelMotorIdB, MotorType.kBrushless);

    private final SmartMotorControllerConfig smcConfigShooter = new SmartMotorControllerConfig(this)
        .withFollowers(Pair.of(flywheelSparkB, true))
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(0.00936, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
        .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
        .withMotorInverted(false)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40));

    private final SmartMotorController smcShooter = new SparkWrapper(flywheelSparkA, DCMotor.getKrakenX60(2), smcConfigShooter);

    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smcShooter)
        .withDiameter(Inches.of(4))
        .withMass(Pounds.of(1))
        .withUpperSoftLimit(RPM.of(6000))
        .withLowerSoftLimit(RPM.of(0))
        .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

    private final FlyWheel shooter = new FlyWheel(shooterConfig);


    private SmartMotorControllerConfig smcConfigTurretYaw = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(15.0, 0, 0, DegreesPerSecond.of(2440), DegreesPerSecondPerSecond.of(2440))
        .withFeedforward(new SimpleMotorFeedforward(0, 7.5, 0))
        .withTelemetry("TurretYawMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 3)))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(10))
        .withClosedLoopRampRate(Seconds.of(0.1))
        .withOpenLoopRampRate(Seconds.of(0.1));

    private SmartMotorController smcTurretYaw = new SparkWrapper(yawMotorSpark, DCMotor.getNEO(1), smcConfigTurretYaw);

    private final PivotConfig turretYawConfig = new PivotConfig(smcTurretYaw)
        .withStartingPosition(Degrees.of(0))
        // .withMOI(0.05)   tOdO check this with engineering
        .withTelemetry("TurretYaw", TelemetryVerbosity.HIGH)
        .withMechanismPositionConfig(new MechanismPositionConfig()
            .withMovementPlane(Plane.XY)
            .withRelativePosition(turretTranslation));

    private Pivot turretYaw = new Pivot(turretYawConfig);


    private SmartMotorControllerConfig smcConfigTurretPitch = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(15.0, 0, 0, DegreesPerSecond.of(2440), DegreesPerSecondPerSecond.of(2440))
        .withFeedforward(new SimpleMotorFeedforward(0, 7.5, 0))
        .withTelemetry("TurretPitchMotor", TelemetryVerbosity.HIGH)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
        .withMotorInverted(true)
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(10))
        .withClosedLoopRampRate(Seconds.of(0.1))
        .withOpenLoopRampRate(Seconds.of(0.1));

    private SmartMotorController smcTurretPitch = new SparkWrapper(pitchMotorSpark, DCMotor.getNEO(1), smcConfigTurretPitch);

    private final PivotConfig turretPitchConfig = new PivotConfig(smcTurretPitch)
        .withStartingPosition(Degrees.of(80))
        .withHardLimit(Degrees.of(TurretConstants.k_minLaunchAngle), Degrees.of(TurretConstants.k_maxLaunchAngle))
        // .withMOI(0.05)   tOdO check this with engineering
        .withTelemetry("TurretPitch", TelemetryVerbosity.HIGH)
        .withMechanismPositionConfig(new MechanismPositionConfig()
            .withMovementPlane(Plane.XZ)
            .withRelativePosition(turretTranslation));

    private Pivot turretPitch = new Pivot(turretPitchConfig);


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

    public Pose3d getTurretPosition() {
        return new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)); // tOdO gibb dis
    }
    public Translation2d getTurretVelocity() { // tOdO gib mehr
        return new Translation2d(0, 0);
    }



    public Command aimToHub(Supplier<ParabolicTrajectory> trajectorySupplier) {
        return Commands.parallel(
            setAnglesDynamic(() -> {return toRobotOrientation(trajectorySupplier.get().launchDirection);}, 
                             () -> {return trajectorySupplier.get().launchAngle;}),
            setSpeedDynamic(() -> {return launchVelocityToAngular(trajectorySupplier.get().launchVelocity);}));
        // Pose3d position = getTurretPosition();
        // Translation2d velocity = getTurretVelocity();
        // ParabolicTrajectory trajectory = ParabolicTrajectory.toHubFromXYWhileDriving(
        //     position.getX(), position.getY(), velocity.getX(), velocity.getY());
    }

    public Command setSpeed(AngularVelocity speed) {
        return shooter.setSpeed(speed);
    }

    public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
        return shooter.setSpeed(speedSupplier);
    }

    public Command spinUp() {
        return setSpeed(RPM.of(5500));

        // return setSpeed(RotationsPerSecond.of(50));

        // return run(() -> {
        // // followerNova.follow(leaderNova.getID());
        // // followerNova.setInverted(true);

        // // leaderNova.setPercent(SHOOTER_SPEED);
        // // followerNova.setPercent(SHOOTER_SPEED);

        // // followerNova.setPercent(0.5);
        // });

        // return shooter.set(0.5);
        // return shooter.setSpeed(RotationsPerSecond.of(500));
    }

    public Command stop() {
        return setSpeed(RPM.of(0));
        // return run(() -> {

        // // leaderNova.setPercent(0);
        // // followerNova.setPercent(0);
        // // followerNova.setPercent(0.5);
        // });
        // return shooter.set(0);
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

    public static Angle toRobotOrientation(double fieldDirection) {
        return Degrees.of(fieldDirection - SwerveSystem.getInstance().getSwerveDrive().getRotation().in(Degrees)); // how to actually access robot SwerveSystem?
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
        // Returns the turret angle in the robot's coordinate frame ---- shouldn't this already be from the robot's frame?
        return turretYaw.getAngle();
    }

    public Angle getFieldRelativeYaw() {
        return turretYaw.getAngle(); // shouldn't this one be the one that needs to be converted?
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