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
// import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
// import frc.robot.ParabolicTrajectory;
// import frc.robot.Telemetry;
// import frc.robot.subsystems.LimelightSystem;
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
        .withMOI(0.05)
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
        .withStartingPosition(Degrees.of(0))
        .withMOI(0.05)
        .withTelemetry("TurretPitch", TelemetryVerbosity.HIGH)
        .withMechanismPositionConfig(new MechanismPositionConfig()
            .withMovementPlane(Plane.XY)
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

    public Command aimToHub() {
        return turretPitch.setAngle(Degrees.of(80));
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

    public Command sysIds() {
        return Commands.sequence(
            shooter.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10)),
            turretYaw.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10)),
            turretPitch.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10))
        );
    }

    @Override
    public void simulationPeriodic() {
        shooter.simIterate();
        turretYaw.simIterate();
        turretPitch.simIterate();
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

    public Angle getRobotAdjustedYaw() {
        // Returns the turret angle in the robot's coordinate frame
        return turretYaw.getAngle();
    }

    public Angle getRawYaw() {
        return turretYaw.getAngle();
    }

    public Angle getRawPitch() {
        return turretPitch.getAngle();
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