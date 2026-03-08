package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.wrappers.REVThroughBoreEncoder;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
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
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

public class HoodSubsystem extends SubsystemBase {
  // 1 Neo, 5:1 gearbox, 60:12 pivot gearing, non-continuous 360 deg
  // Total reduction: 5 * 5 = 25:1

  private double MAX_ANGLE = 80; // degrees
  private double MIN_ANGLE = 45; // degrees

  private TalonFX hoodKraken = new TalonFX(Constants.HoodConstants.k_hoodMotorId);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(15.0, 0, 0,
          DegreesPerSecond.of(2440),
          DegreesPerSecondPerSecond.of(2440))
      // TODO: make this work, and not error
      // .withFeedforward(new SimpleMotorFeedforward(0.0, 7.5, 0.0))
      .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withSoftLimit(Degrees.of(MIN_ANGLE), Degrees.of(MAX_ANGLE))
      .withStatorCurrentLimit(Amps.of(0)) // TODO: make this not 0 to actually run
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

  private SmartMotorController smc = new TalonFXWrapper(hoodKraken, DCMotor.getNEO(1), smcConfig);

  private final PivotConfig hoodConfig = new PivotConfig(smc)
      .withHardLimit(Degrees.of(MIN_ANGLE), Degrees.of(MAX_ANGLE))
      .withStartingPosition(Degrees.of(0))
      // .withMOI(0.05)
      .withTelemetry("Hood", TelemetryVerbosity.HIGH);

  private Pivot hood = new Pivot(hoodConfig);

  public HoodSubsystem() {
  }

  // public Command rezero() {
  // return Commands.runOnce(() ->
  // hoodKraken.getEncoder().setPosition(computeTurretAngleFromAbs()), this)
  // .withName("Hood.Rezero");
  // }

  public Command setAngle(Angle angle) {
    return hood.setAngle(angle);
  }

  public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier) {
    return hood.setAngle(turretAngleSupplier);
  }

  public Command center() {
    return hood.setAngle(Degrees.of(0));
  }

  public Angle getRobotAdjustedAngle() {
    // Returns the turret angle in the robot's coordinate frame
    // since the turret is mounted backwards, we need to add 180 degrees
    return hood.getAngle().plus(Degrees.of(180));
  }

  public Angle getAngle() {
    return hood.getAngle();
  }

  public Command set(double dutyCycle) {
    return hood.set(dutyCycle);
  }

  public Command sysId() {
    return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  // @Override
  // public void periodic() {
  // hood.updateTelemetry();

  // Logger.recordOutput("Turret/RelYamsAngle", getAngle());
  // Logger.recordOutput("Turret/computeTurretAngleFromAbs",
  // computeTurretAngleFromAbs());

  // // Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
  // // new Pose3d(
  // // turretTranslation,
  // // new Rotation3d(0, 0, turret.getAngle().in(Radians)))
  // // });
  // }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }
}
