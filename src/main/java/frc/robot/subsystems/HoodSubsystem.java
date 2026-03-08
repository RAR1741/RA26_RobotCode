package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HoodSubsystem extends SubsystemBase {
  private Angle MIN_ANGLE = Degrees.of(45); // degrees
  private Angle MAX_ANGLE = Degrees.of(80); // degrees

  private TalonFX hoodKraken = new TalonFX(Constants.HoodConstants.k_hoodMotorId);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(15.0, 0, 0,
          DegreesPerSecond.of(2440),
          DegreesPerSecondPerSecond.of(2440))
      // TODO: make this work, and not error
      .withFeedforward(new SimpleMotorFeedforward(0.0, 7.5, 0.0))
      .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.0 / 0.891)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      .withSoftLimit(MIN_ANGLE, MAX_ANGLE)
      .withStatorCurrentLimit(Amps.of(2.0)) // TODO: make this not 0 to actually run
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

  private SmartMotorController smc = new TalonFXWrapper(hoodKraken, DCMotor.getNEO(1), smcConfig);

  private final PivotConfig hoodConfig = new PivotConfig(smc)
      .withHardLimit(MIN_ANGLE, MAX_ANGLE)
      // .withStartingPosition(Degrees.of(0)) // TODO: this breaks everything
      // .withMOI(0.05)
      .withTelemetry("Hood", TelemetryVerbosity.HIGH);

  private Pivot hood = new Pivot(hoodConfig);

  public HoodSubsystem() {
    this.setDefaultCommand(Commands.runOnce(() -> smc.setDutyCycle(0), this));
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

  public Angle getAngle() {
    return hood.getAngle();
  }

  public Command set(double dutyCycle) {
    return hood.set(dutyCycle);
  }

  public Command sysId() {
    return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  @Override
  public void periodic() {
    hood.updateTelemetry();

    Logger.recordOutput("Hood/Angle", hood.getAngle());
    Logger.recordOutput("Hood/MechanismSetpoint",
        hood.getMechanismSetpoint().orElse(Degrees.of(9999)));
  }

  @Override
  public void simulationPeriodic() {
    hood.simIterate();
  }
}
