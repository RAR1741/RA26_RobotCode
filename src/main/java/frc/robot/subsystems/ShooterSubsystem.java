package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX leaderTalon = new TalonFX(
      Constants.ShooterConstants.k_leaderMotorId,
      Constants.ctreCANBus);

  private final TalonFX followerTalon = new TalonFX(
      Constants.ShooterConstants.k_followerMotorId,
      Constants.ctreCANBus);

  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(followerTalon, true))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.00936, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private final SmartMotorController smc = new TalonFXWrapper(leaderTalon, DCMotor.getKrakenX60(2), smcConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(0))
      .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

  private final FlyWheel shooter = new FlyWheel(shooterConfig);

  public ShooterSubsystem() {
    this.setDefaultCommand(Commands.runOnce(() -> smc.setDutyCycle(0), this));
  }

  public Command shoot() {
    return setSpeed(RPM.of(3000));
  }

  public Command setSpeed(AngularVelocity speed) {
    return shooter.setSpeed(speed);
  }

  public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
    return shooter.setSpeed(speedSupplier);
  }

  public Command stopCommand() {
    return Commands.run(() -> setSpeed(RPM.of(0)));
  }

  public AngularVelocity getSpeed() {
    return shooter.getSpeed();
  }

  public boolean isAtSpeed(AngularVelocity targetSpeed) {
    // Tolerance of 100 RPM, adjust as needed
    return Math.abs(leaderTalon.getVelocity().getValueAsDouble() - targetSpeed.in(RPM)) < 100;
  }

  public AngularVelocity getCurrentSpeed() {
    return RPM.of(leaderTalon.getVelocity().getValueAsDouble());
  }

  public boolean isAtSpeedWithTolerance(AngularVelocity targetSpeed, double tolerance) {
    return Math
        .abs((leaderTalon.getVelocity().getValueAsDouble() - targetSpeed.in(RPM))
            / (targetSpeed.in(RPM))) < tolerance;
  }

  public Command setSpeedWithToleranceCommand(AngularVelocity targetSpeed, double tolerance) {
    return Commands.run(() -> setSpeed(targetSpeed))
        .until(() -> isAtSpeedWithTolerance(targetSpeed, tolerance))
        .withName("ShooterSystem.setSpeedWithToleranceCommand");
  }

  @Override
  public void periodic() {
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }
}
