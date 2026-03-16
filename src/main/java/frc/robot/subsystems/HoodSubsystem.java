package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
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
  public Angle MIN_ANGLE = Degrees.of(40);
  public Angle MAX_ANGLE = Degrees.of(80);

  private double GEAR_RATIO = 0.891 * 360.0; // output/input

  private TalonFX hoodKraken = new TalonFX(Constants.HoodConstants.k_hoodMotorId, Constants.ctreCANBus);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(120.0, 0, 0,
          DegreesPerSecond.of(80),
          DegreesPerSecondPerSecond.of(160))
      .withFeedforward(new SimpleMotorFeedforward(0.0, 0.0, 0.0))
      .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(GEAR_RATIO)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withSoftLimit(MIN_ANGLE, MAX_ANGLE)
      .withStatorCurrentLimit(Amps.of(10.0))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

  private SmartMotorController smc = new TalonFXWrapper(hoodKraken, DCMotor.getKrakenX44(1), smcConfig);

  private final PivotConfig hoodConfig = new PivotConfig(smc)
      .withMOI(Inches.of(6), Pounds.of(1))
      .withStartingPosition(MAX_ANGLE)
      .withHardLimit(MIN_ANGLE, MAX_ANGLE)
      .withTelemetry("Hood", TelemetryVerbosity.HIGH);

  private Pivot hood = new Pivot(hoodConfig);

  public HoodSubsystem() {
    // YAMS Pivot bug workaround: the Pivot constructor creates a new DCMotorSim
    // at 0 radians and overwrites the SimSupplier, but never initializes the
    // DCMotorSim position to match withStartingPosition(). This causes the sim
    // to think the hood is at 0° instead of MIN_ANGLE on the first simIterate(),
    // resulting in a huge PID correction and runaway oscillation.
    if (Robot.isSimulation()) {
      smc.getSimSupplier().ifPresent(sim -> sim.setMechanismPosition(MAX_ANGLE));
    }
  }

  /**
   * Homes the hood by slowly driving it downward until it bottoms out
   * against the hard stop, then resets the encoder position to 80 degrees.
   * Soft limits are temporarily disabled during homing.
   */
  public Command homeSequence() {
    final double homingDutyCycle = 0.20;
    final double stallVelocityThreshold = 0.01; // motor rot/s

    if (Robot.isSimulation()) {
      // In simulation, YAMS already initializes the hood at the starting position
      // (MIN_ANGLE) via withStartingPosition(). No homing needed — just command
      // the hood to the starting angle so the closed-loop controller is active.
      return setAngle(MIN_ANGLE).withName("Hood.HomeSim");
    }

    return Commands.sequence(
        Commands.runOnce(() -> {
          hoodKraken.getConfigurator().apply(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(false)
                  .withReverseSoftLimitEnable(false));
        }),
        Commands.run(() -> smc.setDutyCycle(homingDutyCycle), this)
            .withTimeout(0.5),
        Commands.run(() -> smc.setDutyCycle(homingDutyCycle), this)
            .until(() -> Math.abs(hoodKraken.getVelocity().getValueAsDouble()) < stallVelocityThreshold)
            .withTimeout(5.0),
        Commands.runOnce(() -> {
          smc.setDutyCycle(0);
          hoodKraken.setPosition(MAX_ANGLE.in(Rotations));
        }, this),
        Commands.runOnce(() -> {
          // TODO: Add this back... please
          // hoodKraken.getConfigurator().apply(
          // new SoftwareLimitSwitchConfigs()
          // .withForwardSoftLimitEnable(true)
          // .withForwardSoftLimitThreshold(getScaledLimit(MAX_ANGLE))
          // .withReverseSoftLimitEnable(true)
          // .withReverseSoftLimitThreshold(getScaledLimit(MIN_ANGLE)));
        })).withName("Hood.Home");
  }

  public double getScaledLimit(Angle angle) {
    return angle.in(Rotations) * GEAR_RATIO;
  }

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
  }

  @Override
  public void simulationPeriodic() {
    // Save battery voltage before simIterate — YAMS Pivot.simIterate() overwrites
    // RoboRioSim.setVInVoltage() based on this motor's current alone, which
    // corrupts the global voltage that MapleSim's SimulatedBattery manages.
    double voltage = RoboRioSim.getVInVoltage();
    hood.simIterate();
    RoboRioSim.setVInVoltage(voltage);
  }
}
