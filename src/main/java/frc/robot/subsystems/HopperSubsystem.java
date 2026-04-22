package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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
import yams.motorcontrollers.local.SparkWrapper;

public class HopperSubsystem extends SubsystemBase {

  private static final AngularVelocity HOPPER_RPM = RPM.of(4000);

  // Nova motor controller with NEO motor
  private SparkMax hopperSpark = new SparkMax(Constants.HopperConstants.kHopperMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.055, 0, 0)
      .withTelemetry("HopperMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(4))) // no gear reduction
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST);
  // .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController smc = new SparkWrapper(hopperSpark, DCMotor.getNeoVortex(1), smcConfig);

  private final FlyWheelConfig hopperConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(2))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("Hopper", TelemetryVerbosity.HIGH);

  private FlyWheel hopper = new FlyWheel(hopperConfig);

  public HopperSubsystem() {
    this.setDefaultCommand(Commands.run(() -> smc.setDutyCycle(0), this));
    // this.setDefaultCommand(Commands.run(() -> hopper.setSpeed(RPM.of(0)), this));
  }

  Supplier<AngularVelocity> hopperSpeedSupplier = () -> {
    return RPM.of(Double.valueOf(DriverStation.getGameSpecificMessage()));
  };

  public Command feedCommand() {
    return hopper.setSpeed(hopperSpeedSupplier).withName("Hopper.Feed");
    // return hopper.setSpeed(HOPPER_RPM).withName("Hopper.Feed");
  }

  public Command ejectCommand() {
    return hopper.setSpeed(HOPPER_RPM.unaryMinus()).withName("Hopper.Eject");
  }

  @Override
  public void periodic() {
    hopper.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    double voltage = RoboRioSim.getVInVoltage();
    hopper.simIterate();
    RoboRioSim.setVInVoltage(voltage);
  }
}
