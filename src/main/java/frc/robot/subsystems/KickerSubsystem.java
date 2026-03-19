package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
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

public class KickerSubsystem extends SubsystemBase {
  private static final AngularVelocity KICKER_RPM = RPM.of(6000);

  private SparkFlex kickerSpark = new SparkFlex(Constants.KickerConstants.kKickerMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.0205, 0, 0)
      .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) // no gear reduction
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST);
  // .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController smc = new SparkWrapper(kickerSpark, DCMotor.getNeoVortex(1), smcConfig);

  private final FlyWheelConfig kickerConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("Kicker", TelemetryVerbosity.HIGH);

  private FlyWheel kicker = new FlyWheel(kickerConfig);

  public KickerSubsystem() {
    this.setDefaultCommand(Commands.run(() -> smc.setDutyCycle(0), this));
  }

  public Command feedCommand() {
    return kicker.setSpeed(KICKER_RPM).withName("Kicker.Feed");
  }

  public Command ejectCommand() {
    return kicker.setSpeed(KICKER_RPM.unaryMinus()).withName("Kicker.Eject");
  }

  @Override
  public void periodic() {
    kicker.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    double voltage = RoboRioSim.getVInVoltage();
    kicker.simIterate();
    RoboRioSim.setVInVoltage(voltage);
  }
}
