package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
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

public class IntakeSubsystem extends SubsystemBase {

  private static final double INTAKE_SPEED = 0.75;

  // private SparkMax pivotMax = new SparkMax(IntakeConstants.k_pivotMotorId,
  // MotorType.kBrushless);
  // // simple roller + pivot implementation (no Yams)
  private static final double PIVOT_GEAR_RATIO = 5 * 5 * (60.0 / 15.0); // 5:1 * 5:1 * 60/15

  // private final SmartMotorControllerConfig smcConfig = new
  // SmartMotorControllerConfig(this)
  // .withFollowers(Pair.of(pivotMotor, true))
  // .withControlMode(ControlMode.CLOSED_LOOP)
  // .withClosedLoopController(0.00936, 0, 0)
  // .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
  // .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
  // .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
  // .withMotorInverted(false)
  // .withIdleMode(MotorMode.COAST)
  // .withStatorCurrentLimit(Amps.of(40));

  // Nova motor controller with NEO motor
  private SparkFlex rollerSpark = new SparkFlex(Constants.IntakeConstants.k_rollerMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("RollerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) // no gear reduction
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController rollerSmc = new SparkWrapper(rollerSpark, DCMotor.getNeoVortex(1), smcConfig);

  private final FlyWheelConfig rollerConfig = new FlyWheelConfig(rollerSmc)
      .withDiameter(Inches.of(2))
      .withMass(Pounds.of(0.5))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(-6000))
      .withTelemetry("roller", TelemetryVerbosity.HIGH);

  private FlyWheel roller = new FlyWheel(rollerConfig);

  public IntakeSubsystem() {
    // pivotMotor.factoryReset();
    this.setDefaultCommand(Commands.runOnce(() -> rollerSmc.setDutyCycle(0), this));
  }

  public Command intakeCommand() {
    return roller.set(INTAKE_SPEED);
  }

  // /**
  // * Command to eject while held.
  // */
  // public Command ejectCommand() {
  // return intake.set(-INTAKE_SPEED).finallyDo(() ->
  // smc.setDutyCycle(0)).withName("Intake.Eject");
  // }

  // public Command setPivotAngle(Angle angle) {
  // return Commands.runOnce(() -> setPivotPosition(angle),
  // this).withName("IntakePivot.SetAngle");
  // }

  // public Command rezero() {
  // return Commands.runOnce(() -> pivotMax.getEncoder().setPosition(0),
  // this).withName("IntakePivot.Rezero");
  // }

  // private void setIntakeStow() {
  // setPivotPosition(IntakeConstants.k_IntakeStow);
  // }

  // private void setIntakeFeed() {
  // setPivotPosition(IntakeConstants.k_IntakeFeed);
  // }

  // private void setIntakeHold() {
  // setPivotPosition(IntakeConstants.k_IntakeHold);
  // }

  // private void setIntakeDeployed() {
  // setPivotPosition(IntakeConstants.k_IntakeDeployed);
  // }

  // private void setPivotPosition(Angle angle) {
  // // Convert joint degrees to motor rotations, accounting for gear ratio.
  // double jointDegrees = angle.in(Degrees);
  // double motorRotations = (jointDegrees / 360.0) * PIVOT_GEAR_RATIO;
  // // Use SparkMax PID position control (rotations)
  // try {
  // // pivotMax.getPIDController().setReference(motorRotations,
  // // ControlType.kPosition);
  // pivotMax.getEncoder().setPosition(motorRotations);
  // } catch (Exception e) {
  // // If the concrete SparkMax PID API isn't available in this environment,
  // // fallback to open-loop set
  // // This is defensive: replace with appropriate API calls if needed.
  // // No-op or log
  // }
  // }

  @Override
  public void periodic() {
    // Telemetry: pivot encoder position could be sampled here if desired
  }

  @Override
  public void simulationPeriodic() {
    // Simulation hooks removed (Yams-specific)
  }
}
