package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase {

  private static final double INTAKE_ROLLER_POWER = 0.75;

  private SparkMax pivotLeaderSpark = new SparkMax(IntakeConstants.k_pivotPrimaryMotorId, MotorType.kBrushless);
  private SparkMax pivotSecondaySpark = new SparkMax(IntakeConstants.k_pivotSecondaryMotorId, MotorType.kBrushless);

  private static final double PIVOT_GEAR_RATIO = 5 * (60.0 / 30.0);

  private final SmartMotorControllerConfig pivotSmcConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(pivotSecondaySpark, true))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(7.5, 0, 0)
      // .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
      .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(PIVOT_GEAR_RATIO)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(30.0));

  private final SmartMotorController pivotSmc = new SparkWrapper(
      pivotLeaderSpark,
      DCMotor.getNeoVortex(1),
      pivotSmcConfig);

  private final ArmConfig intakePivotConfig = new ArmConfig(pivotSmc)
      .withSoftLimits(Degrees.of(0), Degrees.of(150))
      .withHardLimit(Degrees.of(0), Degrees.of(155))
      .withStartingPosition(Degrees.of(0))
      .withLength(Feet.of(1))
      .withMass(Pounds.of(2))
      .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

  private Arm intakePivot = new Arm(intakePivotConfig);

  // Nova motor controller with NEO motor
  private SparkFlex rollerSpark = new SparkFlex(Constants.IntakeConstants.k_rollerMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig rollerSmcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.0300, 0, 0)
      .withTelemetry("RollerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) // no gear reduction
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST);
  // .withStatorCurrentLimit(Amps.of(60));

  private SmartMotorController rollerSmc = new SparkWrapper(rollerSpark, DCMotor.getNeoVortex(1), rollerSmcConfig);

  private final FlyWheelConfig rollerConfig = new FlyWheelConfig(rollerSmc)
      .withDiameter(Inches.of(2))
      .withMass(Pounds.of(0.5))
      // .withUpperSoftLimit(RPM.of(10000))
      // .withLowerSoftLimit(RPM.of(-10000))
      .withTelemetry("roller", TelemetryVerbosity.HIGH);

  private FlyWheel roller = new FlyWheel(rollerConfig);

  public IntakeSubsystem() {
    this.setDefaultCommand(Commands.runOnce(() -> rollerSmc.setDutyCycle(0), this));
  }

  public Command intakeCommand() {
    return roller.setSpeed(RPM.of(3000.0)).withName("Intake.IntakeCommand");
  }

  public Command setPivotAngle(Angle angle) {
    return Commands.runOnce(() -> intakePivot.setAngle(angle),
        this).withName("IntakePivot.SetAngle");
  }

  public Command rezero() {
    return Commands.runOnce(() -> pivotLeaderSpark.getEncoder().setPosition(0), this).withName("IntakePivot.Rezero");
  }

  public Command setIntakeStow() {
    return intakePivot.setAngle(IntakeConstants.k_IntakeStow);
  }

  public Command setIntakeDeployed() {
    return intakePivot.setAngle(IntakeConstants.k_IntakeDeployed);
  }

  @Override
  public void periodic() {
    rollerSmc.updateTelemetry();
    pivotSmc.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    rollerSmc.simIterate();
    pivotSmc.simIterate();
  }
}
