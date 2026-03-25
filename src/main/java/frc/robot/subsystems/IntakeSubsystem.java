package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.wrappers.REVThroughBoreEncoder;
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

  private static final AngularVelocity INTAKE_ROLLER_SPEED = RPM.of(3000.0);

  private SparkMax pivotLeaderSpark = new SparkMax(IntakeConstants.k_pivotPrimaryMotorId, MotorType.kBrushless);
  private SparkMax pivotSecondaySpark = new SparkMax(IntakeConstants.k_pivotSecondaryMotorId, MotorType.kBrushless);

  private final REVThroughBoreEncoder pivotAbsEncoder;

  private static final double PIVOT_GEAR_RATIO = 9.0 * (48.0 / 22.0);

  private final SmartMotorControllerConfig pivotSmcConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(pivotSecondaySpark, true))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(2.0, 0, 0)
      // .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
      .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(PIVOT_GEAR_RATIO)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE);
  // .withStatorCurrentLimit(Amps.of(40.0));

  private final SmartMotorController pivotSmc = new SparkWrapper(
      pivotLeaderSpark,
      DCMotor.getNEO(2),
      pivotSmcConfig);

  private final ArmConfig intakePivotConfig = new ArmConfig(pivotSmc)
      .withSoftLimits(Degrees.of(0), Degrees.of(120))
      .withHardLimit(Degrees.of(0), Degrees.of(125))
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
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController rollerSmc = new SparkWrapper(rollerSpark, DCMotor.getNeoVortex(1), rollerSmcConfig);

  private final FlyWheelConfig rollerConfig = new FlyWheelConfig(rollerSmc)
      .withDiameter(Inches.of(2))
      .withMass(Pounds.of(0.5))
      // .withUpperSoftLimit(RPM.of(10000))
      // .withLowerSoftLimit(RPM.of(-10000))
      .withTelemetry("roller", TelemetryVerbosity.HIGH);

  private FlyWheel roller = new FlyWheel(rollerConfig);

  /** Timer tracking how long pivot current has been above the stall threshold. */
  private final Timer stallTimer = new Timer();
  private boolean stallTimerRunning = false;

  private boolean hammerTime = false;

  public IntakeSubsystem() {
    this.setDefaultCommand(Commands.runOnce(() -> rollerSmc.setDutyCycle(0), this));
    // this.setDefaultCommand(Commands.run(() -> roller.setSpeed(RPM.of(0)), this));

    pivotAbsEncoder = new REVThroughBoreEncoder(2);

    seedRelativeEncoderFromAbsolute();
  }

  private void seedRelativeEncoderFromAbsolute() {
    double mechanismRotations = getAbsAngleWithOffset();
    pivotLeaderSpark.getEncoder().setPosition(mechanismRotations);
  }

  @AutoLogOutput(key = "Intake/absAngleWithOffset")
  private double getAbsAngleWithOffset() {
    return getAbsAngle() - IntakeConstants.k_pivotAbsEncoderOffset;
  }

  @AutoLogOutput(key = "Intake/absAngle")
  private double getAbsAngle() {
    return pivotAbsEncoder.get();
  }

  public Command intakeCommand() {
    return roller.setSpeed(INTAKE_ROLLER_SPEED).withName("Intake.intakeCommand");
  }

  public Command intakeDeployAndRun() {
    return Commands.parallel(
        roller.setSpeed(INTAKE_ROLLER_SPEED).asProxy(),
        setIntakeDeployed())
        .withName("Intake.intakeDeployAndRun");
  }

  public Command feedCommand() {
    return Commands.parallel(
        setIntakeFeedPivot().asProxy(),
        Commands.sequence(
            roller.setSpeed(INTAKE_ROLLER_SPEED).withTimeout(IntakeConstants.k_feedUpTime),
            roller.setSpeed(RPM.of(0)).withTimeout(IntakeConstants.k_feedDownTime))
            .repeatedly())
        .withName("Intake.feedCommand");
  }

  public Command stopCommand() {
    return roller.setSpeed(RPM.of(0)).withName("Intake.StopCommand");
  }

  public Command ejectCommand() {
    return roller.setSpeed(INTAKE_ROLLER_SPEED.unaryMinus()).withName("Intake.EjectCommand");
  }

  public Command setPivotAngle(Angle angle) {
    return Commands.runOnce(() -> intakePivot.setAngle(angle),
        this).withName("IntakePivot.SetAngle");
  }

  public Command rezero() {
    return Commands.runOnce(() -> seedRelativeEncoderFromAbsolute(), this).withName("IntakePivot.Rezero");
  }

  public Command setIntakeStow() {
    return intakePivot.setAngle(IntakeConstants.k_IntakeStow);
  }

  public Command setIntakeFeedPivot() {
    return intakePivot.setAngle(IntakeConstants.k_IntakeFeed);
  }

  public Command setIntakeDeployed() {
    return intakePivot.setAngle(IntakeConstants.k_IntakeDeployed);
  }

  private boolean isDeployStalled() {
    boolean aboveThreshold = pivotSmc.getStatorCurrent().in(Amps) >= IntakeConstants.k_deployStallCurrentThreshold;

    if (aboveThreshold) {
      if (!stallTimerRunning) {
        stallTimer.restart();
        stallTimerRunning = true;
      }
    } else {
      // Current dropped below threshold — reset the debounce
      stallTimer.stop();
      stallTimer.reset();
      stallTimerRunning = false;
    }

    return stallTimerRunning && stallTimer.hasElapsed(IntakeConstants.k_deployStallDebounce);
  }

  @Override
  public void periodic() {
    seedRelativeEncoderFromAbsolute();

    rollerSmc.updateTelemetry();
    pivotSmc.updateTelemetry();

    if (isDeployStalled()) {
      if (intakePivot.getMechanismSetpoint().get() == IntakeConstants.k_IntakeDeployed) {
        hammerTime = true;
        setIntakeStow().schedule();
      }
    }

    if (hammerTime && intakePivot.isNear(IntakeConstants.k_IntakeStow, Degrees.of(5)).getAsBoolean()) {
      hammerTime = false;
      setIntakeDeployed().schedule();
    }
  }

  @Override
  public void simulationPeriodic() {
    rollerSmc.simIterate();
    pivotSmc.simIterate();
  }
}
