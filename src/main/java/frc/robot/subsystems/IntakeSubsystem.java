package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

  private static final AngularVelocity INTAKE_ROLLER_SPEED = RPM.of(2000.0);
  private static final AngularVelocity EJECT_ROLLER_SPEED = RPM.of(3000.0);

  private SparkMax pivotLeaderSpark = new SparkMax(IntakeConstants.k_pivotPrimaryMotorId, MotorType.kBrushless);
  // private SparkMax pivotSecondaySpark = new
  // SparkMax(IntakeConstants.k_pivotSecondaryMotorId, MotorType.kBrushless);

  private final REVThroughBoreEncoder pivotAbsEncoder;

  private static final double PIVOT_GEAR_RATIO = 9.0;

  private final SmartMotorControllerConfig pivotSmcConfig = new SmartMotorControllerConfig(this)
      // .withFollowers(Pair.of(pivotSecondaySpark, true))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(1.0, 0, 0)
      // .withFeedforward(new ArmFeedforward(0.191, 0.11858, 0.0))
      .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(PIVOT_GEAR_RATIO)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(5.0));

  private final SmartMotorController pivotSmc = new SparkWrapper(
      pivotLeaderSpark,
      DCMotor.getNEO(1),
      pivotSmcConfig);

  private final ArmConfig intakePivotConfig = new ArmConfig(pivotSmc)
      // .withSoftLimits(Degrees.of(0), Degrees.of(1320))
      // .withHardLimit(Degrees.of(0), Degrees.of(1320))
      .withStartingPosition(Degrees.of(0))
      .withLength(Feet.of(1))
      .withMass(Pounds.of(2))
      .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

  private Arm intakePivot = new Arm(intakePivotConfig);

  // Nova motor controller with NEO motor
  private SparkFlex rollerSpark = new SparkFlex(Constants.IntakeConstants.k_rollerMotorId, MotorType.kBrushless);
  private SparkFlex rollerSecondarySpark = new SparkFlex(Constants.IntakeConstants.k_rollerMotorSecondaryId,
      MotorType.kBrushless);

  private SmartMotorControllerConfig rollerSmcConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(rollerSecondarySpark, true))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.0100, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.191, 0.09558, 0.0))
      .withTelemetry("RollerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) // no gear reduction
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST);
  // .withStatorCurrentLimit(Amps.of(40));

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

    pivotAbsEncoder = new REVThroughBoreEncoder(2);
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
    return Commands.sequence(
        intakePivot.setAngle(IntakeConstants.k_IntakeDeployed)
            .until(intakePivot.isNear(IntakeConstants.k_IntakeDeployed, Degrees.of(3))),
        roller.setSpeed(INTAKE_ROLLER_SPEED))
        .withName("Intake.intakeDeployAndRun");
  }

  public Command feedCommand() {
    // TODO: maybe replace this with setIntakeJostle()

    return setIntakeFeedPivot();
  }

  public Command stopCommand() {
    return roller.setSpeed(RPM.of(0)).withName("Intake.StopCommand");
  }

  public Command ejectCommand() {
    return roller.setSpeed(EJECT_ROLLER_SPEED.unaryMinus()).withName("Intake.EjectCommand");
  }

  public Command setPivotAngle(Angle angle) {
    return Commands.runOnce(() -> intakePivot.setAngle(angle),
        this).withName("IntakePivot.SetAngle");
  }

  public Command rezero() {
    return Commands.runOnce(() -> seedRelativeEncoderFromAbsolute(), this).withName("IntakePivot.Rezero");
  }

  public Command homeSequence() {
    final double homingDutyCycle = -0.50;
    final double stallVelocityThreshold = 0.01; // motor rot/s

    return Commands.sequence(
        // Commands.runOnce(() -> {
        // SparkMaxConfig config = new SparkMaxConfig();
        // // config.configure(pivotLeaderSpark.configAccessor);
        // config.apply(pivotLeaderSpark.configAccessor.absoluteEncoder);

        // config.softLimit
        // .forwardSoftLimitEnabled(false)
        // .reverseSoftLimitEnabled(false);

        // pivotLeaderSpark.configure(config, ResetMode.kNoResetSafeParameters,
        // PersistMode.kNoPersistParameters);
        // }),
        Commands.run(() -> pivotSmc.setDutyCycle(homingDutyCycle), this)
            .withTimeout(0.5),
        Commands.run(() -> pivotSmc.setDutyCycle(homingDutyCycle), this)
            .until(() -> Math.abs(pivotSmc.getRotorVelocity().in(RPM)) < stallVelocityThreshold)
            .withTimeout(10.0),
        Commands.runOnce(() -> {
          pivotSmc.setDutyCycle(0);
          pivotSmc.setEncoderPosition(Rotations.of(0));
          // intakePivot.setPosition(Rotations.of(0));
        }, this),
        Commands.runOnce(() -> {
          // hood.getMotorController().setStatorCurrentLimit(RUNNING_CURRENT);

          // TODO: Add this back... please
          // hoodKraken.getConfigurator().apply(
          // new SoftwareLimitSwitchConfigs()
          // .withForwardSoftLimitEnable(true)
          // .withForwardSoftLimitThreshold(getScaledLimit(MAX_ANGLE))
          // .withReverseSoftLimitEnable(true)
          // .withReverseSoftLimitThreshold(getScaledLimit(MIN_ANGLE)));
        })).withName("Hood.Home").withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  public Command setIntakeStow() {
    return intakePivot.setAngle(IntakeConstants.k_IntakeMaxWhileRoller);
    // return intakePivot.setAngle(IntakeConstants.k_IntakeStow);
  }

  public Command setIntakeJostle() {
    return Commands.sequence(
        intakePivot.setAngle(IntakeConstants.k_IntakeMaxWhileRoller).raceWith(Commands.waitSeconds(0.75)),
        intakePivot.setAngle(IntakeConstants.k_IntakeDeployed).raceWith(Commands.waitSeconds(2.0)))
        .repeatedly()
        .withName("Intake.setIntakeJostle");
  }

  public Command setIntakeFeedPivot() {
    return intakePivot.setAngle(IntakeConstants.k_IntakeMaxWhileRoller);
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
    // seedRelativeEncoderFromAbsolute();

    rollerSmc.updateTelemetry();
    pivotSmc.updateTelemetry();

    // if (isDeployStalled()) {
    // if (intakePivot.getMechanismSetpoint().get() ==
    // IntakeConstants.k_IntakeDeployed) {
    // hammerTime = true;
    // setIntakeStow().schedule();
    // }
    // }

    // if (hammerTime && intakePivot.isNear(IntakeConstants.k_IntakeStow,
    // Degrees.of(5)).getAsBoolean()) {
    // hammerTime = false;
    // setIntakeDeployed().schedule();
    // }
  }

  @Override
  public void simulationPeriodic() {
    rollerSmc.simIterate();
    pivotSmc.simIterate();
  }
}
