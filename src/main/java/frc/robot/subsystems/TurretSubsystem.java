package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.wrappers.RARSparkWrapper;
import frc.robot.wrappers.REVThroughBoreEncoder;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class TurretSubsystem extends SubsystemBase {
  // 1 Neo, 5:1 gearbox, 60:12 pivot gearing, non-continuous 360 deg
  // Total reduction: 5 * 5 = 25:1
  public final double GEAR_RATIO = (5.0 * (60.0 / 12.0));

  public final Angle MAX_ANGLE = TurretConstants.MAX_ONE_DIR_FOV;
  public final Angle MIN_ANGLE = TurretConstants.MAX_ONE_DIR_FOV.unaryMinus();

  // Motor & encoder
  private SparkMax turretSpark = new SparkMax(Constants.TurretConstants.k_turretMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // .withClosedLoopController(20.0, 0.0, 0.0)
      .withClosedLoopController(1.0, 0.0, 0.0,
          DegreesPerSecond.of(45),
          DegreesPerSecondPerSecond.of(45))
      .withFeedforward(new SimpleMotorFeedforward(0.0, 1.0, 0.0))
      .withClosedLoopTolerance(Degrees.of(360.0))
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(GEAR_RATIO)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withSoftLimit(MIN_ANGLE, MAX_ANGLE)
      .withStatorCurrentLimit(Amps.of(15.0));
  // .withClosedLoopRampRate(Seconds.of(0.1))
  // .withOpenLoopRampRate(Seconds.of(0.1));

  private SmartMotorController smc = new RARSparkWrapper(turretSpark, DCMotor.getNEO(1), smcConfig);

  private final PivotConfig turretConfig = new PivotConfig(smc)
      .withMOI(Inches.of(6), Pounds.of(1))
      // .withStartingPosition(Degrees.of(0))
      // .withWrapping(Degrees.of(0), Degrees.of(360))
      // .withSoftLimits(MIN_ANGLE, MAX_ANGLE)
      // .withHardLimit(MIN_ANGLE.minus(Degrees.of(5)), MAX_ANGLE.plus(Degrees.of(5)))
      .withTelemetry("Turret", TelemetryVerbosity.HIGH);

  private Pivot turret = new Pivot(turretConfig);

  private boolean isRezeroed = false;
  private boolean isRezeroing = false;

  // Absolute encoders
  private final REVThroughBoreEncoder m12TAbsEncoder;
  private final REVThroughBoreEncoder m13TAbsEncoder;

  // public final Trigger isAtTarget = new Trigger(() -> profiledPID.atGoal());
  public final Trigger isAtTarget = new Trigger(
      () -> turret.getMechanismSetpoint().orElse(Degrees.of(0)).minus(turret.getAngle())
          .abs(Degrees) < SuperstructureConstants.k_turretTolerance.in(Degrees));

  public TurretSubsystem() {
    m12TAbsEncoder = new REVThroughBoreEncoder(1, TurretConstants.m12Frequency);
    m13TAbsEncoder = new REVThroughBoreEncoder(0, TurretConstants.m13Frequency);
  }

  public Command rezero() {
    final ArrayList<Double> samples = new ArrayList<>();
    double averageTime = 1.0;

    return Commands.sequence(
        Commands.runOnce(() -> {
          isRezeroing = true;
          samples.clear();
          System.out.println("==============================");
          System.out.println("== Rezeroing turret (1s avg) =");
          System.out.println("==============================");
        }),
        Commands.run(() -> {
          samples.add(computeTurretAngleFromAbs().in(Rotations));
        }, this).withTimeout(averageTime),
        Commands.runOnce(() -> {
          double avgRotations = samples.stream()
              .mapToDouble(Double::doubleValue)
              .average()
              .orElse(0.0);
          Angle turretAngle = Rotations.of(avgRotations);

          System.out.println("Averaged " + samples.size() + " samples over 1 second(s)");
          System.out.println(
              "Setting Offset: " + turretAngle.in(Degrees) + " deg (" + turretAngle.in(Rotations) + " rot)");

          // Set encoder position (conversion factor is already applied)
          // turretEncoder.setPosition(avgRotations);
          turret.getMotorController().setEncoderPosition(turretAngle);
          // profiledPID.reset(avgRotations); //TODO: this? Maybe?

          isRezeroed = true;
        }, this))
        .finallyDo(() -> isRezeroing = false)
        .ignoringDisable(true)
        .withName("Turret.Rezero");
  }

  @AutoLogOutput(key = "Turret/m12TAbsAngleWithOffset")
  private double getM12TAbsAngleWithOffset() {
    return (getM12TAbsAngle() - TurretConstants.M12_OFFSET + 1.0) % 1.0;
  }

  @AutoLogOutput(key = "Turret/m13TAbsAngleWithOffset")
  private double getM13TAbsAngleWithOffset() {
    return (getM13TAbsAngle() - TurretConstants.M13_OFFSET + 1.0) % 1.0;
  }

  @AutoLogOutput(key = "Turret/m12TAbsAngle")
  private double getM12TAbsAngle() {
    return m12TAbsEncoder.get();
  }

  @AutoLogOutput(key = "Turret/m13TAbsAngle")
  private double getM13TAbsAngle() {
    return m13TAbsEncoder.get();
  }

  private Angle computeTurretAngleFromAbs() {
    double e1 = getM12TAbsAngleWithOffset();
    double e2 = getM13TAbsAngleWithOffset();

    double colE = e1 - e2;
    double colF = ((colE + 1.5) % 1.0) - 0.5;
    double colG = ((colF * 2.6) + 2.0) % 1.0; // 2.6 = (12/60) * (13/60) / ((12/60) - (13/60))
    double turretAngleDeg = colG * 360.0;

    // Wrap from [0, 360) to [-180, 180) so negative turret positions are
    // represented correctly
    if (turretAngleDeg > 180.0) {
      turretAngleDeg -= 360.0;
    }

    Logger.recordOutput("Turret/CRT/colF", colF);
    Logger.recordOutput("Turret/CRT/colG", colG);
    Logger.recordOutput("Turret/CRT/turretAngleDeg", turretAngleDeg);

    return Degrees.of(turretAngleDeg);
  }

  public Translation3d getTurretTranslation() {
    return TurretConstants.turretTranslation;
  }

  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);

    // return turret.setAngle(clampSafeAngle(() -> angle));
    // return run(() -> {
    // // Angle adjustedAngle =
    // // clampSafeAngle(angle.plus(getAngle().minus(computeTurretAngleFromAbs())));

    // Angle adjustedAngle = clampSafeAngle(angle);
    // CommandScheduler.getInstance().schedule(turret.setAngle(adjustedAngle));
    // }).withName("Turret.SetAngle(" + angle.in(Degrees) + " deg)");
  }

  public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier) {
    // return Commands.run(() -> {
    // System.out.println(turretAngleSupplier.get().in(Degrees));
    // });

    return turret.setAngle(turretAngleSupplier);
    // return turret.setAngle(clampSafeAngle(turretAngleSupplier));
    // return Commands.runOnce(() -> {
    // Angle adjustedAngle = clampSafeAngle(turretAngleSupplier.get());
    // turret.setAngle(adjustedAngle).schedule();
    // }).withName("Turret.SetAngleDynamic");
  }

  public Supplier<Angle> clampSafeAngle(Supplier<Angle> angle) {
    Angle actual = angle.get();
    final Angle result;

    if (actual.gt(MAX_ANGLE)) {
      result = MAX_ANGLE;
    } else if (actual.lt(MIN_ANGLE)) {
      result = MIN_ANGLE;
    } else {
      result = actual;
    }

    return () -> result;
  }

  public Command center() {
    return setAngle(Degrees.of(0));
  }

  public Command sysid() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  public Angle getRobotAdjustedAngle() {
    // Returns the turret angle in the robot's coordinate frame
    // since the turret is mounted backwards, we need to add 180 degrees
    return getAngle().plus(Degrees.of(180));
  }

  public Angle getAngle() {
    return turret.getAngle();
  }

  @Override
  public void periodic() {
    // Auto-rezero when absolute encoders become available
    if (!isRezeroed && !isRezeroing && m12TAbsEncoder.isConnected() && m13TAbsEncoder.isConnected()) {
      CommandScheduler.getInstance().schedule(rezero());
    }

    // Telemetry
    Logger.recordOutput("Turret/error",
        turret.getMechanismSetpoint().orElse(Degrees.of(0)).minus(turret.getAngle()).in(Degrees), Degrees);
    Logger.recordOutput("Turret/isRezeroed", isRezeroed);
    Logger.recordOutput("Turret/m12TAbsEncoderConnected", m12TAbsEncoder.isConnected());
    Logger.recordOutput("Turret/m13TAbsEncoderConnected", m13TAbsEncoder.isConnected());

    Logger.recordOutput("Turret/PositionRots",
        turret.getMotorController().getMechanismPosition().in(Degrees), Degrees);
    Logger.recordOutput("Turret/VelocityRotsPerSec",
        turret.getMotorController().getMechanismVelocity().in(DegreesPerSecond), DegreesPerSecond);
    Logger.recordOutput("Turret/SetpointRots",
        turret.getMechanismSetpoint().orElse(Degrees.of(0)).in(Degrees), Degrees);
    Logger.recordOutput("Turret/computeTurretAngleFromAbs", computeTurretAngleFromAbs().in(Degrees), Degrees);

    // MaxMotion
    Logger.recordOutput("Turret/MaxMotion/PositionGoalRots",
        turretSpark.getClosedLoopController().getSetpoint(), Rotations);
    Logger.recordOutput("Turret/MaxMotion/PositionSetpointRots",
        turretSpark.getClosedLoopController().getMAXMotionSetpointPosition(), Rotations);
    Logger.recordOutput("Turret/MaxMotion/PositionSetpointVelocity",
        turretSpark.getClosedLoopController().getMAXMotionSetpointVelocity(), RotationsPerSecond);
    Logger.recordOutput("Turret/MaxMotion/controlType",
        turretSpark.getClosedLoopController().getControlType());

    // Abs encoders
    Logger.recordOutput("Turret/frequency/m12", m12TAbsEncoder.getFrequency());
    Logger.recordOutput("Turret/frequency/m13", m13TAbsEncoder.getFrequency());

    // Turret mechanism
    Logger.recordOutput("TURRET-MECH/Angle", turret.getAngle().in(Degrees), Degrees);
    Logger.recordOutput("TURRET-MECH/Setpoint", turret.getMechanismSetpoint().orElse(Degrees.of(0)).in(Degrees),
        Degrees);

    Logger.recordOutput("Turret/outputVoltage", turret.getMotorController().getVoltage().in(Volts), Volts);
  }

  @Override
  public void simulationPeriodic() {
    // No YAMS sim — add simulation model here if needed
  }
}
