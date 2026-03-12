package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.wrappers.REVThroughBoreEncoder;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.MechanismPositionConfig.Plane;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class TurretSubsystem extends SubsystemBase {

  private final double MAX_ONE_DIR_FOV = 45; // degrees

  private static final double M12_OFFSET = 0.951269;
  private static final double M13_OFFSET = 0.306114;
  // reading

  // TODO: check these for the real bot; these are just estimates
  public final Translation3d turretTranslation = new Translation3d(-0.205, 0.0, 0.375);

  // 1 Neo, 5:1 gearbox, 60:12 pivot gearing, non-continuous 360 deg
  // Total reduction: 5 * 5 = 25:1
  public final double GEAR_RATIO = 5.0 * (60.0 / 12.0);

  private SparkMax turretSpark = new SparkMax(Constants.TurretConstants.k_turretMotorId, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(2.5, 0, 0,
          DegreesPerSecond.of(2440 * GEAR_RATIO),
          DegreesPerSecondPerSecond.of(2440 * GEAR_RATIO))
      // .withFeedforward(new ArmFeedforward(0.0, 0.0, 7.5))
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(GEAR_RATIO)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      // .withSoftLimit(Degrees.of(-MAX_ONE_DIR_FOV), Degrees.of(MAX_ONE_DIR_FOV))
      .withStatorCurrentLimit(Amps.of(60))
      .withSupplyCurrentLimit(Amps.of(60))
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

  private SmartMotorController smc = new SparkWrapper(turretSpark, DCMotor.getNEO(1), smcConfig);

  private final PivotConfig turretConfig = new PivotConfig(smc)
      // .withHardLimit(Degrees.of(-MAX_ONE_DIR_FOV - 5), Degrees.of(MAX_ONE_DIR_FOV +
      // 5))
      // .withStartingPosition(Degrees.of(0))
      // .withMOI(0.05)
      // .withWrapping(Degrees.of(0), Degrees.of(360))
      .withTelemetry("Turret", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(
          new MechanismPositionConfig().withMovementPlane(Plane.XY).withRelativePosition(turretTranslation));

  private Pivot turret = new Pivot(turretConfig);

  private boolean isRezeroed = false;

  // Absolute encoders
  private final REVThroughBoreEncoder m12TAbsEncoder;
  private final REVThroughBoreEncoder m13TAbsEncoder;

  public TurretSubsystem() {
    m12TAbsEncoder = new REVThroughBoreEncoder(1);
    m13TAbsEncoder = new REVThroughBoreEncoder(0);

    // TODO: Actually make this work at some point, PLEASE
    // new Trigger(() -> shouldRezero()).onTrue(rezero().ignoringDisable(true));
  }

  // private boolean shouldRezero() {
  // // Rezero if both encoders are connected and we haven't already rezeroed
  // return (m12TAbsEncoder.isConnected() && m13TAbsEncoder.isConnected()) ||
  // !isRezeroed;
  // }

  public Command rezero() {
    return Commands.runOnce(() -> {
      Angle turretAngle = computeTurretAngleFromAbs();

      System.out.println("==============================");
      System.out.println("====== Rezeroing turret ======");
      System.out.println("==============================");
      System.out.println("Setting Offset: " + turretAngle.in(Degrees) + " deg (" + turretAngle.in(Rotations) + " rot)");

      turretSpark.getEncoder().setPosition(turretAngle.in(Rotations));
      turretConfig.withStartingPosition(turretAngle);

      isRezeroed = true;
    }, this)
        .ignoringDisable(true)
        .withName("Turret.Rezero");
  }

  @AutoLogOutput(key = "Turret/m12TAbsAngleWithOffset")
  private double getM12TAbsAngleWithOffset() {
    return (getM12TAbsAngle() - M12_OFFSET + 1.0) % 1.0;
  }

  @AutoLogOutput(key = "Turret/m13TAbsAngleWithOffset")
  private double getM13TAbsAngleWithOffset() {
    return (getM13TAbsAngle() - M13_OFFSET + 1.0) % 1.0;
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

    // // Wrap from [0, 360) to [-180, 180) so negative turret positions are
    // // represented correctly
    if (turretAngleDeg > 180.0) {
      turretAngleDeg -= 360.0;
    }

    Logger.recordOutput("Turret/CRT/colF", colF);
    Logger.recordOutput("Turret/CRT/colG", colG);
    Logger.recordOutput("Turret/CRT/turretAngleDeg", turretAngleDeg);

    return Degrees.of(turretAngleDeg);
  }

  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier) {
    return turret.setAngle(turretAngleSupplier);
  }

  public Command center() {
    return turret.setAngle(Degrees.of(0));
  }

  public Angle getRobotAdjustedAngle() {
    // Returns the turret angle in the robot's coordinate frame
    // since the turret is mounted backwards, we need to add 180 degrees
    return turret.getAngle().plus(Degrees.of(180));
  }

  public Angle getAngle() {
    return turret.getAngle();
  }

  public Command set(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  public Command sysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  @Override
  public void periodic() {
    turret.updateTelemetry();

    if (!isRezeroed && m12TAbsEncoder.isConnected() && m13TAbsEncoder.isConnected()) {
      CommandScheduler.getInstance().schedule(rezero());
    }

    Logger.recordOutput("Turret/isRezeroed", isRezeroed);
    Logger.recordOutput("Turret/m12TAbsEncoderConnected", m12TAbsEncoder.isConnected());
    Logger.recordOutput("Turret/m13TAbsEncoderConnected", m13TAbsEncoder.isConnected());

    // rezero().schedule();

    Logger.recordOutput("Turret/RelYamsAngle", getAngle());
    Logger.recordOutput("Turret/computeTurretAngleFromAbs", computeTurretAngleFromAbs());

    // Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
    // new Pose3d(
    // turretTranslation,
    // new Rotation3d(0, 0, turret.getAngle().in(Radians)))
    // });
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}
