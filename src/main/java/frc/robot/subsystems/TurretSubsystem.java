package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
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
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class TurretSubsystem extends SubsystemBase {

  private final double MAX_ONE_DIR_FOV = 180; // degrees

  // CRT (Chinese Remainder Theorem) absolute encoder constants
  // The two encoders are geared with coprime tooth counts (12 and 13) to the
  // turret,
  // so they wrap a different number of times over the turret's full 360° range.
  // gcd(12, 13) = 1, so CRT uniquely resolves position within lcm(12,13) = 156
  // sectors.
  private static final int RING_GEAR_TEETH = 60; // teeth on the turret's ring gear
  private static final int CRT_WRAPS_M12 = RING_GEAR_TEETH / 12; // m12 encoder full rotations per turret rotation
  private static final int CRT_WRAPS_M13 = RING_GEAR_TEETH / 13; // m13 encoder full rotations per turret rotation
  private static final double M12_OFFSET = 0.939717; // TODO: calibrate — encoder reading (0-1) when turret is at 0°
  private static final double M13_OFFSET = 0.31208; // TODO: calibrate — encoder reading (0-1) when turret is at 0°
  private static final double CRT_ERROR_THRESHOLD = 0.1; // max allowable error between expected and actual encoder
                                                         // reading

  // TODO: check these for the real bot; these are just estimates
  public final Translation3d turretTranslation = new Translation3d(-0.205, 0.0, 0.375);

  // 1 Neo, 5:1 gearbox, 60:12 pivot gearing, non-continuous 360 deg
  // Total reduction: 5 * 5 = 25:1

  private SparkMax spark = new SparkMax(50, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(15.0, 0, 0, DegreesPerSecond.of(2440),
          DegreesPerSecondPerSecond.of(2440))
      // TODO: make this work, and not error
      // .withFeedforward(new SimpleMotorFeedforward(0.0, 7.5, 0.0))
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 5)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withSoftLimit(Degrees.of(-MAX_ONE_DIR_FOV), Degrees.of(MAX_ONE_DIR_FOV))
      .withStatorCurrentLimit(Amps.of(0)) // TODO: make this not 0 to actually run
      .withClosedLoopRampRate(Seconds.of(0.1))
      .withOpenLoopRampRate(Seconds.of(0.1));

  private SmartMotorController smc = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private final PivotConfig turretConfig = new PivotConfig(smc)
      .withHardLimit(Degrees.of(-MAX_ONE_DIR_FOV - 5), Degrees.of(MAX_ONE_DIR_FOV +
          5))
      .withStartingPosition(Degrees.of(0))
      // .withMOI(0.05)
      .withTelemetry("Turret", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(
          new MechanismPositionConfig().withMovementPlane(Plane.XY).withRelativePosition(turretTranslation));

  private Pivot turret = new Pivot(turretConfig);

  // Absolute encoders
  private final REVThroughBoreEncoder m12TAbsEncoder;
  private final REVThroughBoreEncoder m13TAbsEncoder;

  public TurretSubsystem() {
    m12TAbsEncoder = new REVThroughBoreEncoder(1);
    m13TAbsEncoder = new REVThroughBoreEncoder(0);

    new Trigger(() -> m12TAbsEncoder.isConnected() &&
        m13TAbsEncoder.isConnected()).onTrue(Commands.runOnce(() -> {
          System.out.println("Turret absolute encoders connected, zeroing turret via CRT");
          double turretAngle = computeTurretAngleCRT(m12TAbsEncoder.get(),
              m13TAbsEncoder.get());
          System.out.println("CRT turret angle: " + turretAngle + " deg");
          turret.setAngle(Degrees.of(turretAngle));
        }).ignoringDisable(true));

  }

  /**
   * Uses the Chinese Remainder Theorem to compute the absolute turret angle
   * from two absolute encoders geared at coprime ratios (12 and 13).
   *
   * <p>
   * Each encoder reads a fractional rotation (0 to 1). Because the encoder gears
   * have coprime tooth counts, each encoder wraps a different number of times
   * over
   * the turret's full 360° range. CRT lets us determine which "sector" (wrap
   * number)
   * each encoder is in, and thus recover the unique turret position.
   *
   * <p>
   * Algorithm: for each candidate wrap number k1 of the m12 encoder (0..11),
   * compute the implied turret angle θ = (k1 + r1) × 360 / n1, then check if the
   * m13 encoder's fractional reading is consistent. The candidate with the
   * smallest
   * error is the correct one.
   *
   * @param r1Raw raw m12 encoder reading (0 to 1)
   * @param r2Raw raw m13 encoder reading (0 to 1)
   * @return turret angle in degrees, in the range [0, 360)
   */
  private double computeTurretAngleCRT(double r1Raw, double r2Raw) {
    // Apply calibration offsets and normalize to [0, 1)
    double r1 = ((r1Raw - M12_OFFSET) % 1.0 + 1.0) % 1.0;
    double r2 = ((r2Raw - M13_OFFSET) % 1.0 + 1.0) % 1.0;

    double sectorSize1 = 360.0 / CRT_WRAPS_M12; // degrees per m12 encoder wrap

    double bestError = Double.MAX_VALUE;
    int bestK1 = 0;

    for (int k1 = 0; k1 < CRT_WRAPS_M12; k1++) {
      double theta = (k1 + r1) * sectorSize1;
      // Predict what r2 should be at this theta
      double expectedR2 = (CRT_WRAPS_M13 * theta / 360.0) % 1.0;
      if (expectedR2 < 0)
        expectedR2 += 1.0;

      // Circular distance between expected and actual r2 (handles wrap-around)
      double error = Math.abs(expectedR2 - r2);
      error = Math.min(error, 1.0 - error);

      if (error < bestError) {
        bestError = error;
        bestK1 = k1;
      }
    }

    if (bestError > CRT_ERROR_THRESHOLD) {
      System.err.println("[Turret CRT] WARNING: encoder disagreement! Best error = " + bestError
          + " (threshold " + CRT_ERROR_THRESHOLD + "). Check encoder wiring/calibration.");
    }

    double turretAngleDeg = (bestK1 + r1) * sectorSize1;

    Logger.recordOutput("Turret/CRT/SelectedSector", bestK1);
    Logger.recordOutput("Turret/CRT/Error", bestError);
    Logger.recordOutput("Turret/CRT/ComputedAngle", turretAngleDeg);

    return turretAngleDeg;
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

  public Angle getRawAngle() {
    return turret.getAngle();
  }

  public Command set(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  public Command rezero() {
    return Commands.runOnce(() -> spark.getEncoder().setPosition(0),
        this).withName("Turret.Rezero");
  }

  public Command sysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
  }

  @Override
  public void periodic() {
    turret.updateTelemetry();

    Logger.recordOutput("Turret/RawYamsAngle", getRawAngle().in(Degrees));
    Logger.recordOutput("Turret/m12TAbsEncoder", m12TAbsEncoder.get() - M12_OFFSET);
    Logger.recordOutput("Turret/m13TAbsEncoder", m13TAbsEncoder.get() - M13_OFFSET);
    Logger.recordOutput("Turret/crtAngle", computeTurretAngleCRT(m12TAbsEncoder.get(), m13TAbsEncoder.get()));

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
