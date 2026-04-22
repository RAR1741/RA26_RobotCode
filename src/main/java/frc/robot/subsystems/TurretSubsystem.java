package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.wrappers.REVThroughBoreEncoder;

public class TurretSubsystem extends SubsystemBase {
  // 1 TalonFX, 3:1 stage 1 gearbox, 4:1 stage 2 gearbox, 60:12 pivot gearing,
  // non-continuous 360 deg
  // Total reduction: 3 * 4 * 5 = 60:1
  public final double GEAR_RATIO = 1.0 / (3.0 * 4.0 * (60.0 / 12.0));

  public final Angle MAX_ANGLE = TurretConstants.MAX_ONE_DIR_FOV;
  public final Angle MIN_ANGLE = TurretConstants.MAX_ONE_DIR_FOV.unaryMinus();

  // Motor & control requests
  private final TalonFX turretMotor = new TalonFX(Constants.TurretConstants.k_turretMotorId, "Drivetrain");
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  // Tracked target position for isAtTarget (in mechanism rotations)
  private double targetPositionRotations = 0.0;

  private boolean isRezeroed = false;
  private boolean isRezeroing = false;
  private boolean closedLoopEnabled = false;

  // Absolute encoders
  private final REVThroughBoreEncoder m12TAbsEncoder;
  private final REVThroughBoreEncoder m13TAbsEncoder;

  public final Trigger isAtTarget = new Trigger(
      () -> Rotations
          .of(Math.abs(turretMotor.getClosedLoopError().getValueAsDouble()))
          .lt(SuperstructureConstants.k_turretTolerance));

  public TurretSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Feedback: configure SensorToMechanismRatio so all positions/velocities
    // are reported in mechanism rotations (not motor rotations)
    // SensorToMechanismRatio = motor rotations per mechanism rotation = 25.0
    config.Feedback.SensorToMechanismRatio = 1.0 / GEAR_RATIO;

    // Soft limits (in mechanism rotations)
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Rotations);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // Motor output
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Current limits
    config.CurrentLimits.StatorCurrentLimit = 15;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Slot 0 PID gains for MotionMagic (Voltage output, mechanism rotations)
    // Original SparkMax: kP=45.0 (duty cycle per mechanism rot error)
    // TalonFX Voltage mode: kP in V per mechanism rot error
    // Conversion: 45.0 duty-cycle * 12V = 540 V/rot
    // TODO: Tune PID gains for TalonFX
    config.Slot0.kP = 10.0;// 540.0;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kV = 7.5; // 4.0 // V / (mechanism rot/s)
    config.Slot0.kS = 0;

    // MotionMagic profile constraints (in mechanism rotations)
    // Cruise velocity: 1440 deg/s = 4.0 mechanism rot/s
    config.MotionMagic.MotionMagicCruiseVelocity = Degrees.of(550).in(Rotations);
    // Acceleration: 5760 deg/s² = 16.0 mechanism rot/s²
    config.MotionMagic.MotionMagicAcceleration = Degrees.of(550 * 4).in(Rotations);
    // Jerk: smooths accel/decel transitions (S-Curve). 0 = trapezoidal (no
    // smoothing).
    // TODO: Tune jerk for smooth turret motion
    // config.MotionMagic.MotionMagicJerk = Degrees.of(3600).in(Rotations); // 10.0
    // mechanism rot/s³

    turretMotor.getConfigurator().apply(config);

    m12TAbsEncoder = new REVThroughBoreEncoder(0, TurretConstants.m12Frequency);
    m13TAbsEncoder = new REVThroughBoreEncoder(1, TurretConstants.m13Frequency);
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

          System.out.println("Averaged " + samples.size() + " samples over 1 second");
          System.out.println(
              "Setting Offset: " + turretAngle.in(Degrees) + " deg (" + turretAngle.in(Rotations) + " rot)");

          // Set encoder position (SensorToMechanismRatio handles the conversion)
          turretMotor.setPosition(avgRotations);

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
    double factor = 1.4; // 1.4 = (12/60) * (14/60) / ((12/60) - (14/60))
    double colG = ((colF * factor) + 2.0) % 1.0;
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
    return run(() -> {
      closedLoopEnabled = true;
      targetPositionRotations = angle.in(Rotations);
      turretMotor.setControl(motionMagicRequest.withPosition(targetPositionRotations));
    }).withName("Turret.SetAngle(" + angle.in(Degrees) + " deg)");
  }

  public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier,
      Supplier<AngularVelocity> turretAnglularVelocityCompensationSupplier) {
    return run(() -> {
      closedLoopEnabled = true;
      targetPositionRotations = turretAngleSupplier.get().in(Rotations);

      Voltage ffAngularVelocityCompensation = Volts.of(0.0);
      // TODO: Assume this should be the same as the kV from above, but it can be
      // tuned. It can also be set to 0 to remove the effect
      double compensationkV = 7.5;

      /*
       * Only apply the angular velocity compensation if the turret hasn't wrapped,
       * and
       * our current position is relatively close to the target. If the turret is 30
       * degrees
       * or less away from the setpoint, then apply the compensation FF
       */
      if (turretAngleSupplier.get().isNear(this.getAngle(), Degrees.of(30.0))) {
        ffAngularVelocityCompensation = Volts
            .of(turretAnglularVelocityCompensationSupplier.get().in(RotationsPerSecond) * compensationkV);
      }
      Logger.recordOutput("Turret/FFVolts", ffAngularVelocityCompensation);

      turretMotor.setControl(
          motionMagicRequest.withPosition(targetPositionRotations).withFeedForward(ffAngularVelocityCompensation));
    }).withName("Turret.SetAngleDynamic");
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

  public Angle getRobotAdjustedAngle() {
    // Returns the turret angle in the robot's coordinate frame
    // since the turret is mounted backwards, we need to add 180 degrees
    return getAngle().plus(Degrees.of(180));
  }

  public Angle getAngle() {
    return Rotations.of(turretMotor.getPosition().getValueAsDouble());
  }

  public Command set(double dutyCycle) {
    return run(() -> {
      closedLoopEnabled = false;
      turretMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
    }).withName("Turret.Set(" + dutyCycle + ")");
  }

  @Override
  public void periodic() {
    // Auto-rezero when absolute encoders become available
    if (!isRezeroed && !isRezeroing && m12TAbsEncoder.isConnected() && m13TAbsEncoder.isConnected()) {
      CommandScheduler.getInstance().schedule(rezero());
    }

    // Telemetry
    Logger.recordOutput("Turret/outputVolts", turretMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Turret/closedLoopEnabled", closedLoopEnabled);
    Logger.recordOutput("Turret/isRezeroed", isRezeroed);
    Logger.recordOutput("Turret/m12TAbsEncoderConnected", m12TAbsEncoder.isConnected());
    Logger.recordOutput("Turret/m13TAbsEncoderConnected", m13TAbsEncoder.isConnected());
    Logger.recordOutput("Turret/PositionRots", turretMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("Turret/VelocityRotsPerSec", turretMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Turret/computeTurretAngleFromAbs", computeTurretAngleFromAbs());
    Logger.recordOutput("Turret/error",
        Rotations.of(turretMotor.getClosedLoopError().getValueAsDouble()).in(Degrees),
        Degrees);

    Logger.recordOutput("Turret/frequency/m12", m12TAbsEncoder.getFrequency());
    Logger.recordOutput("Turret/frequency/m13", m13TAbsEncoder.getFrequency());

    // MotionPofile
    Logger.recordOutput("Turret/MotionPofile/PositionGoalRots",
        targetPositionRotations, Rotations);
    Logger.recordOutput("Turret/MotionPofile/PositionReferenceRots",
        turretMotor.getClosedLoopReference().getValueAsDouble(), Rotations);
    Logger.recordOutput("Turret/MotionPofile/ReferenceVelocity",
        turretMotor.getClosedLoopReferenceSlope().getValueAsDouble(), RotationsPerSecond);
  }

  @Override
  public void simulationPeriodic() {
    // No sim model — add simulation model here if needed
  }
}
