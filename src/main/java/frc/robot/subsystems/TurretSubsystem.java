package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
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
  // 1 Neo, 5:1 gearbox, 60:12 pivot gearing, non-continuous 360 deg
  // Total reduction: 5 * 5 = 25:1
  public final double GEAR_RATIO = 1.0 / (5.0 * (60.0 / 12.0));

  public final Angle MAX_ANGLE = TurretConstants.MAX_ONE_DIR_FOV;
  public final Angle MIN_ANGLE = TurretConstants.MAX_ONE_DIR_FOV.unaryMinus();

  // Motor & encoder
  private final SparkMax turretSpark = new SparkMax(Constants.TurretConstants.k_turretMotorId, MotorType.kBrushless);
  private final SparkMaxConfig turretConfig;
  private final SparkClosedLoopController turretController = turretSpark.getClosedLoopController();
  private final RelativeEncoder turretEncoder = turretSpark.getEncoder();

  // Profiled PID controller (operates in degrees)
  // Max velocity: NEO free speed (5676 RPM) / gear ratio => deg/s
  // Max acceleration: set to match velocity (tune as needed)
  private static final double MAX_VELOCITY_ROT_PER_SEC = 2.5; // ~1362 deg/s
  private static final double MAX_ACCEL_ROT_PER_SEC2 = MAX_VELOCITY_ROT_PER_SEC; // ~1362 deg/s^2

  // TODO: Tune PID gains
  private final ProfiledPIDController profiledPID = new ProfiledPIDController(
      45.0, // kP (tune me)
      4.0, // kI
      0.0, // kD
      new TrapezoidProfile.Constraints(MAX_VELOCITY_ROT_PER_SEC, MAX_ACCEL_ROT_PER_SEC2));

  private boolean isRezeroed = false;
  private boolean isRezeroing = false;
  private boolean closedLoopEnabled = false;

  // Absolute encoders
  private final REVThroughBoreEncoder m12TAbsEncoder;
  private final REVThroughBoreEncoder m13TAbsEncoder;

  public final Trigger isAtTarget = new Trigger(
      () -> Rotations
          .of(Math.abs(turretSpark.getClosedLoopController().getSetpoint() -
              turretSpark.getClosedLoopController()
                  .getMAXMotionSetpointPosition()))
          .lt(SuperstructureConstants.k_turretTolerance));

  // public final Trigger isAtTarget = new Trigger(() -> true);

  public TurretSubsystem() {
    turretConfig = new SparkMaxConfig();

    turretConfig.signals.setSetpointAlwaysOn(true);
    turretConfig.signals.maxMotionSetpointPositionAlwaysOn(true);

    turretConfig.softLimit.forwardSoftLimit(MAX_ANGLE.in(Rotations));
    turretConfig.softLimit.reverseSoftLimit(MIN_ANGLE.in(Rotations));
    turretConfig.softLimit.forwardSoftLimitEnabled(true);
    turretConfig.softLimit.reverseSoftLimitEnabled(true);

    // TODO: figure out how to turn on status frames 7/8/9

    turretConfig.inverted(true);
    turretConfig.idleMode(IdleMode.kCoast);
    turretConfig.smartCurrentLimit(15);
    // turretConfig.openLoopRampRate(0.1);
    // turretConfig.closedLoopRampRate(0.1);

    // Encoder conversion: motor rotations -> mechanism degrees
    turretConfig.encoder.positionConversionFactor(GEAR_RATIO); // rotations -> degrees
    turretConfig.encoder.velocityConversionFactor(GEAR_RATIO / 60.0); // RPM -> deg/s

    turretConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(45.0)
        .i(0)
        .d(0)
        .outputRange(-1, 1).feedForward
        // kV is now in Volts, so we multiply by the nominal voltage (12V)
        .kV(4.0);
    // .kV(3.0 * (1 / GEAR_RATIO));

    turretConfig.closedLoop.maxMotion
        .cruiseVelocity(Degrees.of(1440).in(Rotations))
        .maxAcceleration(Degrees.of(5760).in(Rotations)) // TODO: Up this, so the profile ramps down later
        .allowedProfileError(Degrees.of(22.5).in(Rotations));

    turretSpark.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

          System.out.println("Averaged " + samples.size() + " samples over 1 second");
          System.out.println(
              "Setting Offset: " + turretAngle.in(Degrees) + " deg (" + turretAngle.in(Rotations) + " rot)");

          // Set encoder position (conversion factor is already applied)
          turretEncoder.setPosition(avgRotations);
          profiledPID.reset(avgRotations);

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
    return run(() -> {
      // Safety check: prevent commanding angles outside of physical limits (which
      // could cause damage)
      closedLoopEnabled = true;
      // Angle adjustedAngle =
      // angle.plus(getAngle().minus(computeTurretAngleFromAbs()));
      // profiledPID.setGoal(clampSafeAngle(adjustedAngle).in(Rotations));

      // profiledPID.setGoal(clampSafeAngle(adjustedAngle).in(Rotations));

      turretController.setSetpoint(angle.in(Rotations), SparkBase.ControlType.kMAXMotionPositionControl);
    }).withName("Turret.SetAngle(" + angle.in(Degrees) + " deg)");
  }

  public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier) {
    return run(() -> {
      closedLoopEnabled = true;
      // profiledPID.setGoal(clampSafeAngle(turretAngleSupplier.get()).in(Rotations));

      turretController.setSetpoint(turretAngleSupplier.get().in(Rotations),
          SparkBase.ControlType.kMAXMotionPositionControl);
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
    return Rotations.of(turretEncoder.getPosition());
  }

  public Command set(double dutyCycle) {
    return run(() -> {
      closedLoopEnabled = false;
      turretSpark.set(dutyCycle);
    }).withName("Turret.Set(" + dutyCycle + ")");
  }

  @Override
  public void periodic() {
    // Auto-rezero when absolute encoders become available
    if (!isRezeroed && !isRezeroing && m12TAbsEncoder.isConnected() && m13TAbsEncoder.isConnected()) {
      CommandScheduler.getInstance().schedule(rezero());
    }

    // double outputVolts = 0.0;

    // Run profiled PID loop
    // if (closedLoopEnabled) {
    // double measurement = turretEncoder.getPosition();
    // double pidOutput = profiledPID.calculate(measurement);
    // double ffOutput = feedforward.calculate(profiledPID.getSetpoint().velocity);
    // outputVolts = pidOutput + ffOutput;

    // Logger.recordOutput("Turret/calculate/pidOutput", pidOutput);
    // Logger.recordOutput("Turret/calculate/ffOutput", ffOutput);
    // Logger.recordOutput("Turret/calculate/outputVolts", outputVolts);

    // // Clamp to battery voltage and convert to duty cycle
    // outputVolts = Math.max(-12.0, Math.min(12.0, outputVolts));
    // turretSpark.setVoltage(outputVolts);
    // }

    // Telemetry
    Logger.recordOutput("Turret/outputVolts",
        turretSpark.getAppliedOutput() * RobotController.getBatteryVoltage());
    Logger.recordOutput("Turret/closedLoopEnabled", closedLoopEnabled);
    Logger.recordOutput("Turret/isRezeroed", isRezeroed);
    Logger.recordOutput("Turret/m12TAbsEncoderConnected", m12TAbsEncoder.isConnected());
    Logger.recordOutput("Turret/m13TAbsEncoderConnected", m13TAbsEncoder.isConnected());
    Logger.recordOutput("Turret/PositionRots", turretEncoder.getPosition());
    Logger.recordOutput("Turret/VelocityRotsPerSec", turretEncoder.getVelocity());
    Logger.recordOutput("Turret/computeTurretAngleFromAbs", computeTurretAngleFromAbs());
    Logger.recordOutput("Turret/error",
        Rotations.of(turretSpark.getClosedLoopController().getSetpoint()
            - turretSpark.getClosedLoopController().getMAXMotionSetpointPosition()).in(Degrees),
        Degrees);

    Logger.recordOutput("Turret/frequency/m12", m12TAbsEncoder.getFrequency());
    Logger.recordOutput("Turret/frequency/m13", m13TAbsEncoder.getFrequency());

    // MaxMotion
    Logger.recordOutput("Turret/MaxMotion/PositionGoalRots",
        turretSpark.getClosedLoopController().getSetpoint(), Rotations);
    Logger.recordOutput("Turret/MaxMotion/PositionSetpointRots",
        turretSpark.getClosedLoopController().getMAXMotionSetpointPosition(), Rotations);
    Logger.recordOutput("Turret/MaxMotion/PositionSetpointVelocity",
        turretSpark.getClosedLoopController().getMAXMotionSetpointVelocity(), RotationsPerSecond);
    Logger.recordOutput("Turret/MaxMotion/controlType",
        turretSpark.getClosedLoopController().getControlType());
  }

  @Override
  public void simulationPeriodic() {
    // No YAMS sim — add simulation model here if needed
  }

  public String[] getTurretConnections() {
    String[] out = {null, null, null};

    try {
      turretSpark.getAppliedOutput();
    } catch (Exception e) {
      out[0] = e.toString();
    }

    try {
      turretSpark.getMotorTemperature();
    } catch (Exception e) {
      out[1] = e.toString();
    }

    try {
      turretEncoder.getPosition();
    } catch (Exception e) {
      out[2] = e.toString();
    }

    return out;
  }
}
