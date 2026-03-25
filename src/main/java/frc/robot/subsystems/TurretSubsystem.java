package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.SuperstructureConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.wrappers.REVThroughBoreEncoder;
import edu.wpi.first.math.geometry.Translation2d;

public class TurretSubsystem extends SubsystemBase {
  // 1 Neo, 5:1 gearbox, 60:12 pivot gearing, non-continuous 360 deg
  // Total reduction: 5 * 5 = 25:1
  public final double GEAR_RATIO = 1.0 / (5.0 * (60.0 / 12.0));

  // Motor & encoder
  private final SparkMax turretSpark = new SparkMax(Constants.TurretConstants.k_turretMotorId, MotorType.kBrushless);
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

  // Feedforward (kS, kV, kA — in volts, volts*s/deg, volts*s^2/deg)
  // TODO: Tune feedforward gains via SysId
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 3.5, 0.0);

  private boolean isRezeroed = false;
  private boolean isRezeroing = false;
  private boolean closedLoopEnabled = false;

  // Absolute encoders
  private final REVThroughBoreEncoder m12TAbsEncoder;
  private final REVThroughBoreEncoder m13TAbsEncoder;

  // public final Trigger isAtTarget = new Trigger(() -> profiledPID.atGoal());

  public final Trigger isAtTarget = new Trigger(
      () -> Math.abs(Rotations.of(turretEncoder.getPosition()
          - profiledPID.getGoal().position).in(Degrees)) < SuperstructureConstants.k_turretTolerance.in(Degrees));

  public TurretSubsystem() {
    m12TAbsEncoder = new REVThroughBoreEncoder(1);
    m13TAbsEncoder = new REVThroughBoreEncoder(0);

    // Configure SparkMax
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true);
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(30);
    config.openLoopRampRate(0.1);
    config.closedLoopRampRate(0.1);

    // Encoder conversion: motor rotations -> mechanism degrees
    config.encoder.positionConversionFactor(GEAR_RATIO); // rotations -> degrees
    config.encoder.velocityConversionFactor(GEAR_RATIO / 60.0); // RPM -> deg/s

    turretSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    profiledPID.setTolerance(SuperstructureConstants.k_turretTolerance.in(Degrees));
  }

  public Command rezero() {
    final ArrayList<Double> samples = new ArrayList<>();
    double averageTime = 1.0;

    return Commands.sequence(
        Commands.runOnce(() -> {
          isRezeroing = true;
          samples.clear();
          System.out.println("==============================");
          System.out.println("== Rezeroing turret (3s avg) =");
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

          System.out.println("Averaged " + samples.size() + " samples over 3 seconds");
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
      profiledPID.setGoal(clampSafeAngle(angle).in(Rotations));
    }).withName("Turret.SetAngle(" + angle.in(Degrees) + " deg)");
  }

  public Command setAngleDynamic(Supplier<Angle> turretAngleSupplier) {
    return run(() -> {
      closedLoopEnabled = true;
      profiledPID.setGoal(clampSafeAngle(turretAngleSupplier.get()).in(Rotations));
    }).withName("Turret.SetAngleDynamic");
  }

  public Angle clampSafeAngle(Angle angle) {
    double MAX_ONE_DIR_FOV = TurretConstants.MAX_ONE_DIR_FOV;

    if (angle.in(Degrees) > MAX_ONE_DIR_FOV) {
      return Degrees.of(MAX_ONE_DIR_FOV);
    } else if (angle.in(Degrees) < -MAX_ONE_DIR_FOV) {
      return Degrees.of(-MAX_ONE_DIR_FOV);
    }
    return angle;
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

    double outputVolts = 0.0;

    // Run profiled PID loop
    if (closedLoopEnabled) {
      double measurement = turretEncoder.getPosition();
      double pidOutput = profiledPID.calculate(measurement);
      double ffOutput = feedforward.calculate(profiledPID.getSetpoint().velocity);
      outputVolts = pidOutput + ffOutput;

      Logger.recordOutput("Turret/calculate/pidOutput", pidOutput);
      Logger.recordOutput("Turret/calculate/ffOutput", ffOutput);
      Logger.recordOutput("Turret/calculate/outputVolts", outputVolts);

      // Clamp to battery voltage and convert to duty cycle
      outputVolts = Math.max(-12.0, Math.min(12.0, outputVolts));
      turretSpark.setVoltage(outputVolts);
    }

    // Telemetry
    Logger.recordOutput("Turret/outputVolts", outputVolts);
    Logger.recordOutput("Turret/closedLoopEnabled", closedLoopEnabled);
    Logger.recordOutput("Turret/isRezeroed", isRezeroed);
    Logger.recordOutput("Turret/m12TAbsEncoderConnected", m12TAbsEncoder.isConnected());
    Logger.recordOutput("Turret/m13TAbsEncoderConnected", m13TAbsEncoder.isConnected());
    Logger.recordOutput("Turret/PositionRots", turretEncoder.getPosition());
    Logger.recordOutput("Turret/VelocityRotsPerSec", turretEncoder.getVelocity());
    Logger.recordOutput("Turret/GoalRots", profiledPID.getGoal().position);
    Logger.recordOutput("Turret/SetpointRots", profiledPID.getSetpoint().position);
    Logger.recordOutput("Turret/SetpointVelRotsPerSec", profiledPID.getSetpoint().velocity);
    Logger.recordOutput("Turret/AtGoal", profiledPID.atGoal());
    Logger.recordOutput("Turret/computeTurretAngleFromAbs", computeTurretAngleFromAbs());
  }

  @Override
  public void simulationPeriodic() {
    // No YAMS sim — add simulation model here if needed
  }

  public static Translation2d getTurretVelocity(ChassisSpeeds robot, double rot){
    double rotVel = robot.omegaRadiansPerSecond;
    return new Translation2d(
      robot.vxMetersPerSecond + Math.cos(rot) * TurretConstants.k_turretDistToRobotCenter * rotVel,
      robot.vyMetersPerSecond + Math.sin(rot) * TurretConstants.k_turretDistToRobotCenter * rotVel
    );
  }
}
