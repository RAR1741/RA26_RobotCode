package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.StateManager.State;

public class ShootOnTheMoveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Superstructure superstructure;
  // private final LEDSubsystem leds;

  private Supplier<Translation2d> aimPointSupplier; // The point to aim at
  private AngularVelocity latestShootSpeed = RPM.of(0);
  private Angle latestHoodAngle = Degrees.of(80.0);
  private Angle latestTurretAngle = Degrees.of(0.0);
  private AngularVelocity latestTurretAngularVelocityCompensation = DegreesPerSecond.of(0.0);

  private boolean isRunning = false;

  private Command aimCommand;

  public ShootOnTheMoveCommand(CommandSwerveDrivetrain drivetrain, Superstructure superstructure,
      Supplier<Translation2d> aimPointSupplier) {
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.aimPointSupplier = aimPointSupplier;
    // this.leds = superstructure.getLEDs();
  }

  private void setIsRunning(boolean state) {
    isRunning = state;
  }

  private boolean getisRunning() {
    return isRunning;
  }

  @Override
  public void initialize() {
    super.initialize();

    latestHoodAngle = superstructure.getHoodAngle();
    latestTurretAngle = superstructure.getTurretAngle();
    latestShootSpeed = superstructure.getShooterSpeed();
    // It is ok to not initialize latestHoodAngularVelocityCompensation since it will be zero at init

    // TODO: when this current command ends, we should probably cancel the dynamic
    // aim command

    aimCommand = superstructure.aimDynamicCommand(
        () -> {
          return this.latestShootSpeed;
        },
        () -> {
          return this.latestTurretAngle;
        },
        () -> {
          return this.latestHoodAngle;
        },
        () -> {
          return this.latestTurretAngularVelocityCompensation;
        });

    aimCommand.schedule();

    setIsRunning(true);
  }

  @Override
  public boolean isFinished() {
    return !getisRunning(); // it freaks out and runs wrong
  }

  @Override
  public void execute() {
    // Don't even try to calculate if we're in the pass dead zone, just don't shoot
    if (superstructure.stateManager.getState() == State.PASS_DEAD_ZONE) {
      Logger.recordOutput("ShootOnTheMove/rawTarget", drivetrain.getState().Pose.getTranslation());
      return;
    }

    // leds.setAllSolidColor(LEDConstants.sotmOnColor).execute();
    // Calculate trajectory to aimPoint
    Translation2d target = aimPointSupplier.get();
    Logger.recordOutput("ShootOnTheMove/rawTarget", target);

    Transform2d botPoseToShooterPoseTransform = new Transform2d(
      superstructure.getShooterPose().getTranslation().toTranslation2d(),
      superstructure.getShooterPose().getRotation().toRotation2d());
    Pose2d shooterLocation = drivetrain.getState().Pose.plus(botPoseToShooterPoseTransform);

    // Ignore this parameter for now, the range tables will account for it :/
    // var deltaH = target.getMeasureZ().minus(shooterLocation.getMeasureZ());
    Translation2d shooterOnGround = shooterLocation.getTranslation();

    Distance distanceToTarget = Meters.of(shooterOnGround.getDistance(target));

    // Get time of flight. We could try to do this analytically but for now it's
    // easier and more realistic
    // to use a simple linear approximation based on empirical data.
    double timeOfFlight = getFlightTime(distanceToTarget);

    // Calculate corrective vector based on our current velocity multiplied by time
    // of flight.
    // If we're stationary, this should be zero. If we're backing up, this will be
    // "ahead" of the target, etc.
    ChassisSpeeds updatedPosition = ChassisSpeeds
        .fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation())
        .times(timeOfFlight);
    Translation2d correctiveVector = new Translation2d(updatedPosition.vxMetersPerSecond, updatedPosition.vyMetersPerSecond)
        .unaryMinus();

    Logger.recordOutput("FieldSimulation/AimTargetCorrected",
        new Pose2d(target.plus(correctiveVector), Rotation2d.kZero));

    Translation2d correctedTarget = target.plus(correctiveVector);
    Translation2d vectorToTarget = shooterOnGround.minus(correctedTarget);

    Distance correctedDistance = Meters.of(vectorToTarget.getNorm());
    Angle calculatedHeading = vectorToTarget.getAngle()
        .rotateBy(drivetrain.getState().Pose.getRotation().unaryMinus())
        .getMeasure();

    AngularVelocity calculatedAngularVelocityCompensation;
    calculatedAngularVelocityCompensation = RadiansPerSecond.of(drivetrain.getState().Speeds.omegaRadiansPerSecond).unaryMinus();
    // TODO This does not yet account for the angular velocity caused by the x,y velocity of the robot
    // This is maybe ok though since the angle doesn't change much based on x,y the further you get from the target

    Logger.recordOutput("ShootOnTheMove/shooterLocation", shooterLocation);
    Logger.recordOutput("ShootOnTheMove/shooterOnGround", shooterOnGround);
    Logger.recordOutput("ShootOnTheMove/RobotHeading", drivetrain.getState().Pose.getRotation().getDegrees());
    Logger.recordOutput("ShootOnTheMove/DesiredTurretHeading", calculatedHeading.in(Degrees));
    Logger.recordOutput("ShootOnTheMove/distanceToTarget", distanceToTarget);
    Logger.recordOutput("ShootOnTheMove/Running", isRunning);
    Logger.recordOutput("ShootOnTheMove/IsShootingOnTheMove", getisRunning());
    Logger.recordOutput("ShootOnTheMove/robotAngularVelocity", RadiansPerSecond.of(drivetrain.getState().Speeds.omegaRadiansPerSecond));
    Logger.recordOutput("ShootOnTheMove/calculatedAngularVelocityCompensation", calculatedAngularVelocityCompensation);

    latestTurretAngle = calculatedHeading;
    latestTurretAngularVelocityCompensation = calculatedAngularVelocityCompensation;
    latestShootSpeed = calculateRequiredShooterSpeed(correctedDistance);
    latestHoodAngle = calculateRequiredHoodAngle(correctedDistance);

    // FOR TESTING - DO NOT USE (turns off shooter and hood)
    // latestShootSpeed = RPM.of(0);
    // latestHoodAngle = Degrees.of(80.0);

    // Kicker corrective bias based on how close to forward/0 we are
    // latestTurretAngle =
    // latestTurretAngle.times(Double.valueOf(DriverStation.getGameSpecificMessage()));

    // FOR TESTING - DO NOT USE
    // var gameData = DriverStation.getGameSpecificMessage();
    // if (gameData != null && !gameData.isEmpty()) {
    // latestShootSpeed =
    // RPM.of(Double.valueOf(DriverStation.getGameSpecificMessage()));
    // }

    // latestShootSpeed = RPM.of(2400);
    // latestHoodAngle =
    // Degrees.of(Double.parseDouble(DriverStation.getGameSpecificMessage()));
    // latestHoodAngle = superstructure.getHoodAngle();

    // Smartdashboard testing:
    // if (SmartDashboard.getBoolean("SOTMOverride", false)) {
    // latestShootSpeed = RPM.of(SmartDashboard.getNumber("ShooterSpeedRPM",
    // latestShootSpeed.in(RPM)));
    // latestTurretAngle = Degrees.of(SmartDashboard.getNumber("TurretAngleDeg",
    // latestTurretAngle.in(Degrees)));
    // latestHoodAngle = Degrees.of(SmartDashboard.getNumber("HoodAngleDeg",
    // latestHoodAngle.in(Degrees)));
    // }

    superstructure.setShooterSetpoints(
        latestShootSpeed,
        latestTurretAngle,
        latestHoodAngle);

    // System.out.println("Shooting at distance: " + correctedDistance + " requires
    // speed: " + latestShootSpeed
    // + ", hood angle: " + latestHoodAngle + ", turret angle: " +
    // latestTurretAngle);
  }

  public void end(boolean interrupted) {

    // leds.setAllSolidColor(LEDConstants.teleColor).execute();
    setIsRunning(false);
    Logger.recordOutput("ShootOnTheMove/Running", getisRunning());
    if (aimCommand != null) {
      aimCommand.end(true);
      aimCommand.cancel();
      Commands.runOnce(() -> superstructure.getShooterSubsystem().stopCommand().schedule());
      Logger.recordOutput("ShootOnTheMove/IsSched", CommandScheduler.getInstance().isScheduled(aimCommand));
      Logger.recordOutput("ShootOnTheMove/isDone", isFinished());
    }
    Logger.recordOutput("ShootOnTheMove/IsShootingOnTheMove", getisRunning());
  }

  private double getFlightTime(Distance distanceToTarget) {
    // Simple linear approximation based on empirical data.
    return TIME_OF_FLIGHT_BY_DISTANCE.get(distanceToTarget.in(Meters));
  }

  private AngularVelocity calculateRequiredShooterSpeed(Distance distanceToTarget) {
    // If we're firing at the hub, use the distance to determine the hood angle
    if (superstructure.stateManager.getState() == State.SHOOTING) {
      return RPM.of(HUB_SHOOTING_SPEED_BY_DISTANCE.get(distanceToTarget.in(Meters)));
    }

    return RPM.of(PASS_SHOOTING_SPEED_BY_DISTANCE.get(distanceToTarget.in(Meters)));
  }

  private Angle calculateRequiredHoodAngle(Distance distanceToTarget) {
    // If we're firing at the hub, use the distance to determine the hood angle
    if (superstructure.stateManager.getState() == State.SHOOTING) {
      return Degrees.of(HOOD_ANGLE_BY_DISTANCE.get(distanceToTarget.in(Meters)));
    }

    // Default hood angle if we're not aiming at the hub
    // return Degrees.of(45.0);
    return Degrees.of(40.0);
  }

  // meters, seconds
  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.0, 1.2),
      Map.entry(4.792132, 1.2),
      Map.entry(14.07683, 3.5));

  // meters, RPM
  private static final InterpolatingDoubleTreeMap HUB_SHOOTING_SPEED_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.554228, 1900.0), // HUB
      Map.entry(3.153828, 2000.0), // TRENCH
      Map.entry(4.792132, 2350.0)); // OUTPOST

  // meters, RPM
  private static final InterpolatingDoubleTreeMap PASS_SHOOTING_SPEED_BY_DISTANCE = InterpolatingDoubleTreeMap
      .ofEntries(
          Map.entry(5.281523, 1700.0), // Close bump
          Map.entry(7.883727, 2300.0), // Midfield
          Map.entry(10.29897, 2800.0), // Far bump
          Map.entry(14.07683, 4000.0)); // Far wall

  // meters, degrees
  private static final InterpolatingDoubleTreeMap HOOD_ANGLE_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.554228, 73.0), // HUB
      Map.entry(3.153828, 54.5), // TRENCH
      Map.entry(4.792132, 51.5)); // OUTPOST
}
