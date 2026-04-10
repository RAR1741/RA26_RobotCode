package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.StateManager.State;

public class ShootOnTheMoveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Superstructure superstructure;
  private final LEDSubsystem leds;

  private Supplier<Translation2d> aimPointSupplier; // The point to aim at
  private AngularVelocity latestShootSpeed = RPM.of(0);
  private Angle latestHoodAngle = Degrees.of(80.0);
  private Angle latestTurretAngle = Degrees.of(0.0);

  private boolean isRunning = false;

  private Command aimCommand;

  public ShootOnTheMoveCommand(CommandSwerveDrivetrain drivetrain, Superstructure superstructure,
      Supplier<Translation2d> aimPointSupplier) {
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.aimPointSupplier = aimPointSupplier;
    this.leds = superstructure.getLEDs();
  }

  private void setIsRunning(boolean state){
    isRunning = state;
  }

  private boolean getisRunning(){
    return isRunning;
  }

  @Override
  public void initialize() {
    super.initialize();

    latestHoodAngle = superstructure.getHoodAngle();
    latestTurretAngle = superstructure.getTurretAngle();
    latestShootSpeed = superstructure.getShooterSpeed();

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
        }
    );

    CommandScheduler.getInstance().schedule(aimCommand);
    
    setIsRunning(true);
  }

  @Override
  public boolean isFinished() {
    System.out.println(!isRunning);
    return !getisRunning(); //it freaks out and runs wrong
  }

  @Override
  public void execute() {
    // Don't even try to calculate if we're in the pass dead zone, just don't shoot
    if (superstructure.stateManager.getState() == State.PASS_DEAD_ZONE) {
      Logger.recordOutput("ShootOnTheMove/rawTarget", drivetrain.getState().Pose.getTranslation());
      return;
    }

    leds.setAllSolidColor(LEDConstants.sotmOnColor).execute();
    // Calculate trajectory to aimPoint
    var target = aimPointSupplier.get();
    Logger.recordOutput("ShootOnTheMove/rawTarget", target);

    var shooterLocation = drivetrain.getState().Pose.getTranslation()
        .plus(superstructure.getShooterPose().toPose2d().getTranslation());

    // Ignore this parameter for now, the range tables will account for it :/
    // var deltaH = target.getMeasureZ().minus(shooterLocation.getMeasureZ());
    var shooterOnGround = new Translation2d(shooterLocation.getX(), shooterLocation.getY());

    var distanceToTarget = Meters.of(shooterOnGround.getDistance(target));

    // Get time of flight. We could try to do this analytically but for now it's
    // easier and more realistic
    // to use a simple linear approximation based on empirical data.
    double timeOfFlight = getFlightTime(distanceToTarget);

    // Calculate corrective vector based on our current velocity multiplied by time
    // of flight.
    // If we're stationary, this should be zero. If we're backing up, this will be
    // "ahead" of the target, etc.
    var updatedPosition = ChassisSpeeds
        .fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation())
        .times(timeOfFlight);
    var correctiveVector = new Translation2d(updatedPosition.vxMetersPerSecond, updatedPosition.vyMetersPerSecond)
        .unaryMinus();

    Logger.recordOutput("FieldSimulation/AimTargetCorrected",
        new Pose2d(target.plus(correctiveVector), Rotation2d.kZero));

    var correctedTarget = target.plus(correctiveVector);
    var vectorToTarget = shooterLocation.minus(correctedTarget);

    var correctedDistance = Meters.of(vectorToTarget.getNorm());
    var calculatedHeading = vectorToTarget.getAngle()
        .rotateBy(drivetrain.getState().Pose.getRotation().unaryMinus())
        .getMeasure();

    Logger.recordOutput("ShootOnTheMove/RobotHeading", drivetrain.getState().Pose.getRotation().getDegrees());
    Logger.recordOutput("ShootOnTheMove/DesiredTurretHeading", calculatedHeading.in(Degrees));
    Logger.recordOutput("ShootOnTheMove/distanceToTarget", distanceToTarget);
    Logger.recordOutput("ShootOnTheMove/Running", isRunning);
    Logger.recordOutput("ShootOnTheMove/IsShootingOnTheMove", getisRunning());

    latestTurretAngle = calculatedHeading;
    latestShootSpeed = calculateRequiredShooterSpeed(correctedDistance);
    latestHoodAngle = calculateRequiredHoodAngle(correctedDistance);

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
    
    leds.setAllSolidColor(LEDConstants.teleColor).execute();
    setIsRunning(false);
    Logger.recordOutput("ShootOnTheMove/Running", getisRunning());
    if (aimCommand != null) {
      aimCommand.cancel();
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
      Map.entry(3.153828, 2100.0), // TRENCH
      Map.entry(4.792132, 2350.0)); // OUTPOST

  // meters, RPM
  private static final InterpolatingDoubleTreeMap PASS_SHOOTING_SPEED_BY_DISTANCE = InterpolatingDoubleTreeMap
      .ofEntries(
          Map.entry(5.281523, 1700.0), // Close bump
          Map.entry(7.883727, 2300.0), // Midfield
          Map.entry(10.29897, 2800.0), // Far bump
          Map.entry(14.07683, 3300.0)); // Far wall

  // meters, degrees
  private static final InterpolatingDoubleTreeMap HOOD_ANGLE_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.554228, 73.0), // HUB
      Map.entry(3.153828, 54.5), // TRENCH
      Map.entry(4.792132, 51.5)); // OUTPOST
}
