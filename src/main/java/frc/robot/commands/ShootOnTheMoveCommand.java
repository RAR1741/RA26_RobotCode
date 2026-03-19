package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AimPoints;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;

public class ShootOnTheMoveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Superstructure superstructure;

  private Supplier<Translation3d> aimPointSupplier; // The point to aim at
  private AngularVelocity latestShootSpeed;
  private Angle latestHoodAngle;
  private Angle latestTurretAngle;

  public ShootOnTheMoveCommand(CommandSwerveDrivetrain drivetrain, Superstructure superstructure,
      Supplier<Translation3d> aimPointSupplier) {
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.aimPointSupplier = aimPointSupplier;

    // We use the drivetrain to determine linear velocity, but don't require it for
    // control. We
    // will be using the superstructure to control the shooting mechanism so it's a
    // requirement.
    // addRequirements(superstructure);

    // TODO: figure out if the above is actually required. Right now, when you start
    // some other command, the auto aim can't start back up again
  }

  @Override
  public void initialize() {
    super.initialize();

    latestHoodAngle = superstructure.getHoodAngle();
    latestTurretAngle = superstructure.getTurretAngle();
    latestShootSpeed = superstructure.getShooterSpeed();

    // SmartDashboard.putNumber("ShootOnTheMove/ShooterSpeedRPM",
    // latestShootSpeed.in(RPM));
    // SmartDashboard.putNumber("ShootOnTheMove/TurretAngleDeg",
    // latestTurretAngle.in(Degrees));
    // SmartDashboard.putNumber("ShootOnTheMove/HoodAngleDeg",
    // latestHoodAngle.in(Degrees));

    // TODO: when this current command ends, we should probably cancel the dynamic
    // aim command
    CommandScheduler.getInstance().schedule(superstructure.aimDynamicCommand(
        () -> {
          return this.latestShootSpeed;
        },
        () -> {
          return this.latestTurretAngle;
        },
        () -> {
          return this.latestHoodAngle;
        }));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {
    // Calculate trajectory to aimPoint
    var target = aimPointSupplier.get();
    Logger.recordOutput("ShootOnTheMove/rawTarget", target);

    var shooterLocation = drivetrain.getState().Pose.getTranslation()
        .plus(superstructure.getShooterPose().toPose2d().getTranslation());

    // Ignore this parameter for now, the range tables will account for it :/
    // var deltaH = target.getMeasureZ().minus(shooterLocation.getMeasureZ());
    var shooterOnGround = new Translation2d(shooterLocation.getX(), shooterLocation.getY());
    var targetOnGround = new Translation2d(target.getX(), target.getY());

    var distanceToTarget = Meters.of(shooterOnGround.getDistance(targetOnGround));

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
    var correctiveVector3d = new Translation3d(correctiveVector.getX(), correctiveVector.getY(), 0);

    Logger.recordOutput("FieldSimulation/AimTargetCorrected",
        new Pose3d(target.plus(correctiveVector3d), Rotation3d.kZero));

    var correctedTarget = targetOnGround.plus(correctiveVector);
    var vectorToTarget = shooterLocation.minus(correctedTarget);

    var correctedDistance = Meters.of(vectorToTarget.getNorm());
    var calculatedHeading = vectorToTarget.getAngle()
        .rotateBy(drivetrain.getState().Pose.getRotation().unaryMinus())
        .getMeasure();

    Logger.recordOutput("ShootOnTheMove/RobotHeading", drivetrain.getState().Pose.getRotation().getDegrees());
    Logger.recordOutput("ShootOnTheMove/DesiredTurretHeading", calculatedHeading.in(Degrees));
    Logger.recordOutput("ShootOnTheMove/distanceToTarget", distanceToTarget);

    latestTurretAngle = calculatedHeading;
    latestShootSpeed = calculateRequiredShooterSpeed(correctedDistance);
    latestHoodAngle = calculateRequiredHoodAngle(correctedDistance);

    // FORE TESTING - DO NOT USE
    // latestShootSpeed =
    // RPM.of(Double.parseDouble(DriverStation.getGameSpecificMessage()));

    // latestShootSpeed = RPM.of(2400);
    // latestHoodAngle =
    // Degrees.of(Double.parseDouble(DriverStation.getGameSpecificMessage()));
    // latestHoodAngle = superstructure.getHoodAngle();

    // Smartdashboard testing:
    // latestShootSpeed =
    // RPM.of(SmartDashboard.getNumber("ShootOnTheMove/ShooterSpeedRPM",
    // latestShootSpeed.in(RPM)));
    // latestTurretAngle = Degrees
    // .of(SmartDashboard.getNumber("ShootOnTheMove/TurretAngleDeg",
    // latestTurretAngle.in(Degrees)));
    // latestHoodAngle =
    // Degrees.of(SmartDashboard.getNumber("ShootOnTheMove/HoodAngleDeg",
    // latestHoodAngle.in(Degrees)));

    superstructure.setShooterSetpoints(
        latestShootSpeed,
        latestTurretAngle,
        latestHoodAngle);

    // System.out.println("Shooting at distance: " + correctedDistance + " requires
    // speed: " + latestShootSpeed
    // + ", hood angle: " + latestHoodAngle + ", turret angle: " +
    // latestTurretAngle);
  }

  private double getFlightTime(Distance distanceToTarget) {
    // Simple linear approximation based on empirical data.
    return TIME_OF_FLIGHT_BY_DISTANCE.get(distanceToTarget.in(Meters));
  }

  private AngularVelocity calculateRequiredShooterSpeed(Distance distanceToTarget) {
    // If we're firing at the hub, use the distance to determine the hood angle
    if (superstructure.getAimPoint() == AimPoints.getAllianceHubPosition()) {
      return RPM.of(HUB_SHOOTING_SPEED_BY_DISTANCE.get(distanceToTarget.in(Meters)));
    }

    return RPM.of(PASS_SHOOTING_SPEED_BY_DISTANCE.get(distanceToTarget.in(Meters)));
  }

  private Angle calculateRequiredHoodAngle(Distance distanceToTarget) {
    // If we're firing at the hub, use the distance to determine the hood angle
    if (superstructure.getAimPoint() == AimPoints.getAllianceHubPosition()) {
      return Degrees.of(HOOD_ANGLE_BY_DISTANCE.get(distanceToTarget.in(Meters)));
    }

    // Default hood angle if we're not aiming at the hub
    // return Degrees.of(45.0);
    return Degrees.of(40.0);
  }

  // meters, seconds
  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.0, 1.2),
      Map.entry(5.208015, 1.2),
      Map.entry(14.07683, 3.5));

  // meters, RPM
  private static final InterpolatingDoubleTreeMap HUB_SHOOTING_SPEED_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.2319, 2500.0), // HUB
      Map.entry(3.319674, 2600.0), // TRENCH
      Map.entry(5.208015, 2600.0), // OUTPOST
      Map.entry(7.866163, 2600.0)); // Midfield

  // meters, RPM
  private static final InterpolatingDoubleTreeMap PASS_SHOOTING_SPEED_BY_DISTANCE = InterpolatingDoubleTreeMap
      .ofEntries(
          Map.entry(5.281523, 1900.0), // Close bump
          Map.entry(7.883727, 2500.0), // Midfield
          Map.entry(10.29897, 3000.0), // Far bump
          Map.entry(14.07683, 3500.0)); // Far wall

  // meters, degrees
  private static final InterpolatingDoubleTreeMap HOOD_ANGLE_BY_DISTANCE = InterpolatingDoubleTreeMap.ofEntries(
      Map.entry(1.2319, 80.0), // HUB
      Map.entry(3.319674, 70.0), // TRENCH
      Map.entry(5.208015, 55.0)); // OUTPOST

}
