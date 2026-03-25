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
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterInstruction;

public class ShootOnTheMoveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Superstructure superstructure;

  private Supplier<Translation3d> aimPointSupplier; // The point to aim at
  private AngularVelocity latestShootSpeed = RPM.of(0);
  private Angle latestHoodAngle = Degrees.of(80.0);
  private Angle latestTurretAngle = Degrees.of(0.0);

  public ShootOnTheMoveCommand(CommandSwerveDrivetrain drivetrain, Superstructure superstructure,
      Supplier<Translation3d> aimPointSupplier) {
    this.drivetrain = drivetrain;
    this.superstructure = superstructure;
    this.aimPointSupplier = aimPointSupplier;
  }

  @Override
  public void initialize() {
    super.initialize();

    latestHoodAngle = superstructure.getHoodAngle();
    latestTurretAngle = superstructure.getTurretAngle();
    latestShootSpeed = superstructure.getShooterSpeed();

    // when this current command ends, we should probably cancel the dynamic
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
    Translation2d turretPosition = drivetrain.getState().Pose.getTranslation()
                                     .plus(superstructure.getShooterPose().toPose2d().getTranslation());
    Translation2d turretVelocity = TurretSubsystem.getTurretVelocity(ChassisSpeeds
    .fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation()), drivetrain.getState().Pose.getRotation().getRadians());

    ShooterInstruction instruction = ShooterInstruction.generateInstructionBareMinimumFunctional(turretPosition, turretVelocity, superstructure.getAimRotation3d(), superstructure.getShooterSpeed());
    superstructure.setShooterSetpoints(instruction.targetVelocity, instruction.targetYaw, instruction.targetPitch, instruction.doShoot);

    /*
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
    */

    // System.out.println("Shooting at distance: " + correctedDistance + " requires
    // speed: " + latestShootSpeed
    // + ", hood angle: " + latestHoodAngle + ", turret angle: " +
    // latestTurretAngle);
  }
}
