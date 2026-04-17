package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.StateManager.StateManager;

public class Superstructure extends SubsystemBase {
  public final IntakeSubsystem intake;
  public final HopperSubsystem hopper;
  public final KickerSubsystem kicker;
  public final TurretSubsystem turret;
  public final HoodSubsystem hood;
  public final ShooterSubsystem shooter;
  //public final LEDSubsystem leds;

  public final CommandSwerveDrivetrain drivetrain;

  public final LimeLightSubsystem limelightUp;
  public final LimeLightSubsystem limelightDown;

  @SuppressWarnings("unused")
  private AngularVelocity targetShooterSpeed;
  @SuppressWarnings("unused")
  private Angle targetTurretAngle;
  @SuppressWarnings("unused")
  private Angle targetHoodAngle;

  // Trigger for readiness checks
  private final Trigger isReadyToShoot;

  public final StateManager stateManager;

  public Superstructure(CommandSwerveDrivetrain swerve) {
    stateManager = StateManager.initalize(swerve, this);

    this.drivetrain = swerve;

    // Initialize subsystems here if needed
    this.intake = new IntakeSubsystem();
    this.hopper = new HopperSubsystem();
    this.kicker = new KickerSubsystem();
    this.turret = new TurretSubsystem();
    this.hood = new HoodSubsystem(stateManager);
    this.shooter = new ShooterSubsystem();
    //this.leds = new LEDSubsystem();

    this.limelightUp = new LimeLightSubsystem(drivetrain,
        LimelightConstants.upName,
        LimelightConstants.upCameraOffset);

    this.limelightDown = new LimeLightSubsystem(drivetrain,
        LimelightConstants.downName,
        LimelightConstants.downCameraOffset);

    // Create triggers for checking if mechanisms are at their targets
    this.isReadyToShoot = stateManager.hasValidTarget
        .and(shooter.isAtTarget)
        .and(turret.isAtTarget)
        .and(hood.isAtTarget);
  }

  // public LEDSubsystem getLEDs(){
  //   return this.leds;
  // }

  public Command feedAllCommand() {
    return Commands.waitUntil(isReadyToShoot)
        .andThen(
            Commands.parallel(
                // intake.feedCommand(),
                hopper.feedCommand().asProxy(),
                kicker.feedCommand().asProxy())
                .onlyWhile(isReadyToShoot))
        .repeatedly()
        .withName("Superstructure.feedAll");
  }

  public Command feedAllAutoCommand() {
    return Commands.waitUntil(isReadyToShoot)
        .andThen(
            Commands.parallel(
                // intake.feedCommand(),
                hopper.feedCommand(),
                kicker.feedCommand())
                .onlyWhile(isReadyToShoot))
        .repeatedly()
        .withName("Superstructure.feedAllAuto");
  }

  // public Command shootCommand() {
  // return shooter.shoot().asProxy().withName("Superstructure.shoot");
  // }

  public Command intakeCommand() {
    return intake.intakeCommand().withName("Superstructure.intake");
  }

  public Command intakeDeployAndRun() {
    return intake.intakeDeployAndRun().asProxy().withName("Superstructure.intakeDeployAndRun");
  }

  public Command intakeStopCommand() {
    return intake.stopCommand().asProxy().withName("Superstructure.intakeStop");
  }

  public Command intakeRezero() {
    return intake.rezero().asProxy().withName("Superstructure.intakeRezero");
  }

  public Command intakeStowCommand() {
    return intake.setIntakeStow().asProxy().withName("Superstructure.intakeStow");
  }

  public Command intakeDeployCommand() {
    return intake.setIntakeDeployed().asProxy().withName("Superstructure.intakeDeploy");
  }

  public Command hoodUpCommand() {
    return hood.setAngle(Degree.of(65)).asProxy().withName("Superstructure.hoodUp");
  }

  public Command hoodDownCommand() {
    return hood.setAngle(hood.MAX_ANGLE).asProxy().withName("Superstructure.hoodDown");
  }

  public Command hoodHomeSequence() {
    return hood.homeSequence().withName("Superstructure.hoodHome");
  }

  public Command turretLeftCommand() {
    return turret.setAngle(Degrees.of(90)).asProxy().withName("Superstructure.turretLeft");
  }

  public Command turretCenterCommand() {
    return Commands.parallel(
        shooter.stopCommand(),
        hood.setAngle(hood.MAX_ANGLE),
        turret.setAngle(Degrees.of(0))).withName("Superstructure.turretCenter");
  }

  public Command turretRightCommand() {
    return turret.setAngle(Degrees.of(-90)).asProxy().withName("Superstructure.turretRight");
  }

  public Command turretRezeroCommand() {
    return turret.rezero().asProxy().withName("Superstructure.turretRezero");
  }

  public Command ejectAllFuel() {
    return Commands.parallel(
        intake.ejectCommand(),
        hopper.ejectCommand(),
        kicker.ejectCommand()).withName("Superstructure.ejectAllFuel");
  }

  public Command stopAllCommand() {
    return Commands.parallel(
        shooter.stopCommand().asProxy(),
        turret.setAngle(Degrees.of(0)).asProxy()).withName("Superstructure.stopAll");
  }

  public Command aimDynamicCommand(
      Supplier<AngularVelocity> shooterSpeedSupplier,
      Supplier<Angle> turretAngleSupplier,
      Supplier<Angle> hoodAngleSupplier) {
    return Commands.parallel(
        shooter.setSpeedDynamic(shooterSpeedSupplier),
        turret.setAngleDynamic(turretAngleSupplier),
        hood.setAngleDynamic(hoodAngleSupplier))
        .withName("Superstructure.aimDynamic");
  }

  public Translation2d getAimPoint() {
    return stateManager.getTargetPose().getTranslation();
  }

  public Angle getHoodAngle() {
    return hood.getAngle();
  }

  public Angle getTurretAngle() {
    return turret.getAngle();
  }

  public AngularVelocity getShooterSpeed() {
    return shooter.getSpeed();
  }

  public Pose3d getShooterPose() {
    // Position of the shooter relative to the "front" of the robot. Rotation
    // element is based on hood and turret angles
    return new Pose3d(turret.getTurretTranslation(), getAimRotation3d());
  }

  public Rotation3d getAimRotation3d() {
    // See
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    return new Rotation3d(
        Degrees.of(0), // no roll 🤞
        hood.getAngle().unaryMinus(), // pitch is negative hood angle
        turret.getRobotAdjustedAngle());
  }

  public StateManager getStateManager() {
    return stateManager;
  }

  public void setShooterSetpoints(AngularVelocity shooterSpeed, Angle turretAngle, Angle hoodAngle) {
    targetShooterSpeed = shooterSpeed;
    targetTurretAngle = turretAngle;
    targetHoodAngle = hoodAngle;
  }

  @Override
  public void periodic() {
    // Log SOTM ready states
    Logger.recordOutput("Superstructure/shooterReady", shooter.isAtTarget.getAsBoolean());
    Logger.recordOutput("Superstructure/turretReady", turret.isAtTarget.getAsBoolean());
    Logger.recordOutput("Superstructure/hoodReady", hood.isAtTarget.getAsBoolean());
    Logger.recordOutput("Superstructure/isReadyToShoot", isReadyToShoot.getAsBoolean());
  }

  public ShooterSubsystem getShooterSubsystem(){
    return shooter;
  }

  public TurretSubsystem getTurrenSubsystem(){
    return turret;
  }
}
