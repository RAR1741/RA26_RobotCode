package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  private final IntakeSubsystem intake;
  private final HopperSubsystem hopper;
  private final KickerSubsystem kicker;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;
  private final ShooterSubsystem shooter;

  private final CommandSwerveDrivetrain drivetrain;

  private final LimeLightSubsystem limelight;

  private final boolean isShooter; // This will be used to determine if the shooter is at the correct speed for
                                   // firing, can be used in an auto command to wait until the shooter is ready
                                   // before firing

  private AngularVelocity targetShooterSpeed;
  private Angle targetTurretAngle;

  public Superstructure(CommandSwerveDrivetrain swerve) {
    this.drivetrain = swerve;

    // Initialize subsystems here if needed
    this.intake = new IntakeSubsystem();
    this.hopper = new HopperSubsystem();
    this.kicker = new KickerSubsystem();
    this.turret = new TurretSubsystem();
    this.hood = new HoodSubsystem();
    this.shooter = new ShooterSubsystem();

    this.limelight = new LimeLightSubsystem(drivetrain);

    this.isShooter = false;
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return intake;
  }

  public ShooterSubsystem getShooterSubsystem() {
    return shooter;
  }

  public TurretSubsystem getTurretSubsystem() {
    return turret;
  }

  public KickerSubsystem getKickerSubsystem() {
    return kicker;
  }

  public HopperSubsystem getHopperSubsystem() {
    return hopper;
  }

  public Command feedAllCommand() {
    return Commands.parallel(
        hopper.feedCommand().asProxy(),
        kicker.feedCommand().asProxy()).withName("Superstructure.feedAll");
  }

  public Command shootCommand() {
    return shooter.shoot().asProxy().withName("Superstructure.shoot");
  }

  public Command intakeCommand() {
    return intake.intakeCommand().asProxy().withName("Superstructure.intake");
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
    return hood.setAngle(hood.MIN_ANGLE).asProxy().withName("Superstructure.hoodUp");
  }

  public Command hoodDownCommand() {
    return hood.setAngle(hood.MAX_ANGLE).asProxy().withName("Superstructure.hoodDown");
  }

  public Command hoodHomeSequence() {
    return hood.homeSequence().asProxy().withName("Superstructure.hoodHome");
  }

  public Command turretLeftCommand() {
    return turret.setAngle(Degrees.of(160)).asProxy().withName("Superstructure.turretLeft");
  }

  public Command turretCenterCommand() {
    return turret.setAngle(Degrees.of(0)).asProxy().withName("Superstructure.turretCenter");
  }

  public Command turretRightCommand() {
    return turret.setAngle(Degrees.of(-160)).asProxy().withName("Superstructure.turretRight");
  }

  public Command turretRezeroCommand() {
    return turret.rezero().asProxy().withName("Superstructure.turretRezero");
  }

  // Aim at shooter for auto
  public Command aimCommand(AngularVelocity shooterSpeed, Angle turretAngle) {
    return Commands.runOnce(() -> {
      targetShooterSpeed = shooterSpeed;
      targetTurretAngle = turretAngle;
    }).andThen(
        Commands.parallel(
            shooter.setSpeed(shooterSpeed).asProxy(),
            turret.setAngle(turretAngle).asProxy())
            .withName("Superstructure.aimCommand"));
  }

  public void setShooterSetpoints(AngularVelocity shooterSpeed, Angle turretAngle) {
    targetShooterSpeed = shooterSpeed;
    targetTurretAngle = turretAngle;
  }

  public Command stopAllCommand() {
    return Commands.parallel(
        shooter.stopCommand().asProxy(),
        turret.setAngle(Degrees.of(0)).asProxy()).withName("Superstructure.stopAll");
  }

  public Command aimDynamic(Supplier<AngularVelocity> shooterSpeedSupplier, Supplier<Angle> turretAngleSupplier) {
    return Commands.parallel(
        shooter.setSpeedDynamic(shooterSpeedSupplier).asProxy(),
        turret.setAngleDynamic(turretAngleSupplier).asProxy())
        .withName("Superstructure.aimDynamic");
  }
}
