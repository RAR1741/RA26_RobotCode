package frc.robot.subsystems;

import frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;
  private final SwivelSubsystem swivel;
  private final AngleChangerSystem angleChanger;
  private final KickerSubsystem kicker;
  private final HopperSubsystem hopper;

  @SuppressWarnings("unused")
  private final boolean isShooter; // This will be used to determine if the shooter is at the correct speed for
                                   // firing, can be used in an auto command to wait until the shooter is ready
                                   // before firing
                                   // then why is this final? whatt??

  @SuppressWarnings("unused")
  private AngularVelocity targetShooterSpeed;
  @SuppressWarnings("unused")
  private Angle targetTurretYaw;
  private Angle targetTurretPitch;

  public Superstructure() {
    // Initialize subsystems here if needed
    this.intake = new IntakeSubsystem();
    this.hopper = new HopperSubsystem();
    this.kicker = new KickerSubsystem();
    this.swivel = new SwivelSubsystem();
    this.angleChanger = new AngleChangerSystem();
    this.shooter = new ShooterSubsystem();

    this.isShooter = false;
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return intake;
  }

  public ShooterSystem getShooterSubsystem() {
    return shooter;
  }

  public SwivelSystem getSwivelSubsystem() {
    return swivel;
  }

  public AngleChangerSystem getAngleChangerSubsystem() [
    return angleChanger;
  ]

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

  public Command aimDynamic(Supplier<AngularVelocity> shooterSpeedSupplier, Supplier<Angle> turretSupplier) {
    return Commands.parallel(
        shooter.setSpeedDynamic(shooterSpeedSupplier).asProxy(),
        turret.setAngleDynamic(turretAngleSupplier).asProxy())
        .withName("Superstructure.aimDynamic");
  }
}
