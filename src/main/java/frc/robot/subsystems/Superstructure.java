package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final KickerSubsystem kickerSubsystem;
    private final HopperSubsystem hopperSubsystem;

    private final boolean isShooter; // This will be used to determine if the shooter is at the correct speed for
                                     // firing, can be used in an auto command to wait until the shooter is ready
                                     // before firing

    private AngularVelocity targetShooterSpeed;
    private Angle targetTurretAngle;

    public Superstructure() {
        // Initialize subsystems here if needed

        this.intakeSubsystem = new IntakeSubsystem();
        this.shooterSubsystem = new ShooterSubsystem();
        this.turretSubsystem = new TurretSubsystem();
        this.kickerSubsystem = new KickerSubsystem();
        this.hopperSubsystem = new HopperSubsystem();
        this.isShooter = false;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }

    public TurretSubsystem getTurretSubsystem() {
        return turretSubsystem;
    }

    public KickerSubsystem getKickerSubsystem() {
        return kickerSubsystem;
    }

    public HopperSubsystem getHopperSubsystem() {
        return hopperSubsystem;
    }

    // Aim at shooter for auto
    public Command aimCommand(AngularVelocity shooterSpeed, Angle turretAngle) {
        return Commands.runOnce(() -> {
            targetShooterSpeed = shooterSpeed;
            targetTurretAngle = turretAngle;
        }).andThen(
                Commands.parallel(
                        shooterSubsystem.setSpeed(shooterSpeed).asProxy(),
                        turretSubsystem.setAngle(turretAngle).asProxy())
                        .withName("Superstructure.aimCommand"));
    }

    public void setShooterSetpoints(AngularVelocity shooterSpeed, Angle turretAngle) {
        targetShooterSpeed = shooterSpeed;
        targetTurretAngle = turretAngle;
    }

    public Command stopAllCommand() {
        return Commands.parallel(
                shooterSubsystem.stopCommand().asProxy(),
                turretSubsystem.setAngle(Degrees.of(0)).asProxy()).withName("Superstructure.stopAll");
    }

    public Command aimDynamic(Supplier<AngularVelocity> shooterSpeedSupplier, Supplier<Angle> turretAngleSupplier) {
        return Commands.parallel(
                shooterSubsystem.setSpeedDynamic(shooterSpeedSupplier).asProxy(),
                turretSubsystem.setAngleDynamic(turretAngleSupplier).asProxy())
                .withName("Superstructure.aimDynamic");
    }

}
