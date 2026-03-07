package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
        private final IntakeSubsystem intakeSystem;
        private final ShooterSubsystem shooterSystem;
        private final TurretSystem turretSystem;
        private final KickerSystem kickerSystem;
        private final HopperSubsystem hopperSystem;
        private final boolean isShooter; // This will be used to determine if the shooter is at the correct speed for firing, can be used in an auto command to wait until the shooter is ready before firing

        private AngularVelocity targetShooterSpeed;
        private Angle targetTurretAngle;

        public Superstructure() {
            // Initialize subsystems here if needed

            this.intakeSystem = new IntakeSubsystem();
            this.shooterSystem = new ShooterSubsystem();
            this.turretSystem = new TurretSystem();
            this.kickerSystem = new KickerSystem();
            this.hopperSystem = new HopperSubsystem();
            this.isShooter = false;
        }

        public IntakeSubsystem getIntakeSystem() {
            return intakeSystem;
        }

        public ShooterSubsystem getShooterSystem() {
            return shooterSystem;
        }

        public TurretSystem getTurretSystem() {
            return turretSystem;
        }

        public KickerSystem getKickerSystem() {
            return kickerSystem;
        }

        public HopperSubsystem getHopperSystem() {
            return hopperSystem;
        }

        //Aim at shooter for auto
        public Command aimCommand(AngularVelocity shooterSpeed, Angle turretAngle) {
            return Commands.runOnce(() -> {
            targetShooterSpeed = shooterSpeed;
            targetTurretAngle = turretAngle;
                }).andThen(
                    Commands.parallel(
                        shooterSystem.setSpeed(shooterSpeed).asProxy(),
                        turretSystem.setAngle(turretAngle).asProxy())
                    .withName("Superstructure.aimCommand"));
        }

        public void setShooterSetpoints(AngularVelocity shooterSpeed, Angle turretAngle){
            targetShooterSpeed = shooterSpeed;
            targetTurretAngle = turretAngle;
        }

        public Command stopAllCommand() {
            return Commands.parallel(
                shooterSystem.stopCommand().asProxy(),
                turretSystem.setAngle(Degrees.of(0)).asProxy()
                ).withName("Superstructure.stopAll");
        }

        public Command aimDynamic(Supplier<AngularVelocity> shooterSpeedSupplier, Supplier<Angle> turretAngleSupplier) {
            return Commands.parallel(
                shooterSystem.setSpeedDynamic(shooterSpeedSupplier).asProxy(),
                turretSystem.setAngleDynamic(turretAngleSupplier).asProxy())
                .withName("Superstructure.aimDynamic");
        }


}
