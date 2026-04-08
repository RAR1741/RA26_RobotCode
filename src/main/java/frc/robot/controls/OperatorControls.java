package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;

public class OperatorControls {
  public static boolean shooting = false;
  public static void configure(int port, CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    // controller.rightTrigger().whileTrue(superstructure.shootCommand());

    // controller.x().whileTrue(superstructure.intakeDeployAndRun());

    // controller.y().onTrue(superstructure.hoodUpCommand());

    // controller.b().onTrue(superstructure.hoodDownCommand());

    // controller.start().onTrue(superstructure.hoodHomeSequence());

    // Intake Pivot Rezero (for testing, not intended for driver use)
    // controller.back().onTrue(superstructure.intakeRezero().ignoringDisable(true));

    // Comp controls
    controller.y().onTrue(superstructure.turretCenterCommand());
    controller.start().onTrue(superstructure.turretRezeroCommand().ignoringDisable(true));

    controller.a().toggleOnTrue(
        new ShootOnTheMoveCommand(drivetrain, superstructure, () -> superstructure.getAimPoint())
            .ignoringDisable(true)
            .withName("OperatorControls.aimCommand"));

    controller.a().onTrue(Commands.runOnce(() -> {shooting = !shooting;}));

    controller.rightTrigger().whileTrue(superstructure.feedAllCommand());

    // Intake pivot deployment
    controller.leftBumper().whileTrue(superstructure.intakeStowCommand());
    controller.rightBumper().whileTrue(superstructure.intakeDeployCommand());

    controller.leftTrigger().whileTrue(superstructure.intakeCommand());
    // controller.back().onTrue(superstructure.intakeDeployAndRun().ignoringDisable(true));

    controller.povDown().whileTrue(superstructure.ejectAllFuel());
  }
}
