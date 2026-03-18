package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;

public class OperatorControls {
  public static void configure(int port, CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    controller.a().whileTrue(superstructure.feedAllCommand());

    controller.rightTrigger().whileTrue(superstructure.shootCommand());

    controller.x().whileTrue(superstructure.intakeCommand());

    controller.y().onTrue(superstructure.hoodUpCommand());

    controller.b().onTrue(superstructure.hoodDownCommand());

    // controller.start().onTrue(superstructure.hoodHomeSequence());
    controller.start().onTrue(superstructure.ejectAllFuel());

    // Intake Pivot Rezero (for testing, not intended for driver use)
    controller.back().onTrue(superstructure.intakeRezero().ignoringDisable(true));

    // Intake pivot deployment
    controller.povLeft().whileTrue(superstructure.intakeStowCommand());
    controller.povRight().whileTrue(superstructure.intakeDeployCommand());
  }
}
