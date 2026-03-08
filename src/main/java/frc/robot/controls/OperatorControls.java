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

    controller.y().whileTrue(superstructure.hoodUpCommand());

    controller.b().whileTrue(superstructure.hoodDownCommand());
  }
}
