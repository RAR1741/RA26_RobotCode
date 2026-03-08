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

    // double hoodSpeed = 0.5;

    // controller.x().onTrue(Commands.runOnce(() -> {
    // hood.setControl(new DutyCycleOut(hoodSpeed));
    // })).onFalse(Commands.runOnce(() -> {
    // hood.setControl(new DutyCycleOut(0));
    // }));

    // controller.y().onTrue(Commands.runOnce(() -> {
    // hood.setControl(new DutyCycleOut(-hoodSpeed));
    // })).onFalse(Commands.runOnce(() -> {
    // hood.setControl(new DutyCycleOut(0));
    // }));
  }
}
