package frc.robot.controls;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;

public class OperatorControls {
  public static void configure(int port, CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    // controller.a().onTrue(Commands.runOnce(() -> {
    // hopperFloor.set(0.5);
    // kicker.set(0.5);
    // })).onFalse(Commands.runOnce(() -> {
    // hopperFloor.set(0);
    // kicker.set(0);
    // }));

    controller.a().whileTrue(superstructure.feedAllCommand());

    // controller.b().onTrue(Commands.runOnce(() -> {
    // shooterPrimary.setControl(new DutyCycleOut(0.45));
    // // shooterPrimary.setControl(new DutyCycleOut(0.67));
    // })).onFalse(Commands.runOnce(() -> {
    // shooterPrimary.setControl(new DutyCycleOut(0));
    // }));

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
