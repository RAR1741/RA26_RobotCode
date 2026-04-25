package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootOnTheMoveCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.Command;

public class OperatorControls {
  public static boolean shooting = false;

  private static Command Sotm;
  private static boolean inZone = false;
  private static boolean operatorOveride = false;

  public static void configure(int port, CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    CommandXboxController controller = new CommandXboxController(port);

    // controller.rightTrigger().whileTrue(superstructure.shootCommand());

    // controller.x().whileTrue(superstructure.intakeDeployAndRun());

    // controller.y().onTrue(superstructure.hoodUpCommand());

    // controller.b().onTrue(superstructure.hoodDownCommand());

    controller.start().onTrue(superstructure.hoodHomeSequence());
    controller.back().onTrue(superstructure.intakeHomeSequence());

    // Intake Pivot Rezero (for testing, not intended for driver use)
    // controller.back().onTrue(superstructure.intakeRezero().ignoringDisable(true));

    // Comp controls
    controller.start().onTrue(superstructure.turretRezeroCommand().ignoringDisable(true));

    controller.a().onTrue(
        Commands.runOnce(() -> {
          if (!inZone && !shooting) {
            Sotm = new ShootOnTheMoveCommand(
                drivetrain,
                superstructure,
                () -> superstructure.getAimPoint())
                .ignoringDisable(true)
                .withName("OperatorControls.aimCommand");

            Sotm.schedule();
            shooting = true;
            operatorOveride = true;
          }
        }));

    controller.y().onTrue(Commands.parallel(
        superstructure.turretCenterCommand(),
        Commands.runOnce(() -> {
          if (!inZone && Sotm != null) {
            Sotm.end(true);
            shooting = false;
            operatorOveride = true;
          } else {
          }
        })));

    superstructure.getStateManager().inDecapitationZoneTrigger.onTrue(
        Commands.runOnce(() -> {
          if (shooting) {
            Sotm.end(true);
            operatorOveride = false;
          }
          inZone = true;
        }));

    superstructure.getStateManager().inDecapitationZoneTrigger.onFalse(
        Commands.runOnce(() -> {
          if (shooting && !operatorOveride) {
            Sotm = new ShootOnTheMoveCommand(
                drivetrain,
                superstructure,
                () -> superstructure.getAimPoint())
                .ignoringDisable(true)
                .withName("OperatorControls.aimCommand");

            Sotm.schedule();
            operatorOveride = true;
          }
          inZone = false;
        }));

    controller.rightTrigger().whileTrue(superstructure.feedAllCommand());
    // controller.rightTrigger().whileTrue(superstructure.feedAllCommandNoSafety());

    // Intake pivot deployment
    controller.leftBumper().whileTrue(superstructure.intakeStowCommand());
    controller.rightBumper().whileTrue(superstructure.intakeDeployCommand());
    controller.povUp().whileTrue(superstructure.intakeJostleCommand());

    controller.leftTrigger().whileTrue(superstructure.intakeCommand());
    // controller.back().onTrue(superstructure.intakeDeployAndRun().ignoringDisable(true));

    controller.povDown().whileTrue(superstructure.ejectAllFuel());
  }
}
