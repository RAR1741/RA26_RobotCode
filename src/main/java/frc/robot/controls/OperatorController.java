package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.SwerveSubsystem;

public class OperatorController {
    public static void configure(int port) {
        @SuppressWarnings("unused")
        CommandXboxController controller = new CommandXboxController(port);

        // controller.rightBumper()
        // .onTrue()                   -Work in progress, will be used for intake control

        //controller.rightBumper().whileTrue(superstructure.setIntakeDeployAndRoll().withName("OperatorControls.intakeDeployed"));
    }
}
