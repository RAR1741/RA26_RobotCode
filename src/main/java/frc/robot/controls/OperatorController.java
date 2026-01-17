package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OperatorController {
    public static void configure() {
        @SuppressWarnings("unused")
        CommandXboxController controller = new CommandXboxController(1);
    }
}
