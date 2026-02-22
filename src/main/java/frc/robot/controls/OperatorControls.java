package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OperatorControls {
    public static void configure() {
        @SuppressWarnings("unused")
        CommandXboxController controller = new CommandXboxController(1);
    }
}
