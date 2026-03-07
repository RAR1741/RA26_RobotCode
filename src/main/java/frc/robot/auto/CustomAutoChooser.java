package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj2.command.Command;

public class CustomAutoChooser {
    private final AutoChooser autoChooser;

    public CustomAutoChooser() {
        autoChooser = new AutoChooser();
    }

    public void createAndRegister(String name, Command command) {
        autoChooser.addCmd(name, () -> command);
    }

    public void createAndRegister(String name, AutoRoutine routine) {
        autoChooser.addRoutine(name, () -> routine);
    }

    public AutoChooser getAutoChooser() {
        return autoChooser;
    }
}
