package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSystem;
import swervelib.SwerveInputStream;
import frc.robot.Constants;

public class DriverController {
    private static CommandXboxController controller;

    public static void configure(int port, SwerveSystem drivetrain) {
        controller = new CommandXboxController(port);
    
        SwerveInputStream driveInputStream = SwerveInputStream.of(drivetrain.getSwerveDrive(),
            () -> controller.getLeftY() * -1,
            () -> controller.getLeftX() * -1)
            .withControllerRotationAxis(() -> controller.getRightX() * -1)
            .robotRelative(false)
            .allianceRelativeControl(true)
            // .scaleTranslation(0.25) // TODO: Tune speed scaling
            .deadband(Constants.ControllerConstants.k_DEADBAND);

        drivetrain.setDriveInputStream(driveInputStream);
    }

    public static CommandXboxController getController(){
        return controller;
    }
}
