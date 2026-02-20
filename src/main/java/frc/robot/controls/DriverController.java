package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.Commands;
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

    public static void testGetControllerButtonA(){
        controller.a().onTrue(Commands.runOnce(() -> {
            System.out.println("Button A Pressed");
        }));       
    }
    
    public static void testGetControllerButtonB(){
        controller.b().onTrue(Commands.runOnce(() -> {
            System.out.println("Button B Pressed");
        }));
    }

    public static void testGetControllerButtonX(){
        controller.x().onTrue(Commands.runOnce(() -> {
            System.out.println("Button X Pressed");

        }));
    }
    public static void testGetControllerButtonY(){
        controller.y().onTrue(Commands.runOnce(() -> {
            System.out.println("Button Y Pressed");
        }));
    }

    public static void testGetControllerTriggerRight(){
        controller.rightTrigger().onTrue(Commands.runOnce(() -> {
            System.out.println("Trigger Right Pressed");
        }));
    }

    public static void testGetControllerTriggerLeft(){
        controller.leftTrigger().onTrue(Commands.runOnce(() -> {
            System.out.println("Trigger Left Pressed");
        }));
    }

    public static void testGetControllerBumperRight(){
        controller.rightBumper().onTrue(Commands.runOnce(() -> {
            System.out.println("Bumper Right Pressed");

        }));
    }
        
    public static void testGetControllerBumperLeft(){
        controller.leftBumper().onTrue(Commands.runOnce(() -> {
            System.out.println("Bumper Left Pressed");
        }));
    }
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    //why are we here?