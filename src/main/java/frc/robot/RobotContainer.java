// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.controls.DriverController;
import frc.robot.subsystems.SwerveSystem;

public class RobotContainer {
  private final SwerveSystem m_swerve = new SwerveSystem();;

  public RobotContainer() {
    configureBindings();
    
    m_swerve.setDefaultCommand( 
      m_swerve.driveCommand( 
        DriverController.getController().getLeftY() * -1, 
        DriverController.getController().getLeftX() * -1, 
        DriverController.getController().getRightX() * -1 
      ) 
    );
  }

  private void configureBindings() {
    DriverController.configure(Constants.ControllerConstants.kDriverControllerPort, m_swerve);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
