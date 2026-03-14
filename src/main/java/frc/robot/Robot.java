// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.auto.AutoChooser;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private Simulation sim;

  private final RobotContainer m_robotContainer;

  public Robot() {
    Logger.recordMetadata("ProjectName", "RA26_RobotCode");
    Logger.addDataReceiver(new NT4Publisher());

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
    }

    Logger.start();

    m_robotContainer = new RobotContainer();

    configureAutos();
  }

  public void configureAutos() {
    // AutoChooser autoChooser = new AutoChooser();

    // autoChooser.addRoutine("Test Routine", m_robotContainer::getTestAuto);
    // // autoChooser.addCmd("Example Auto Command",
    // // m_robotContainer::exampleAutoCommand);

    // AutoChooser autoChooser = new AutoChooser();

    // // Put the auto chooser on the dashboard
    // SmartDashboard.putData(autoChooser);

    // // Schedule the selected auto during the autonomous period
    // RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println("=================== Selected Auto: " + m_autonomousCommand.getName());

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    CommandScheduler.getInstance().schedule(m_robotContainer.getHoodHomeCommand());
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationInit() {
    // TODO: reimplement this when we have a simulation to run
    sim = new Simulation(m_robotContainer.getSwerveSystem());
    sim.init();
  }

  @Override
  public void simulationPeriodic() {
    sim.periodic();
  }
}
