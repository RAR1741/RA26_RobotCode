// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveSystem;
import frc.robot.subsystems.LimelightSystem;
import frc.robot.subsystems.TurretSystem;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private Simulation sim;

  private SwerveSystem m_swerve;
  private LimelightSystem m_limelight;
  private TurretSystem m_TurretSystem;

  private Subsystem[] m_subsystems = {m_swerve, m_limelight, m_TurretSystem};

  private final RobotContainer m_robotContainer;

  public Robot() {
    Logger.recordMetadata("ProjectName", "RA26_RobotCode");
    Logger.addDataReceiver(new NT4Publisher());

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
    }

    Logger.start();

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    for (Subsystem system : m_subsystems){
      system.periodic();
    }
    CommandScheduler.getInstance().run(); //how is it going
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
    // sim = new Simulation(m_robotContainer.getSwerveSystem());
    // sim.init();
  }

  @Override
  public void simulationPeriodic() {
    sim.periodic();
  }
}
