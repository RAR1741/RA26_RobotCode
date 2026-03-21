// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.fasterxml.jackson.core.format.MatchStrength;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Simulation sim;

  private final RobotContainer m_robotContainer;

  private final Field2d field = new Field2d();

  public Robot() {
    Logger.recordMetadata("ProjectName", "RA26_RobotCode");
    Logger.addDataReceiver(new NT4Publisher());

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
    }

    Logger.start();

    m_robotContainer = new RobotContainer();

    SmartDashboard.putData(field);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    field.setRobotPose(m_robotContainer.getSwerveSystem().getState().Pose);
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
    CommandScheduler.getInstance().schedule(m_robotContainer.getHoodHomeCommand());

    // Auto is scheduled automatically by AutoChooser via RobotModeTriggers
    if (RobotBase.isSimulation()) {
      m_robotContainer.resetPoseToAutoStart();
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
    if (DriverStation.getMatchType() == MatchType.None){
      CommandScheduler.getInstance().schedule(m_robotContainer.getHoodHomeCommand());    
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
    sim = new Simulation(m_robotContainer.getSwerveSystem());
    sim.init();
  }

  @Override
  public void simulationPeriodic() {
    sim.periodic();
  }
}
