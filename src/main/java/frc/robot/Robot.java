// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.StateConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class Robot extends LoggedRobot {
  private Simulation sim;

  private final RobotContainer m_robotContainer;

  private final Field2d field = new Field2d();

  private final LEDSubsystem leds;

  public Robot() {
    Logger.recordMetadata("ProjectName", "RA26_RobotCode");
    Logger.addDataReceiver(new NT4Publisher());

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
    }

    Logger.start();

    StateConstants.initConstants();

    m_robotContainer = new RobotContainer();

    this.leds = m_robotContainer.getLEDs();
    SmartDashboard.putData(field);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    field.setRobotPose(m_robotContainer.getSwerveSystem().getState().Pose);
  }

  @Override
  public void disabledInit() {
    leds.setAllSolidColor(LEDConstants.disableColor).execute(); //we are criminals
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().schedule(leds.setAllSolidColor(LEDConstants.autoColor));
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
    CommandScheduler.getInstance().schedule(leds.setAllSolidColor(LEDConstants.teleColor));
    if (DriverStation.getMatchType() == MatchType.None){
      CommandScheduler.getInstance().schedule(m_robotContainer.getHoodHomeCommand());    
    }
  }

  @Override
  public void teleopPeriodic() {
    logMatchInformation();
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

  public void logMatchInformation() {
    String gameData = DriverStation.getGameSpecificMessage();
    double timeLeftinMatch = DriverStation.getMatchTime();
    Alliance ourAlliance = DriverStation.getAlliance().orElse(null);
    boolean isActive = false;

    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        // Blue is inactive first
        case 'B':
          isActive = (ourAlliance != null && ourAlliance == Alliance.Red);
          break;

        // Red is inactive first
        case 'R':
          isActive = (ourAlliance != null && ourAlliance == Alliance.Blue);
          break;

        default:
          break;
      }
    }

    String logKey = "MatchInfo/isActive";

    // Transition Shift
    if (timeLeftinMatch <= 140 && timeLeftinMatch > 130) {
      Logger.recordOutput(logKey, true);
    }

    // Shift One
    else if (timeLeftinMatch <= 130 && timeLeftinMatch > 105) {
      Logger.recordOutput(logKey, isActive);
    }

    // Shift Two
    else if (timeLeftinMatch <= 105 && timeLeftinMatch > 80) {
      Logger.recordOutput(logKey, !isActive);
    }

    // Shift Three
    else if (timeLeftinMatch <= 80 && timeLeftinMatch > 55) {
      Logger.recordOutput(logKey, isActive);
    }

    // Shift Four
    else if (timeLeftinMatch <= 55 && timeLeftinMatch > 30) {
      Logger.recordOutput(logKey, !isActive);
    }

    // Endgame
    else if (timeLeftinMatch <= 30) {
      Logger.recordOutput(logKey, true);
    }
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
