package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Simulation {
  private SimulatedArena m_arena;
  private CommandSwerveDrivetrain m_swerve;

  public Simulation(CommandSwerveDrivetrain swerveDrive) {
    m_swerve = swerveDrive;
    m_arena = SimulatedArena.getInstance();
    m_arena.resetFieldForAuto();
  }

  public void init() {
  }

  public void periodic() {
    m_arena.simulationPeriodic();

    // TODO: why does this error
    Logger.recordOutput("FieldSimulation/Fuel", m_arena.getGamePiecesArrayByType("Fuel"));

    Logger.recordOutput("Sim/RobotPose", m_swerve.getState().Pose);
  }
}
