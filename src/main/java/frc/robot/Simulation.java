package frc.robot;

import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.SwerveSystem;

public class Simulation {
    private SimulatedArena m_arena;
    private SwerveSystem m_swerve;

    public Simulation(SwerveSystem swerveDrive) {
        m_swerve = swerveDrive;
        m_arena = SimulatedArena.getInstance();
        m_arena.resetFieldForAuto();
    }

    public void init() {
    }

    public void periodic() {
        m_arena.simulationPeriodic();
        Logger.recordOutput("FieldSimulation/Fuel",
                m_arena.getGamePiecesArrayByType("Fuel"));
        Logger.recordOutput("Sim/RobotPose", m_swerve.getSwerveDrive().getPose());
    }
}
