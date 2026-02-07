package frc.robot;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.SwerveSystem;

public class Simulation {
    private SimulatedArena m_arena;
    private SwerveSystem m_swerve;
    private SwerveDriveSimulation m_sim;

    public Simulation(SwerveSystem swerveDrive){
        m_swerve = swerveDrive;
        m_arena = SimulatedArena.getInstance();
        m_arena.resetFieldForAuto();
        Logger.recordOutput("FieldSimulation/Fuel", 
            m_arena.getGamePiecesArrayByType("Fuel")
        );
    }

    public void init(){
        m_sim = new SwerveDriveSimulation(
                Constants.SimulationConstants.k_config,
                new Pose2d(3, 3, new Rotation2d())
            );

//        SimulatedArena.getInstance().addDriveTrainSimulation(m_sim);
    }

    public void periodic(){
        m_arena.simulationPeriodic();
        Logger.recordOutput("Sim/RobotPose", m_swerve.getSwerveDrive().getPose());
        System.out.println("" + m_swerve.getSwerveDrive().getPose());
    }
}
