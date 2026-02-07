package frc.robot;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

public class Simulation {
    public Simulation(){
        SimulatedArena.overrideInstance(SimulatedArena.getInstance());
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void periodic(){
        SimulatedArena.getInstance().simulationPeriodic();
    }
}
