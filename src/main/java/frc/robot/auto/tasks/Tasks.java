package frc.robot.auto.tasks;
import org.littletonrobotics.junction.Logger;

public abstract class Tasks {
    public boolean m_isFinished = false;
    public boolean m_prepared = false;

    public abstract void prepare();

    public abstract void update();

    public void updateSim() {
    }

    public abstract boolean isFinished();

    public void done() {
    };

    public void logIsRunning(boolean isRunning) {
        Logger.recordOutput("Auto/Tasks/" + this.getClass().getSimpleName(), isRunning);
    }

    public boolean isPrepared() {
        return m_prepared;
    }
}
