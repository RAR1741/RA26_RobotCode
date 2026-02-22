package frc.robot.auto.tasks;
import edu.wpi.first.wpilibj.Timer;

public class Wait extends Tasks {
    private Timer m_runningTimer = new Timer();
    private double m_time;

    public Wait(double time) {
        m_time = time;
    }

    @Override
    public void prepare() {
        m_runningTimer.reset();
        m_runningTimer.start();
        m_prepared = true;
    }

    @Override
    public void update() {
        logIsRunning(m_isFinished);
    }

    @Override
    public boolean isFinished() {
        return m_runningTimer.get() >= m_time;
    }

    @Override
    public void done(){
        logIsRunning(false);
    }
    
}
