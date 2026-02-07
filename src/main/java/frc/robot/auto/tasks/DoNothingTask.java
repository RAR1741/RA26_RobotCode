package frc.robot.auto.tasks;

import frc.robot.Telemetry;

public class DoNothingTask extends Tasks {
    @Override
    public void prepare(){
        Telemetry.logString("DoNothingTask", "Starting do nothing auto...");
        m_prepared = true;
    }
    
    @Override
    public void updateSim(){
        logIsRunning(true);

        Telemetry.logString("DoNothingTask", "Do nothing auto complete");
    }

    @Override
    public void update(){
        logIsRunning(true);

        Telemetry.logString("DoNothingTask", "Do nothing auto complete");
    }

    @Override
    public void done(){
        logIsRunning(false);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
