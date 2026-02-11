package frc.robot.auto.tasks;

import frc.robot.subsystems.SwerveSystem;
import frc.robot.Telemetry;

public class Brake extends Tasks {
    private SwerveSystem m_swerve;
    private boolean m_brake;
    private double m_speedX = 0.0;
    private double m_speedY = 0.0;

    public Brake(SwerveSystem swerve, boolean brake) {
        m_brake = brake;
        m_swerve = swerve;
    }

    @Override
    public void prepare() {
        m_prepared = true;

    }

    @Override
    public void update() {
        logIsRunning(true);
        if (m_brake){
            m_swerve.driveSpeedCommand(m_speedX, m_speedY, 0, false);
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {
        logIsRunning(false);
        Telemetry.logString("Brake Task", "Braking Done");
    }
}
