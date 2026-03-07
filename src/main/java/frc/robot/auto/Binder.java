package frc.robot.auto;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;

public class Binder {
    private final AutoFactory m_factory;

    public Binder(AutoFactory m_factory2) {
        m_factory = m_factory2;
    }

    public void bind(String name, Command command) {
        m_factory.bind(name, command);
    }
}
