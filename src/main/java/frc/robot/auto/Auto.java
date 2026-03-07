package frc.robot.auto;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Auto {
    private final Binder m_Binder;
    private final CustomAutoChooser m_Chooser;
    private final CustomAutoFactory m_Factory;
    private final CommandSwerveDrivetrain m_CommandSwerveDrivetrain;

    public Auto() {
        m_CommandSwerveDrivetrain = new CommandSwerveDrivetrain(null, null);
    }
}
