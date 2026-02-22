package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;


public class TurretSystem extends SubsystemBase {


    public TurretSystem() {

    }

    public static double calculateLaunchVelocity(double launchAngle, double xDisplacement, double yDisplacement) {
        double cos = Math.cos(Math.toRadians(launchAngle));
        double sin = Math.sin(Math.toRadians(launchAngle));
        double divisor = 2.0 * cos * (xDisplacement * sin - yDisplacement * cos);
        double radicand = Constants.TurretConstants.k_gravitationalAcceleration / divisor;
        double velocity = xDisplacement * Math.sqrt(radicand);

        // Log inputs and intermediate values
        Logger.recordOutput("Turret/launchAngle_deg", launchAngle);
        Logger.recordOutput("Turret/xDisp_m", xDisplacement);
        Logger.recordOutput("Turret/yDisp_m", yDisplacement);
        Logger.recordOutput("Turret/launch/divisor", divisor);
        Logger.recordOutput("Turret/launch/radicand", radicand);
        Logger.recordOutput("Turret/launchVelocity_mps", velocity);

        return velocity;
    }
    public static double calculateLaunchAngle(double launchVelocity, double xDisplacement, double yDisplacement) {
        // Placeholder implementation; log inputs and return 0.0 for now
        Logger.recordOutput("Turret/launchVelocity_mps", launchVelocity);
        Logger.recordOutput("Turret/xDisp_m", xDisplacement);
        Logger.recordOutput("Turret/yDisp_m", yDisplacement);
        Logger.recordOutput("Turret/launchAngle_deg", 0.0);
        return 0.0;
    }
    public static double calculateMaxHeightFromLaunch(double launchAngle, double launchVelocity) {
        double vYSquared = Math.pow(launchVelocity * Math.sin(Math.toRadians(launchAngle)), 2.0);
        double twoG = 2.0 * Constants.TurretConstants.k_gravitationalAcceleration;
        double maxHeight = vYSquared / twoG;

        Logger.recordOutput("Turret/launchAngle_deg", launchAngle);
        Logger.recordOutput("Turret/launchVelocity_mps", launchVelocity);
        Logger.recordOutput("Turret/maxHeight_m", maxHeight);

        return maxHeight;
    }
}