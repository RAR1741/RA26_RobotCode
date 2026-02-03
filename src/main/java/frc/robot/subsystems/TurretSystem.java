package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class TurretSystem extends SubsystemBase {


    public TurretSystem() {

    }

    @Override
    public void periodic(){
        
    }

    public static double calculateLaunchVelocity(double launchAngle, double xDisplacement, double yDisplacement) {
        double cos = Math.cos(Math.toRadians(launchAngle));
        double sin = Math.sin(Math.toRadians(launchAngle));
        double divisor = 2.0 * cos * (xDisplacement * sin - yDisplacement * cos);
        double radicand = Constants.TurretConstants.k_gravitationalAcceleration / divisor;
        return xDisplacement * Math.sqrt(radicand);
    }
    public static double calculateLaunchAngle(double launchVelocity, double xDisplacement, double yDisplacement) {
        return 0.0;
    }
    public static double calculateMaxHeightFromLaunch(double launchAngle, double launchVelocity) {
        double vYSquared = Math.pow(launchVelocity * Math.sin(Math.toRadians(launchAngle)), 2.0);
        double twoG = 2.0 * Constants.TurretConstants.k_gravitationalAcceleration;
        return vYSquared / twoG;
    }
}