package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants.k_gravitationalAcceleration;
import frc.robot.Constants.TurretConstants.k_turretHeight;
import frc.robot.Constants.TurretConstants.k_extraTimeToPassSensor;
import frc.robot.Constants.FieldConstants.k_hubRadius;
import frc.robot.Constants.FieldConstants.k_hubHeight;
import frc.robot.Constants.FieldConstants.k_hubX;
import frc.robot.Constants.FieldConstants.k_hubY;
import frc.robot.Constants.FieldConstants.k_fuelRadius;

public class TurretSystem extends SubsystemBase {


    public TurretSystem() {

    }

    // TODO add desmos graph link

    // launch geometry and calculations
    
    public static double launchVelocity(double launchAngle, double xDistance, double yDistance) {
        double divisor = 2.0 * cos(launchAngle) * (xDistance * sin(launchAngle) - yDistance * cos(launchAngle));
        double radicand = k_gravitationalAcceleration / divisor;
        return xDistance * Math.sqrt(radicand);
    }
    public static double upperLaunchAngle(double launchVelocity, double xDistance, double yDistance) {
        double distance = Math.hypot(xDistance, yDistance);
        double targetFactor = yDistance / distance;
        double dropFactor = k_gravitationalAcceleration * xDistance * xDistance / (launchVelocity * launchVelocity * distance);
        return 0.5 * (arccos(targetFactor + dropFactor) + arccos(-targetFactor));
    }
    public static double lowerLaunchAngle(double launchVelocity, double xDistance, double yDistance) {
        double distance = Math.hypot(xDistance, yDistance);
        double targetFactor = yDistance / distance;
        double dropFactor = k_gravitationalAcceleration * xDistance * xDistance / (launchVelocity * launchVelocity * distance);
        return 0.5 * (arcsin(targetFactor + dropFactor) + arcsin(targetFactor));
    }
    public static double maxLaunchHeight(double launchAngle, double launchVelocity) {
        double vYSquared = Math.pow(launchVelocity * sin(launchAngle), 2.0);
        return 0.5 * vYSquared / k_gravitationalAcceleration;
    }
    public static double absHeightAtDistance(double launchAngle, double launchVelocity, double testX) {
        double time = testX / (launchVelocity * cos(launchAngle));
        double launchFactor = time * launchVelocity * sin(launchAngle);
        double dropFactor = 0.5 * time * k_gravitationalAcceleration * k_gravitationalAcceleration;
        return k_turretHeight + launchFactor - dropFactor;
    }
    public static double secondTimeToAbsHeight(double launchAngle, double launchVelocity, double testHeight) {
        double VxSin = launchVelocity * sin(launchAngle);
        double radicand = VxSin * VxSin - 2 * k_gravitationalAcceleration * (testHeight - k_turretHeight);
        return (VxSin + Math.sqrt(radicand)) / k_gravitationalAcceleration;
    }
    public static double farDistanceToAbsHeight(double launchAngle, double launchVelocity, double testHeight) {
        return launchVelocity * cos(launchAngle) * secondTimeToAbsoluteHeight(launchAngle, launchVelocity, testHeight);
    }
    public static double trajectorySlopeAtDistance(double launchAngle, double launchVelocity, double testX) {
        double xVelocity = launchVelocity * cos(launchAngle);
        double time = testX / xVelocity;
        double yVelocity = launchVelocity * sin(launchAngle) - time * k_gravitationalAcceleration;
        return yVelocity / xVelocity;
    }
    public static boolean testValidHubTrajectory(double launchDirection, double launchAngle, double launchVelocity, 
                                                 double turretX, double turretY, double sideClearance, double verticalClearance) {
        // turret x, y, height should be measured as the location of the bottom of the fuel at the moment it is launched from the turret
        // weather permitting, air resistance may be neglected
        double distance = Math.hypot(k_hubX - turretX, k_hubY - turretY);
        double sideOffset = Math.abs(cos(launchDirection) * (k_hubY - turretY) - sin(launchDirection) * (k_hubX - turretX));
        double launchDistance = Math.sqrt(distance * distance - sideOffset * sideOffset);
        double nearEdgeDistance = launchDistance + hubNearEdgeOffset(launchDirection, sideOffset, 0.0);
        double entranceDistance = farDistanceToAbsoluteHeight(launchAngle, launchVelocity, k_hubHeight);
        double entranceHeight = absHeightAtDistance(launchAngle, launchVelocity, nearEdgeDistance);
        double maxHeight = 2.0 * k_fuelRadius + maxLaunchHeight(launchAngle, launchVelocity);
        double nearClearanceDistance = launchDistance - hubNearEdgeOffset(launchDirection, sideOffset, sideClearance);
        double farClearanceingDistance = launchDistance + hubFarEdgeOffset(launchDirection, sideOffset, sideClearance);
        boolean clearsHub = entranceHeight >= verticalClearance  // measured at the center of the fuel entering the space above the funnel
                         && entranceDistance >= nearClearanceDistance  // measured at the bottom of the fuel hitting 72 inches above the ground
                         && entranceDistance <= farClearanceDistance  // clearance treats the funnel opening as if it has been shrunk
                         && maxLaunchHeight <= k_ceilingHeight;  // it is admittedly unlikely that the turret will hit the ceiling, but still
        return clearsHub;
    }
    public static double timeToHubScoring(double launchAngle, double launchVelocity) {
        return k_extraTimeToPassSensor + secondTimeToAbsHeight(launchAngle, launchVelocity, k_hubHeight);
    }
    
    // field geometry and calculations
    
    public static double hubNearEdgeOffset(double approachAngle, double sideOffset, double clearance) {
        return hubFarEdgeOffset(approachAngle, -sideOffset, clearance); // Exploits the hub's hexagonal symmetry
    }                                                                   // Hexagons really are the bestagons...
    public static double hubFarEdgeOffset(double approachAngle, double sideOffset, double clearance) {
        double hubRadius = k_hubRadius - clearance * 2.0 / Math.sqrt(3.0);
        double[] pXArr = new double[4];
        pXArr[0] = hubRadius * cos(210 - approachAngle % 60);
        pXArr[1] = hubRadius * cos(150 - approachAngle % 60);
        pXArr[2] = hubRadius * cos(90 - approachAngle % 60);
        pXArr[3] = hubRadius * cos(30 - approachAngle % 60);
        int intersectingSide = 
            (sideOffset < pXArr[0] || sideOffset > pXArr[3])? NAN : 
                ((sideOffset < pXArr[1])? 0 : 
                    ((sideOffset < pXArr[2])? 1 : 2));
        double p1X = pXArr[intersectingSide];
        double p2X = pXArr[intersectingSide + 1];
        double p1Y = Math.sqrt(hubRadius * hubRadius - p1X * p1X);
        double p2Y = Math.sqrt(hubRadius * hubRadius - p2X * p2X);
        double p1Scale = (sideOffset - p2X) / (p1X - p2X);
        double p2Scale = (sideOffset - p1X) / (p2X - p1X);
        return p1Y * p1Scale + p2Y * p2Scale;
    }
    
    // math helpers
    
    public static double sin(double degrees) {return Math.sin(Math.toRadians(degrees));}
    public static double cos(double degrees) {return Math.cos(Math.toRadians(degrees));}
    public static double arcsin(double ratio) {return Math.toDegrees(Math.asin(ratio));}
    public static double arccos(double ratio) {return Math.toDegrees(Math.acos(ratio));}
}
