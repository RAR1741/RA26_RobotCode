package frc.robot;

import frc.robot.Constants.TurretConstants;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;

public class ParabolicTrajectory {
    public static String k_alliance = DriverStation.getAlliance().toString();

    public static double k_gravitationalAcceleration = TurretConstants.k_gravitationalAcceleration;
    public static double k_turretHeight = TurretConstants.k_turretHeight;
    public static double k_fieldWidth = FieldConstants.k_fieldWidth;
    public static double k_fieldLength = FieldConstants.k_fieldLength;
    public static double k_allianceZoneDepth = FieldConstants.k_allianceZoneDepth;
    public static double k_hubZoneDepth = FieldConstants.k_hubZoneDepth;
    public static double k_neutralZoneDepth = FieldConstants.k_neutralZoneDepth;
    public static double k_hubRadius = FieldConstants.k_hubRadius;
    public static double k_hubHeight = FieldConstants.k_hubHeight;
    public static double k_hubX = (k_alliance == "Red")? FieldConstants.k_redHubX : 
                                      (k_alliance == "Blue")? FieldConstants.k_blueHubX :
                                          Double.NaN;
    public static double k_hubY = FieldConstants.k_hubY;

    final double launchDirection;
    final double launchAngle;
    final double launchVelocity;
    final double launchX;
    final double launchY;
    final double launchHeight;

    public ParabolicTrajectory(double launchDirection, double launchAngle, double launchVelocity, double launchX, double launchY, double launchHeight) {
        this.launchDirection = launchDirection;
        this.launchAngle = launchAngle;
        this.launchVelocity = launchVelocity;
        this.launchX = launchX;
        this.launchY = launchY;
        this.launchHeight = launchHeight;
    }

    // add desmos graph link

    // launch geometry and calculations
    // constructors

    public static ParabolicTrajectory toXYHFromVXYHMinimizeAngle(double targetX, double targetY, double targetHeight, double launchX, 
                                                                 double launchY, double launchHeight, double launchVelocity) {
        double launchDirection = Math.atan2(k_hubY - launchY, k_hubX - launchX);
        double xDistance = Math.hypot(targetX - launchX, targetY - launchY);
        double yDistance = targetHeight - launchHeight;
        double launchAngle = solveLowerLaunchAngle(launchVelocity, xDistance, yDistance);
        if (launchAngle == Double.NaN) {return null;}
        if (launchAngle < TurretConstants.k_minLaunchAngle) {
            launchAngle = TurretConstants.k_minLaunchAngle;
            launchVelocity = solveLaunchVelocity(launchAngle, xDistance, yDistance);
            if (launchVelocity == Double.NaN) {return null;}
        } else if (launchAngle > TurretConstants.k_maxLaunchAngle) {
            launchAngle = TurretConstants.k_maxLaunchAngle;
            launchVelocity = solveLaunchVelocity(launchAngle, xDistance, yDistance);
            if (launchVelocity == Double.NaN) {return null;}
        }
        return new ParabolicTrajectory(launchDirection, launchAngle, launchVelocity, launchX, launchY, launchHeight);
    }
    public static ParabolicTrajectory toHubFromAXY(double launchAngle, double launchX, double launchY) {
        double xDistance = Math.hypot(k_hubX - launchX, k_hubY - launchY);
        double yDistance = k_hubHeight - k_turretHeight;
        double launchVelocity = solveLaunchVelocity(launchAngle, xDistance, yDistance);
        double launchDirection = Math.atan2(k_hubY - launchY, k_hubX - launchX);

        return new ParabolicTrajectory(launchDirection, launchAngle, launchVelocity, launchX, launchY, k_turretHeight);
    }
    public static ParabolicTrajectory toHubFromVXY(double launchVelocity, double launchX, double launchY) {
        double xDistance = Math.hypot(k_hubX - launchX, k_hubY - launchY);
        double yDistance = k_hubHeight - k_turretHeight;
        double launchAngle = solveUpperLaunchAngle(launchVelocity, xDistance, yDistance);
        double launchDirection = Math.toDegrees(Math.atan2(k_hubY - launchY, k_hubX - launchX));
        if (launchAngle == Double.NaN) {return null;}
        return new ParabolicTrajectory(launchDirection, launchAngle, launchVelocity, launchX, launchY, k_turretHeight);
    }
    public static ParabolicTrajectory toOppositeZoneFromXY(double launchX, double launchY, boolean isBlueTeam) { // chooses the zone closer to (0, 0), based on the coordinates inputted
        double targetX = isBlueTeam? k_allianceZoneDepth : k_fieldLength - k_allianceZoneDepth;
        double targetY = k_fieldWidth / 2.0 + () * (launchY > k_fieldWidth / 2.0? 1 : -1);
        ParabolicTrajectory trajectory = toXYHFromVXYHMinimizeAngle(targetX, targetY, )
    }
    
    public static double solveLaunchVelocity(double launchAngle, double xDistance, double yDistance) {
        double divisor = 2.0 * cos(launchAngle) * (xDistance * sin(launchAngle) - yDistance * cos(launchAngle));
        double radicand = k_gravitationalAcceleration / divisor;
        return xDistance * Math.sqrt(radicand);
    }
    public static double solveUpperLaunchAngle(double launchVelocity, double xDistance, double yDistance) {
        double distance = Math.hypot(xDistance, yDistance);
        double targetFactor = yDistance / distance;
        double dropFactor = k_gravitationalAcceleration * xDistance * xDistance / (launchVelocity * launchVelocity * distance);
        return 0.5 * (arccos(targetFactor + dropFactor) + arccos(-targetFactor));
    }
    public static double solveLowerLaunchAngle(double launchVelocity, double xDistance, double yDistance) {
        double distance = Math.hypot(xDistance, yDistance);
        double targetFactor = yDistance / distance;
        double dropFactor = k_gravitationalAcceleration * xDistance * xDistance / (launchVelocity * launchVelocity * distance);
        return 0.5 * (arcsin(targetFactor + dropFactor) + arcsin(targetFactor));
    }

    // instance methods

    public double maxLaunchHeight(double launchAngle, double launchVelocity) {
        double vYSquared = Math.pow(launchVelocity * sin(launchAngle), 2.0);
        return 0.5 * vYSquared / k_gravitationalAcceleration;
    }
    public double absHeightAtDistance(double launchAngle, double launchVelocity, double testX) {
        double time = testX / (launchVelocity * cos(launchAngle));
        double launchFactor = time * launchVelocity * sin(launchAngle);
        double dropFactor = 0.5 * time * k_gravitationalAcceleration * k_gravitationalAcceleration;
        return k_turretHeight + launchFactor - dropFactor;
    }
    public double secondTimeToAbsHeight(double launchAngle, double launchVelocity, double testHeight) {
        double VxSin = launchVelocity * sin(launchAngle);
        double radicand = VxSin * VxSin - 2.0 * k_gravitationalAcceleration * (testHeight - k_turretHeight);
        return (VxSin + Math.sqrt(radicand)) / k_gravitationalAcceleration;
    }
    public double farDistanceToAbsHeight(double launchAngle, double launchVelocity, double testHeight) {
        return launchVelocity * cos(launchAngle) * secondTimeToAbsHeight(launchAngle, launchVelocity, testHeight);
    }
    public double trajectorySlopeAtDistance(double launchAngle, double launchVelocity, double testX) {
        double xVelocity = launchVelocity * cos(launchAngle);
        double time = testX / xVelocity;
        double yVelocity = launchVelocity * sin(launchAngle) - time * k_gravitationalAcceleration;
        return yVelocity / xVelocity;
    }
    public boolean testValidHubTrajectory(double launchDirection, double launchAngle, double launchVelocity, 
                                                 double turretX, double turretY, double sideClearance, double verticalClearance) {
        // turret x, y, height should be measured as the location of the bottom of the fuel at the moment it is launched from the turret
        // weather permitting, air resistance may be neglected
        double distance = Math.hypot(k_hubX - turretX, k_hubY - turretY);
        double sideOffset = Math.abs(cos(launchDirection) * (k_hubY - turretY) - sin(launchDirection) * (k_hubX - turretX));
        double launchDistance = Math.sqrt(distance * distance - sideOffset * sideOffset);
        double nearEdgeDistance = launchDistance + hubNearEdgeOffset(launchDirection, sideOffset, 0.0);
        double entranceDistance = farDistanceToAbsHeight(launchAngle, launchVelocity, k_hubHeight);
        double entranceHeight = absHeightAtDistance(launchAngle, launchVelocity, nearEdgeDistance);
        double maxHeight = 2.0 * FieldConstants.k_fuelRadius + maxLaunchHeight(launchAngle, launchVelocity);
        double nearClearanceDistance = launchDistance - hubNearEdgeOffset(launchDirection, sideOffset, sideClearance);
        double farClearanceDistance = launchDistance + hubFarEdgeOffset(launchDirection, sideOffset, sideClearance);
        boolean clearsHub = entranceHeight >= verticalClearance  // measured at the center of the fuel entering the space above the funnel
                         && entranceDistance >= nearClearanceDistance  // measured at the bottom of the fuel hitting 72 inches above the ground
                         && entranceDistance <= farClearanceDistance  // clearance treats the funnel opening as if it has been shrunk
                         && maxHeight <= FieldConstants.k_ceilingHeight;  // it is admittedly unlikely that the turret will hit the ceiling, but still
        return clearsHub;
    }
    public double timeToHubScoring(double launchAngle, double launchVelocity) {
        return TurretConstants.k_extraTimeToPassSensor + secondTimeToAbsHeight(launchAngle, launchVelocity, k_hubHeight);
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
           (sideOffset < pXArr[0] || sideOffset > pXArr[3])? -1 : 
               ((sideOffset < pXArr[1])? 0 : 
                    ((sideOffset < pXArr[2])? 1 : 2));
       if (intersectingSide == -1) {return Double.NaN;}
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
