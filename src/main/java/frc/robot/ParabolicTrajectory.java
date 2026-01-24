package frc.robot;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.FieldConstants;

public class ParabolicTrajectory {

    public static double k_gravitationalAcceleration = TurretConstants.k_gravitationalAcceleration;
    public static double k_turretHeight = TurretConstants.k_turretHeight;
    public static double k_fieldWidth = FieldConstants.k_fieldWidth;
    public static double k_fieldLength = FieldConstants.k_fieldLength;
    public static double k_allianceZoneLength = FieldConstants.k_allianceZoneLength;
    public static double k_hubZoneLength = FieldConstants.k_hubZoneLength;
    public static double k_neutralZoneLength = FieldConstants.k_neutralZoneLength;
    public static double k_hubRadius = FieldConstants.k_hubRadius;
    public static double k_hubHeight = FieldConstants.k_hubHeight;
    public static double k_hubX = FieldConstants.k_hubX;
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
        double launchDirection = Math.atan2(k_hubY - launchY, k_hubX - launchX);

        return new ParabolicTrajectory(launchDirection, launchAngle, launchVelocity, launchX, launchY, k_turretHeight);
    }
    public static ParabolicTrajectory toOppositeZoneFromXY(double launchX, double launchY) { // opposite from whichever one the robot is currently at, not based on robot alliance
        double targetX;
        double targetY;
        if (launchX > k_fieldLength / 2.0) {
            targetX = k_allianceZoneLength;
        } else {
            targetX = k_fieldLength - k_allianceZoneLength;
        }
        if (launchY > k_fieldWidth / 2)
        return new ParabolicTrajectory(, , , launchX, launchY, k_turretHeight)
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
