package frc.robot;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.FieldConstants;

import edu.wpi.first.wpilibj.DriverStation;

public class ParabolicTrajectory {
    public static String k_alliance = DriverStation.getAlliance().toString();
    public static Boolean isBlueTeam = k_alliance == "Blue";
    public static Boolean isRedTeam = k_alliance == "Red";

    public static double k_gravitationalAcceleration = TurretConstants.k_gravitationalAcceleration;
    public static double k_turretHeight = TurretConstants.k_turretHeight;
    public static double k_fieldWidth = FieldConstants.k_fieldWidth;
    public static double k_fieldLength = FieldConstants.k_fieldLength;
    public static double k_allianceZoneDepth = FieldConstants.k_allianceZoneDepth;
    public static double k_hubZoneDepth = FieldConstants.k_hubZoneDepth;
    public static double k_neutralZoneDepth = FieldConstants.k_neutralZoneDepth;
    public static double k_trenchWidth = FieldConstants.k_trenchWidth;
    public static double k_hubBodyWidth = FieldConstants.k_hubBodyWidth;
    public static double k_hubBodyDepth = FieldConstants.k_hubBodyDepth;
    public static double k_hubRadius = FieldConstants.k_hubRadius;
    public static double k_hubHeight = FieldConstants.k_hubHeight;
    public static double k_hubX = (isRedTeam)? FieldConstants.k_redHubX : 
                                      (isBlueTeam)? FieldConstants.k_blueHubX :
                                          Double.NaN;
    public static double k_hubY = FieldConstants.k_hubY;
    public static double k_fuelRadius = FieldConstants.k_fuelRadius;
    public static double k_launchDirectionTolerance = TurretConstants.k_launchDirectionTolerance;
    public static double k_launchAngleTolerance = TurretConstants.k_launchAngleTolerance;
    public static double k_launchVelocityTolerance = TurretConstants.k_launchVelocityTolerance;
    public static double k_maxLaunchVelocity = TurretConstants.k_maxLaunchVelocity;

    public static double targetLaunchVY = TurretConstants.k_targetLaunchVY;
    public static double slopeMaxDegrees = tan(TurretConstants.k_maxLaunchAngle);
    public static double slopeMinDegrees = tan(TurretConstants.k_maxLaunchAngle);
    public static double minLaunchVX = targetLaunchVY / slopeMaxDegrees;
    public static double maxLaunchVX = targetLaunchVY / slopeMinDegrees;
    public static double verticalDistance = k_hubHeight + k_fuelRadius - k_turretHeight;
    public static double targetVXCoefficient = k_gravitationalAcceleration / 
        (targetLaunchVY + Math.sqrt(targetLaunchVY * targetLaunchVY - 2.0 * k_gravitationalAcceleration * verticalDistance));
    public static double sqrtHalfGravity = Math.sqrt(k_gravitationalAcceleration / 2.0);

    public final double launchDirection;
    public final double launchAngle;
    public final double launchVelocity;
    public final double launchX;
    public final double launchY;
    public final double launchHeight;

    public ParabolicTrajectory(double launchDirection, double launchAngle, double launchVelocity, double launchX, double launchY, double launchHeight) {
        this.launchDirection = launchDirection;
        this.launchAngle = launchAngle;
        this.launchVelocity = launchVelocity;
        this.launchX = launchX;
        this.launchY = launchY;
        this.launchHeight = launchHeight;
    }

    // graph link: https://www.desmos.com/calculator/67orqpt33w

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

    public static double timeToHubFromXY(double launchX, double launchY) {
        double horizontalDistance = Math.hypot(k_hubX - launchX, k_hubY - launchY);
        double launchHorizontalVelocity = targetVXCoefficient * horizontalDistance;
        if (launchHorizontalVelocity < minLaunchVX) {
            return horizontalDistance / minLaunchVX;
        } else if (launchHorizontalVelocity > maxLaunchVX) {
            return Math.sqrt(horizontalDistance - verticalDistance) / sqrtHalfGravity;
        }
        return 1.0 / targetVXCoefficient;
    }

    public static ParabolicTrajectory toHubFromXYWhileDriving(double[] launch) { // launchX, double launchY, double turretVX, double turretVY) {
        double launchX = launch[0]; double launchY = launch[1]; double turretVX = launch[2]; double turretVY = launch[3];
        double time = timeToHubFromXY(launchX, launchY);
        if (time == Double.NaN) {return null;}
        double xDisplacement = k_hubX - launchX - time * turretVX;
        double yDisplacement = k_hubY - launchY - time * turretVY;
        double horizontalDistance = Math.hypot(xDisplacement, yDisplacement);
        double launchHorizontalVelocity = targetVXCoefficient * horizontalDistance;
        double launchVerticalVelocity = targetLaunchVY;
        double launchAngle;
        if (launchHorizontalVelocity < minLaunchVX) {
            launchHorizontalVelocity = minLaunchVX;
            launchAngle = TurretConstants.k_maxLaunchAngle;
        } else if (launchHorizontalVelocity > maxLaunchVX) {
            launchHorizontalVelocity = sqrtHalfGravity * horizontalDistance / Math.sqrt(horizontalDistance - verticalDistance);
            launchVerticalVelocity = launchHorizontalVelocity * slopeMinDegrees;
            launchAngle = TurretConstants.k_minLaunchAngle;
        } else {
            launchAngle = atan2(launchVerticalVelocity, launchHorizontalVelocity);
        }
        if (launchHorizontalVelocity == Double.NaN) {return null;}
        double launchVelocity = Math.hypot(launchHorizontalVelocity, launchVerticalVelocity);
        double launchDirection = atan2(yDisplacement, xDisplacement);
        return new ParabolicTrajectory(launchDirection, launchAngle, launchVelocity, launchX, launchY, k_turretHeight);
    }

    public static ParabolicTrajectory toZoneFromXYWhileDriving(double[] launch) { // launchX, double launchY, double turretVX, double turretVY) {
        double launchX = launch[0]; double launchY = launch[1]; double turretVX = launch[2]; double turretVY = launch[3];
        double targetX = isBlueTeam? k_allianceZoneDepth + k_hubBodyWidth / 2.0 : k_fieldLength - k_allianceZoneDepth - k_hubBodyWidth / 2.0;
        double targetY = k_fieldWidth / 2.0;
        boolean topHalf = launchY > k_fieldWidth / 2.0;
        if (isBlueTeam && launchX < k_allianceZoneDepth + k_hubZoneDepth) {
            targetX = 0.0;
        } else if (!isBlueTeam && launchX > k_fieldLength - k_allianceZoneDepth - k_hubZoneDepth) {
            targetX = k_fieldLength;
        } else if (Math.abs(targetY - launchY) > k_hubBodyWidth / 2.0 + k_fuelRadius) {
            double hubCornerX = isBlueTeam? k_allianceZoneDepth : k_fieldLength - k_allianceZoneDepth;
            double hubCornerY = targetY + k_hubBodyWidth / 2.0 * (topHalf? 1.0 : -1.0);
            double directionToHubCorner = atan2(hubCornerY - launchY, hubCornerX - launchX);
            double targetDirection = directionToHubCorner - k_launchDirectionTolerance * (topHalf? 1.0 : -1.0) * (isBlueTeam? 1.0 : -1.0);
            targetY = launchY + (targetX - launchX) * tan(targetDirection);
            if (targetY < 0 || targetY > k_fieldWidth) {
                return null;
            } else if (targetY < k_trenchWidth || targetY > k_fieldWidth - k_trenchWidth) {
                targetY = Math.min(k_trenchWidth, Math.max(k_fieldWidth - k_trenchWidth, targetY));
            }
        } else if (launchX > k_allianceZoneDepth && launchX < k_fieldLength - k_allianceZoneDepth) {
            double hubCornerX = isBlueTeam? k_allianceZoneDepth : k_fieldLength - k_allianceZoneDepth;
            double hubCornerY = targetY + k_hubBodyWidth / 2.0 * (topHalf? 1.0 : -1.0);
            double directionToHubCorner = atan2(hubCornerY - launchY, hubCornerX - launchX);
            double targetDirection = directionToHubCorner - k_launchDirectionTolerance * (topHalf? 1.0 : -1.0) * (isBlueTeam? 1.0 : -1.0);
            targetY = launchY + (targetX - launchX) * tan(targetDirection);
            if (targetY < 0 || targetY > k_fieldWidth) {
                return null;
            } else if (targetY < k_trenchWidth || targetY > k_fieldWidth - k_trenchWidth) {
                targetY = Math.min(k_trenchWidth, Math.max(k_fieldWidth - k_trenchWidth, targetY));
            }
        } else {
            return null;
        }
        double targetDistance = Math.hypot(targetX - launchX, targetY - launchY);
        double idealLaunchVelocity = targetDistance * TurretConstants.k_maxLaunchVelocity / TurretConstants.k_maxZoneLaunchDistance;
        ParabolicTrajectory trajectory = toXYHFromVXYHMinimizeAngle(targetX, targetY, k_turretHeight, launchX, launchY, k_turretHeight, idealLaunchVelocity);
        if (trajectory == null) {return null;}
        double time = targetDistance / trajectory.getHorizontalLaunchVelocity();
        trajectory = toXYHFromVXYHMinimizeAngle(targetX - turretVX * time, targetY - turretVY * time, k_turretHeight, launchX, launchY, k_turretHeight, idealLaunchVelocity);
        return (trajectory != null && trajectory.testValid())? trajectory : null;
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

    public double getHorizontalLaunchVelocity() {
        return launchVelocity * cos(launchAngle);
    }
    public double getVerticalLaunchVelocity() {
        return launchVelocity * sin(launchAngle);
    }
    public double maxLaunchHeight() {
        return Math.pow(getVerticalLaunchVelocity(), 2.0) / k_gravitationalAcceleration / 2.0;
    }
    public double absHeightAtDistance(double testX) {
        double time = testX / getHorizontalLaunchVelocity();
        double launchFactor = testX * tan(launchAngle);
        double dropFactor = time * k_gravitationalAcceleration * k_gravitationalAcceleration / 2.0;
        return k_turretHeight + launchFactor - dropFactor;
    }
    public double secondTimeToAbsHeight(double testHeight) {
        double launchVY = getVerticalLaunchVelocity();
        double radicand = launchVY * launchVY - 2.0 * k_gravitationalAcceleration * (testHeight - k_turretHeight);
        return (launchVY + Math.sqrt(radicand)) / k_gravitationalAcceleration;
    }
    public double farDistanceToAbsHeight(double launchAngle, double launchVelocity, double testHeight) {
        return getHorizontalLaunchVelocity() * secondTimeToAbsHeight(testHeight);
    }
    public double trajectorySlopeAtDistance(double launchAngle, double launchVelocity, double testX) {
        double xVelocity = getHorizontalLaunchVelocity();
        double time = testX / xVelocity;
        double yVelocity = getVerticalLaunchVelocity() - time * k_gravitationalAcceleration;
        return yVelocity / xVelocity;
    }
    public boolean testValid() {
        return launchVelocity > 0 && launchVelocity <= k_maxLaunchVelocity && 
               launchAngle >= TurretConstants.k_minLaunchAngle && launchAngle <= TurretConstants.k_maxLaunchAngle && 
               launchX >= 0.0 && launchX <= k_fieldLength && 
               launchY >= 0.0 && launchY <= k_fieldWidth && 
               launchHeight >= 0.0 && launchHeight <= FieldConstants.k_ceilingHeight;
    }
    public double timeToHubScoring() {
        return TurretConstants.k_extraTimeToPassSensor + secondTimeToAbsHeight(k_hubHeight);
    }

    
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
    
    public static double sin(double degrees) {return Math.sin(Math.toRadians(degrees));}
    public static double cos(double degrees) {return Math.cos(Math.toRadians(degrees));}
    public static double tan(double degrees) {return Math.tan(Math.toRadians(degrees));}
    public static double arcsin(double ratio) {return Math.toDegrees(Math.asin(ratio));}
    public static double arccos(double ratio) {return Math.toDegrees(Math.acos(ratio));}
    public static double atan2(double y, double x) {return Math.toDegrees(Math.atan2(y, x));}
}
