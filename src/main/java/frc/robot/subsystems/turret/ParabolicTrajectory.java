package frc.robot.subsystems.turret;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.FieldConstants;

import edu.wpi.first.wpilibj.DriverStation;

public class ParabolicTrajectory {
    public static String k_alliance = DriverStation.getAlliance().toString();
    public static Boolean isBlueTeam = k_alliance == "Blue";
    public static Boolean isRedTeam = k_alliance == "Red";

    public static double k_gravity = TurretConstants.k_gravity;
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
    public static double targetVXCoefficient = k_gravity / 
        (targetLaunchVY + Math.sqrt(targetLaunchVY * targetLaunchVY - 2.0 * k_gravity * verticalDistance));
    public static double halfGravity = k_gravity / 2.0;
    public static double sqrtHalfGravity = Math.sqrt(halfGravity);

    public final double launchDirection;
    public final double launchAngle;
    public final double launchVelocity;
    public final double launchX;
    public final double launchY;
    public final double launchHeight;
    public boolean doShoot;

    public ParabolicTrajectory(double launchDirection, double launchAngle, double launchVelocity, double launchX, double launchY, double launchHeight) {
        this.launchDirection = launchDirection;
        this.launchAngle = launchAngle;
        this.launchVelocity = launchVelocity;
        this.launchX = launchX;
        this.launchY = launchY;
        this.launchHeight = launchHeight;
        this.doShoot = true;
    }

    public ParabolicTrajectory(double launchDirection, double launchAngle, double launchVelocity, double launchX, double launchY, double launchHeight, boolean doShoot) {
        this.launchDirection = launchDirection;
        this.launchAngle = launchAngle;
        this.launchVelocity = launchVelocity;
        this.launchX = launchX;
        this.launchY = launchY;
        this.launchHeight = launchHeight;
        this.doShoot = doShoot;
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

    public static ParabolicTrajectory toDDHFromVXYMinimizeAngle(double launchDirection, double horizontalDistance, double targetHeight, double launchX, 
                                                                 double launchY, double launchVelocity) {
        double verticalDistance = targetHeight - k_turretHeight;
        double launchAngle = solveLowerLaunchAngle(launchVelocity, horizontalDistance, verticalDistance);
        if (launchAngle == Double.NaN) {return null;}
        if (launchAngle < TurretConstants.k_minLaunchAngle) {
            launchAngle = TurretConstants.k_minLaunchAngle;
            launchVelocity = solveLaunchVelocity(launchAngle, horizontalDistance, verticalDistance);
            if (launchVelocity == Double.NaN) {return null;}
        } else if (launchAngle > TurretConstants.k_maxLaunchAngle) {
            launchAngle = TurretConstants.k_maxLaunchAngle;
            launchVelocity = solveLaunchVelocity(launchAngle, horizontalDistance, verticalDistance);
            if (launchVelocity == Double.NaN) {return null;}
        }
        return new ParabolicTrajectory(launchDirection, launchAngle, launchVelocity, launchX, launchY, k_turretHeight);
    }

    public static double timeToHubFromAXYSkewInput(double launchAngle, double launchDirection, double launchX, double launchY, double turretVX, double turretVY) {
        double horizontalDistance = Math.hypot(k_hubX - launchX, k_hubY - launchY);
        double verticalDistance = k_hubHeight - k_turretHeight;
        // dot product
        double turretVelocityToHub = turretVX * cos(launchDirection) + turretVY * sin(launchDirection);
        double launchVelocity = solveLaunchVelocity(launchAngle, horizontalDistance, verticalDistance);
        for (int i = 0; i < TurretConstants.k_timeSolvingIterations; i++) { // 5 trig ops + 1 sqrt ops + 1 div operation per iteration
            launchAngle = skewLaunchToHubAngleByVelocity(launchAngle, launchDirection, launchVelocity, turretVelocityToHub);
            launchVelocity = solveLaunchVelocity(launchAngle, horizontalDistance, verticalDistance);
        }
        return horizontalDistance / (launchVelocity * cos(launchAngle));
    }

    // public static ParabolicTrajectory toHubFromVXY(double launchVelocity, double launchX, double launchY) { // maybe don't need, but nonetheless skew angles
    //     double xDistance = Math.hypot(k_hubX - launchX, k_hubY - launchY);
    //     double yDistance = k_hubHeight - k_turretHeight;
    //     double launchAngle = solveUpperLaunchAngle(launchVelocity, xDistance, yDistance);
    //     double launchDirection = Math.toDegrees(Math.atan2(k_hubY - launchY, k_hubX - launchX));
    //     if (launchAngle == Double.NaN) {return null;}
    //     return new ParabolicTrajectory(launchDirection, launchAngle, launchVelocity, launchX, launchY, k_turretHeight);
    // }

    public static double timeToHubFromXYSkewLimits(double launchDirection, double launchX, double launchY, double launchVX, double launchVY) { // tOdO skew limit angles
        double horizontalDistance = Math.hypot(k_hubX - launchX, k_hubY - launchY);
        double launchHorizontalVelocity = targetVXCoefficient * horizontalDistance;
        if (launchHorizontalVelocity < minLaunchVX) {
            return horizontalDistance / minLaunchVX;
        } else if (launchHorizontalVelocity > maxLaunchVX) {
            return Math.sqrt(horizontalDistance - verticalDistance) / sqrtHalfGravity;
        }
        return 1.0 / targetVXCoefficient;
    }

    public static ParabolicTrajectory toHubFromXYWhileDriving(double launchX, double launchY, double turretVX, double turretVY) {
        double naiveXDisplacement = k_hubX - launchX;
        double naiveYDisplacement = k_hubY - launchY;
        double naiveLaunchDirection = atan2(naiveYDisplacement, naiveXDisplacement);
        double time = timeToHubFromXYSkewLimits(naiveLaunchDirection, launchX, launchY, turretVX, turretVY);
        if (time == Double.NaN) {return null;}
        double xDisplacement = naiveXDisplacement - time * turretVX;
        double yDisplacement = naiveYDisplacement - time * turretVY;
        double launchDirection = atan2(yDisplacement, xDisplacement);
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
        return new ParabolicTrajectory(launchDirection, launchAngle, launchVelocity, launchX, launchY, k_turretHeight);
    }

    public static ParabolicTrajectory toHubFromAXYWhileDriving(double launchAngle, double launchX, double launchY, double turretVX, double turretVY) {
        double naiveXDisplacement = k_hubX - launchX;
        double naiveYDisplacement = k_hubY - launchY;
        double naiveLaunchDirection = atan2(naiveYDisplacement, naiveXDisplacement);
        double time = timeToHubFromAXYSkewInput(launchAngle, naiveLaunchDirection, launchX, launchY, turretVX, turretVY);
        if (time == Double.NaN) {
            time = 0.0;
        }
        double xDisplacement = naiveXDisplacement - time * turretVX;
        double yDisplacement = naiveYDisplacement - time * turretVY;
        double horizontalDistance = Math.hypot(xDisplacement, yDisplacement);
        double launchVelocity = solveLaunchVelocity(launchAngle, horizontalDistance, k_hubHeight - k_turretHeight);
        if (launchVelocity == Double.NaN) {return null;}
        double launchDirection = atan2(yDisplacement, xDisplacement);
        return new ParabolicTrajectory(launchDirection, launchAngle, launchVelocity, launchX, launchY, k_turretHeight);
    }

    public static ParabolicTrajectory toZoneFromXYWhileDriving(double launchX, double launchY, double turretVX, double turretVY) {
        double targetX;
        double targetY;
        @SuppressWarnings("unused")
        boolean doShoot;
        boolean topHalf = launchY > k_hubY;
        double halfHubEffectiveWidth = k_hubBodyWidth / 2.0 + FieldConstants.k_hubNetOverhang + k_fuelRadius;
        double hubTargetXDistance = k_allianceZoneDepth + k_hubZoneDepth + k_fuelRadius;
        double targetDirection;
        double targetDistance;
        if (isBlueTeam) {
            if (launchX > FieldConstants.k_redHubX &&  Math.abs(launchY - k_hubY) < halfHubEffectiveWidth) {
                targetX = hubTargetXDistance;
                targetY = launchY;
                doShoot = false;
            } else {
                targetX = (launchX > FieldConstants.k_blueHubX && Math.abs(launchY - k_hubY) < halfHubEffectiveWidth)? hubTargetXDistance : 0.0;
                targetY = k_hubY + halfHubEffectiveWidth * (topHalf? 1.0 : -1.0);
                doShoot = true;
            }
            targetDirection = atan2(targetY - launchY, targetX - launchX) - TurretConstants.k_launchAngleTolerance * (topHalf? 1.0 : -1.0);
            targetDistance = Math.abs(Math.max(k_allianceZoneDepth, launchX - k_allianceZoneDepth) / cos(targetDirection));
        } else {
            if (launchX < FieldConstants.k_blueHubX &&  Math.abs(launchY - k_hubY) < halfHubEffectiveWidth) {
                targetX = k_fieldLength - hubTargetXDistance;
                targetY = launchY;
                doShoot = false;
            } else {
                targetX = (launchX < FieldConstants.k_redHubX && Math.abs(launchY - k_hubY) < halfHubEffectiveWidth)? k_fieldLength - hubTargetXDistance : 0.0;
                targetY = k_hubY + halfHubEffectiveWidth * (topHalf? 1.0 : -1.0);
                doShoot = true;
            }
            targetDirection = atan2(targetY - launchY, targetX - launchX) + TurretConstants.k_launchAngleTolerance * (topHalf? 1.0 : -1.0);
            targetDistance = Math.abs(Math.max(k_allianceZoneDepth, k_fieldLength - k_allianceZoneDepth - launchX) / cos(targetDirection));
        }
        targetX = launchX + targetDistance * cos(targetDirection);
        targetY = launchY + targetDistance * sin(targetDirection);
        if (targetY < 0.0 || targetY > k_fieldWidth) {
            doShoot = false; // this should only happen if we are shooting from right behind our own hub, in which case there would be no good shot to take
        } 
        double idealLaunchVelocity = TurretConstants.k_minZoneLaunchVelocity + (targetDistance - k_allianceZoneDepth) / (TurretConstants.k_maxZoneLaunchDistance - k_allianceZoneDepth)
                                                                                * (TurretConstants.k_maxLaunchVelocity - TurretConstants.k_minZoneLaunchVelocity);
        //double time = timeToXYHFromVXY(stuff);
        ParabolicTrajectory trajectory = toDDHFromVXYMinimizeAngle(targetDirection, targetDistance, k_turretHeight, launchX, launchY, idealLaunchVelocity);
        if (trajectory == null) {return null;} // need to sort this out another time bc idk but ts pmo irl ykwim
        @SuppressWarnings("unused")
        double skewedLaunchVelocity = skewVelocityByTurretVelocity(trajectory.launchAngle, trajectory.launchDirection, idealLaunchVelocity, turretVX, turretVY);
        double time = targetDistance / trajectory.getHorizontalLaunchVelocity();
        trajectory = toXYHFromVXYHMinimizeAngle(targetX - turretVX * time, targetY - turretVY * time, k_turretHeight, launchX, launchY, k_turretHeight, idealLaunchVelocity);
        return (trajectory != null && trajectory.testValid())? trajectory : null;
    }

    public static double skewVelocityByTurretVelocity(double launchAngle, double launchDirection, double launchVelocity, double turretVX, double turretVY) {
        double vX = launchVelocity * cos(launchAngle) * cos(launchDirection) - turretVX; // minus or plus?? i forget
        double vY = launchVelocity * cos(launchAngle) * sin(launchDirection) - turretVY;
        double vZ = launchVelocity * sin(launchAngle);
        return Math.sqrt(vX * vX + vY * vY + vZ * vZ);
    }
    public static double skewLaunchToHubAngleByVelocity(double launchAngle, double launchDirection, double launchVelocity, double turretVelocityToHub) {
        return atan2(launchVelocity * sin(launchAngle), turretVelocityToHub + launchVelocity * cos(launchAngle));
    }

    public static double solveLaunchVelocity(double launchAngle, double xDistance, double yDistance) {
        // return xDistance * Math.sqrt(k_gravity / (2.0 * cos * (xDistance * sin - yDistance * cos)));
        return xDistance * Math.sqrt(k_gravity / (xDistance * sin(2.0 * launchAngle) - yDistance * (1.0 + cos(2.0 * launchAngle))));
    }
    public static double solveUpperLaunchAngle(double launchVelocity, double xDistance, double yDistance) {
        double distance = Math.hypot(xDistance, yDistance);
        double targetFactor = yDistance / distance;
        double dropFactor = k_gravity * xDistance * xDistance / (launchVelocity * launchVelocity * distance);
        return (arccos(targetFactor + dropFactor) + arccos(-targetFactor)) / 2.0;
    }
    public static double solveLowerLaunchAngle(double launchVelocity, double xDistance, double yDistance) {
        double distance = Math.hypot(xDistance, yDistance);
        double targetFactor = yDistance / distance;
        double dropFactor = k_gravity * xDistance * xDistance / (launchVelocity * launchVelocity * distance);
        return (arcsin(targetFactor + dropFactor) + arcsin(targetFactor)) / 2.0;
    }

    // instance methods

    public double getHorizontalLaunchVelocity() {
        return launchVelocity * cos(launchAngle);
    }
    public double getVerticalLaunchVelocity() {
        return launchVelocity * sin(launchAngle);
    }
    public double maxLaunchHeight() {
        return Math.pow(getVerticalLaunchVelocity(), 2.0) / (2.0 * k_gravity);
    }
    public double absHeightAtDistance(double testX) {
        double time = testX / getHorizontalLaunchVelocity();
        double launchFactor = testX * tan(launchAngle);
        double dropFactor = time * k_gravity * halfGravity;
        return k_turretHeight + launchFactor - dropFactor;
    }
    public double secondTimeToAbsHeight(double testHeight) {
        double launchVY = getVerticalLaunchVelocity();
        double radicand = launchVY * launchVY - 2.0 * k_gravity * (testHeight - k_turretHeight);
        return (launchVY + Math.sqrt(radicand)) / k_gravity;
    }
    public double timeToDistance(double testDistance) {
        return testDistance / getHorizontalLaunchVelocity();
    }
    public double launchToHubDistance() {
        return Math.hypot(k_hubX - launchX, k_hubY - launchY);
    }
    public double farDistanceToAbsHeight(double launchAngle, double launchVelocity, double testHeight) {
        return getHorizontalLaunchVelocity() * secondTimeToAbsHeight(testHeight);
    }
    public double trajectorySlopeAtDistance(double launchAngle, double launchVelocity, double testX) {
        double xVelocity = getHorizontalLaunchVelocity();
        double time = testX / xVelocity;
        double yVelocity = getVerticalLaunchVelocity() - time * k_gravity;
        return yVelocity / xVelocity;
    }
    public boolean testValid() {
        return launchVelocity > 0.0 && launchVelocity <= k_maxLaunchVelocity && 
               launchAngle >= TurretConstants.k_minLaunchAngle && launchAngle <= TurretConstants.k_maxLaunchAngle && 
               launchX >= 0.0 && launchX <= k_fieldLength && 
               launchY >= 0.0 && launchY <= k_fieldWidth && 
               launchHeight >= 0.0 && launchHeight <= FieldConstants.k_ceilingHeight && 
               maxLaunchHeight() <= FieldConstants.k_ceilingHeight;
    }
    public double timeToHubScoring() {
        return TurretConstants.k_extraTimeToPassSensor + timeToDistance(launchToHubDistance());
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

    // math helpers
    
    public static double sin(double degrees) {return Math.sin(Math.toRadians(degrees));}
    public static double cos(double degrees) {return Math.cos(Math.toRadians(degrees));}
    public static double tan(double degrees) {return Math.tan(Math.toRadians(degrees));}
    public static double arcsin(double ratio) {return Math.toDegrees(Math.asin(ratio));}
    public static double arccos(double ratio) {return Math.toDegrees(Math.acos(ratio));}
    public static double atan2(double y, double x) {return Math.toDegrees(Math.atan2(y, x));}
    public static double qFormulaPlus(double a, double b, double c) {return (-b + Math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a);}
    public static double qFormulaMinus(double a, double b, double c) {return (-b - Math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a);}
    public static double qFormulaGreater(double a, double b, double c) {return (-b + (a > 0? 1.0 : -1.0) * Math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a);}
    public static double qFormulaLesser(double a, double b, double c) {return (-b + (a > 0? -1.0 : 1.0) * Math.sqrt(b * b - 4.0 * a * c)) / (2.0 * a);}
}
