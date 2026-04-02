package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import org.littletonrobotics.junction.Logger;

public class Zone {
  private Path2D side;

  public Zone() {
  }

  public Zone(Translation2d[] zoneVertices) {
    side = new Path2D.Double(Path2D.WIND_EVEN_ODD, zoneVertices.length);
    side.moveTo(zoneVertices[0].getX(), zoneVertices[0].getY());

    for (int i = 1; i < zoneVertices.length; i++) {
      side.lineTo(zoneVertices[i].getX(), zoneVertices[i].getY());
    }

    side.closePath();
  }

  // Specify vertices of zone
  public void updateZone(Translation2d[] zoneVertices) {
    side = new Path2D.Double(Path2D.WIND_EVEN_ODD, zoneVertices.length);
    side.moveTo(zoneVertices[0].getX(), zoneVertices[0].getY());

    for (int i = 1; i < zoneVertices.length; i++) {
      side.lineTo(zoneVertices[i].getX(), zoneVertices[i].getY());
    }

    side.closePath();
  }

  // Log the zone
  public void logPoints(String zoneName) {
    Rectangle2D rect = side.getBounds2D();
    double x = rect.getX();
    double y = rect.getY();
    double w = rect.getWidth();
    double h = rect.getHeight();

    Logger.recordOutput(
        "Zones/" + zoneName,
        new Pose2d[] {
            new Pose2d(x, y, new Rotation2d()),
            new Pose2d(x + w, y, new Rotation2d()),
            new Pose2d(x + w, y + h, new Rotation2d()),
            new Pose2d(x, y + h, new Rotation2d()),
            new Pose2d(x, y, new Rotation2d()) // close shape
        });
  }

  // Check whether robot is within the zone
  public boolean contains(Pose2d robotPose) {
    Rectangle2D rect = side.getBounds2D();
    return rect.contains(robotPose.getX(), robotPose.getY());
  }
}
