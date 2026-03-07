package frc.robot.subsystems.turret;

import java.util.function.Supplier;

// import org.littletonrobotics.junction.Logger;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
// import frc.robot.Telemetry;

import yams.mechanisms.positional.Pivot;

public class SwivelSystem extends SubsystemBase {

    public static Pivot turretYaw = null;
    public static SparkMax yawMotorSpark = null;

    public final Translation3d turretTranslation = new Translation3d(
        TurretConstants.k_turretRelativeX, 
        TurretConstants.k_turretRelativeY, 
        TurretConstants.k_turretHeight);

    public SwivelSystem() {

    }

    @Override
    public void periodic() {
        turretYaw.updateTelemetry();

        // Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
        //     new Pose3d(
        //         turretTranslation,
        //         new Rotation3d(0, turretPitch.getAngle().in(Radians), turretYaw.getAngle().in(Radians)))
        // });
    }

    public Command sysId() {
        return null;
    }

    @Override
    public void simulationPeriodic() {
        turretYaw.simIterate();
    }

    public Command setAngle(Angle yaw) {
        return turretYaw.setAngle(yaw);
    }

    public Command setAngleDynamic(Supplier<Angle> yawSupplier) {
        return turretYaw.setAngle(yawSupplier);
    }

    public Command center() {
        return turretYaw.setAngle(Degrees.of(0));
    }

    public Angle getRobotRelativeYaw() {
        return turretYaw.getAngle();
    }

    public Angle getFieldRelativeYaw(Angle robotOrientation) {
        return turretYaw.getAngle().minus(robotOrientation);
    }

    public Command setYawDutyCycle(double dutyCycle) {
        return turretYaw.set(dutyCycle);
    }

    public Command rezeroYaw() {
        return Commands.runOnce(() -> yawMotorSpark.getEncoder().setPosition(0), this).withName("TurretYaw.Rezero");
    }
}