package frc.robot.subsystems.turret;

import java.util.function.Supplier;

// import org.littletonrobotics.junction.Logger;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
// import frc.robot.Telemetry;

import yams.mechanisms.positional.Pivot;

public class AngleChangerSystem extends SubsystemBase {

    public static Pivot turretPitch = null;

    public final Translation3d turretTranslation = new Translation3d(
        TurretConstants.k_turretRelativeX, 
        TurretConstants.k_turretRelativeY, 
        TurretConstants.k_turretHeight);

    public AngleChangerSystem() {

    }

    @Override
    public void periodic() {

        turretPitch.updateTelemetry();

        // Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
        //     new Pose3d(
        //         turretTranslation,
        //         new Rotation3d(0, turretPitch.getAngle().in(Radians), turretYaw.getAngle().in(Radians)))
        // });
    }

    public Command sysId() {
        return turretPitch.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10));
    }

    @Override
    public void simulationPeriodic() {
        turretPitch.simIterate();
    }

    public Command setAngle(Angle pitch) {
        return turretPitch.setAngle(pitch);
    }

    public Command setAngleDynamic(Supplier<Angle> pitchSupplier) {
        return turretPitch.setAngle(pitchSupplier);
    }

    public Command center() {
        return turretPitch.setAngle(Degrees.of(80));
    }

    public Angle getRawPitch() {
        return turretPitch.getAngle(); // this one does need to be converted
    }

    public Command setPitchDutyCycle(double dutyCycle) {
        return turretPitch.set(dutyCycle);
    }

    public static double launchPitchToMotorPos(double launchPitch) {
        return 0.0; // BRUUUUHHHH
    }
    public static double pitchMotorToLaunchPitch(double motorPos) {
        return 0.0; // tragic
    }
}