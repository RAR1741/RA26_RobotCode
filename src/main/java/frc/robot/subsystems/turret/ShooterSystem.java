package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
// import frc.robot.Telemetry;

import yams.mechanisms.velocity.FlyWheel;

public class ShooterSystem extends SubsystemBase {

    public static SparkMax flywheelSparkA = null;
    public static SparkMax flywheelSparkB = null;
    public static FlyWheel shooter = null;

    public final Translation3d turretTranslation = new Translation3d(
        TurretConstants.k_turretRelativeX, 
        TurretConstants.k_turretRelativeY, 
        TurretConstants.k_turretHeight);

    public ShooterSystem() {

    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/FlywheelVelocityA", flywheelSparkA.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/FlywheelVelocityB", flywheelSparkB.getEncoder().getVelocity());

        Logger.recordOutput("ASCalibration/FinalComponentPoses", new Pose3d[] {
            new Pose3d(
                turretTranslation,
                new Rotation3d(0, turretPitch.getAngle().in(Radians), turretYaw.getAngle().in(Radians)))
        });
    }

    @Override
    public Command sysId() {
        return shooter.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(10)); // check these numbers??
    }

    @Override
    public void simulationPeriodic() {
        shooter.simIterate();
    }

    public Command setSpeed(AngularVelocity speed) {
        return shooter.setSpeed(speed);
    }

    public Command setSpeedDynamic(Supplier<AngularVelocity> speedSupplier) {
        return shooter.setSpeed(speedSupplier);
    }

    public Command spinUp() {
        return setSpeed(RPM.of(5500));
    }

    public Command stop() {
        return setSpeed(RPM.of(0));
    }

    public AngularVelocity getSpeed() {
        return shooter.getSpeed();
    }

    public static AngularVelocity launchVelocityToAngular(double launchVelocity) {
        return RadiansPerSecond.of(launchVelocity / (TurretConstants.k_wheelRadius * TurretConstants.k_shootingVelocityTransferEfficiency));
    }

    public LinearVelocity getTangentialVelocity() {
        // Calculate tangential velocity at the edge of the wheel and convert to LinearVelocity
        return FeetPerSecond.of(getSpeed().in(RadiansPerSecond) * TurretConstants.k_wheelRadius);
    }
}