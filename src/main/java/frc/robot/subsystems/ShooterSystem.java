package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.thethriftybot.devices.ThriftyNova;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;
import yams.motorcontrollers.local.SparkWrapper;


public class ShooterSystem extends SubsystemBase {
    
    private final SparkMax shooterSparky = new SparkMax(Constants.ShooterConstants.k_shooterMotorId, MotorType.kBrushless);
    private final SparkMax followerSparky = new SparkMax(Constants.ShooterConstants.k_followerMotorId, MotorType.kBrushless);
    
  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withFollowers(Pair.of(followerSparky, true))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.00936, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

    private final SmartMotorController smc = new SparkWrapper(shooterSparky, DCMotor.getNeoVortex(1), smcConfig);

    private final FlyWheelConfig shooterConfig = new FlyWheelConfig(smc)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withUpperSoftLimit(RPM.of(6000))
      .withLowerSoftLimit(RPM.of(0))
      .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

      private final FlyWheel FlywheelSparky = new FlyWheel(shooterConfig);

    public ShooterSystem() {
        
    }


     public Command stopCommand() {
        return Commands.run(() -> setShooterSpeed(RPM.of(0)));
     }

     public boolean isAtSpeed(AngularVelocity targetSpeed) {
        return Math.abs(FlywheelSparky.getVelocity() - targetSpeed.getValue()) < 100; // Tolerance of 100 RPM, adjust as needed
     }

     public AngularVelocity getCurrentSpeed() {
        return Degrees.of(FlywheelSparky.getVelocity());
     }

     public boolean isAtSpeedWithTolerance(AngularVelocity targetSpeed, double tolerance) {
        return Math.abs((FlywheelSparky.getVelocity() - targetSpeed.getValue())/(targetSpeed.getValue())) < tolerance;
     }

     public Command setShooterSpeedWithToleranceCommand(AngularVelocity targetSpeed, double tolerance) {
        return Commands.run(() -> setShooterSpeed(targetSpeed))
                .until(() -> isAtSpeedWithTolerance(targetSpeed, tolerance))
                .withName("ShooterSystem.setShooterSpeedWithToleranceCommand");
     }

     
     
    
}

