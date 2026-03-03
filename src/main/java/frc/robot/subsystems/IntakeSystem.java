package frc.robot.subsystems;

import com.revrobotics.ControlType;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;




public class IntakeSystem extends SubsystemBase {

  private static final double INTAKE_SPEED = 1.0;
  private static final double INTAKE_SPEED_STOP = 0.0;

  // SparkMax controlling the intake roller
  private SparkMax rollerMax = new SparkMax(IntakeConstants.k_rollerMotorId, MotorType.kBrushless);
  private SparkMax pivotMax = new SparkMax(IntakeConstants.k_pivotMotorId, MotorType.kBrushless);
  // simple roller + pivot implementation (no Yams)
  private static final double PIVOT_GEAR_RATIO = 5 * 5 * (60.0 / 15.0); // 5:1 * 5:1 * 60/15


// private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
//       .withFollowers(Pair.of(pivotMotor, true))
//       .withControlMode(ControlMode.CLOSED_LOOP)
//       .withClosedLoopController(0.00936, 0, 0)
//       .withFeedforward(new SimpleMotorFeedforward(0.191, 0.11858, 0.0))
//       .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
//       .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
//       .withMotorInverted(false)
//       .withIdleMode(MotorMode.COAST)
//       .withStatorCurrentLimit(Amps.of(40));

 //  private final SmartMotorController smc = new SparkWrapper(rollerMax, DCMotor.getNeoVortex(1), smcConfig);

  // private final FlyWheelConfig intakeConfig = new FlyWheelConfig(smc)
  //     .withDiameter(Inches.of(4))
  //     .withMass(Pounds.of(1))
  //     .withUpperSoftLimit(RPM.of(6000))
  //     .withLowerSoftLimit(RPM.of(0))
  //     .withTelemetry("Intake", TelemetryVerbosity.HIGH);

  // private final FlyWheel FlywheelIntake = new FlyWheel(intakeConfig);


  // NOTE: we no longer use Yams abstractions (FlyWheel/Arm/SmartMotorController).
  // Roller will be driven directly through the ThriftyNova device.

  public IntakeSystem() {
    // pivotMotor.factoryReset();
  }

  /**
   * Command to run the intake while held.
   */
  public Command intakeCommand() {
    return Commands.run(() -> rollerMax.set(INTAKE_SPEED), this)
        .finallyDo(() -> pivotMax.set(0))
        .withName("Intake.Run");
  }

  // /**
  //  * Command to eject while held.
  //  */
  // public Command ejectCommand() {
  //   return intake.set(-INTAKE_SPEED).finallyDo(() -> smc.setDutyCycle(0)).withName("Intake.Eject");
  // }

  public Command setPivotAngle(Angle angle) {
    return Commands.runOnce(() -> setPivotPosition(angle), this).withName("IntakePivot.SetAngle");
  }

  public Command rezero() {
    return Commands.runOnce(() -> pivotMax.getEncoder().setPosition(0), this).withName("IntakePivot.Rezero");
  }

  /**
   * Command to deploy intake and run roller while held.
   * Stops roller when released.
   */
  public Command ForwardRollCommand() {
    return Commands.run(() -> {
      setIntakeDeployed();
      rollerMax.set(INTAKE_SPEED);
      pivotMax.set(-INTAKE_SPEED);
    }, this).finallyDo(() -> {
      rollerMax.set(INTAKE_SPEED_STOP);
      pivotMax.set(INTAKE_SPEED_STOP);
      setIntakeHold();
    }).withName("Intake.ForwardRollCommand");
  }

  public Command BackwardRollCommand() {
    return Commands.run(() -> {
      setIntakeStow();
      rollerMax.set(-INTAKE_SPEED);
      pivotMax.set(INTAKE_SPEED);
    }, this).finallyDo(() -> {
      rollerMax.set(INTAKE_SPEED_STOP);
      pivotMax.set(INTAKE_SPEED_STOP);
      setIntakeHold();
    }).withName("Intake.BackwardRollCommand");
  }

  private void setIntakeStow() {
    setPivotPosition(IntakeConstants.k_IntakeStow);
  }

  private void setIntakeFeed() {
    setPivotPosition(IntakeConstants.k_IntakeFeed);
  }

  private void setIntakeHold() {
    setPivotPosition(IntakeConstants.k_IntakeHold);
  }

  private void setIntakeDeployed() {
    setPivotPosition(IntakeConstants.k_IntakeDeployed);
  }

  private void setPivotPosition(Angle angle) {
    // Convert joint degrees to motor rotations, accounting for gear ratio.
    double jointDegrees = angle.in(Degrees);
    double motorRotations = (jointDegrees / 360.0) * PIVOT_GEAR_RATIO;
    // Use SparkMax PID position control (rotations)
    try {
      // pivotMax.getPIDController().setReference(motorRotations, ControlType.kPosition);
      pivotMax.getEncoder().setPosition(motorRotations); 
    } catch (Exception e) {
      // If the concrete SparkMax PID API isn't available in this environment, fallback to open-loop set
      // This is defensive: replace with appropriate API calls if needed.
      // No-op or log
    }
  }

  @Override
  public void periodic() {
    // Telemetry: pivot encoder position could be sampled here if desired
  }

  @Override
  public void simulationPeriodic() {
    // Simulation hooks removed (Yams-specific)
  }
}
// Deploy Gear Ratio: 25:1(2 5:1's) - 60:15
// Spin Gear Ratio: 9:1
//make it toggaleable