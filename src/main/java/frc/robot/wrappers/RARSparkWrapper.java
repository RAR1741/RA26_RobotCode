package frc.robot.wrappers;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class RARSparkWrapper extends SparkWrapper {

  private final SmartMotorControllerConfig m_config;

  public RARSparkWrapper(SparkBase controller, DCMotor motor, SmartMotorControllerConfig config) {
    super(controller, motor, config);

    m_config = config;
  }

  @Override
  public void setPosition(Angle angle) {

    Optional<Constraints> constraints = m_config.getTrapezoidProfile();

    Logger.recordOutput("Turret/MotionPofile/trapezoidProfile/maxAcceleration",
        constraints.map(c -> c.maxAcceleration).orElse(0.0));
    Logger.recordOutput("Turret/MotionPofile/trapezoidProfile/maxVelocity",
        constraints.map(c -> c.maxVelocity).orElse(0.0));

    super.setPosition(angle);
  }
}
