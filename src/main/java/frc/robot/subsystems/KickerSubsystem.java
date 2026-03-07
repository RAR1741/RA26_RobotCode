package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {
  private static final double KICKER_SPEED = 1.0;
  private static final double KICKER_SPEED_STOP = 0;
  private SparkFlex SparkyKicker = new SparkFlex(Constants.KickerConstants.kKickerMotorId, MotorType.kBrushless);

  public Command kickerCommand() {
    return Commands.run(() -> SparkyKicker.set(KICKER_SPEED), this)
        .withName("kicker.Run");
  }

  public Command ForwardRollCommand() {
    return Commands.run(() -> {
      SparkyKicker.set(KICKER_SPEED);
    }, this).finallyDo(() -> {
      SparkyKicker.set(KICKER_SPEED_STOP);
    }).withName("kicker.ForwardRoll");
  }

  public Command BackwardRollCommand() {
    return Commands.run(() -> {
      SparkyKicker.set(-KICKER_SPEED);
    }, this).finallyDo(() -> {
      SparkyKicker.set(KICKER_SPEED_STOP);
    }).withName("kicker.BackwardRoll");
  }

  private void setKickerStop() {
    SparkyKicker.set(KICKER_SPEED_STOP);
  }
}
