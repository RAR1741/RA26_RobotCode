package frc.robot.subsystems;

public class KickerSystem extends SubsystemBase{
    private static final double KICKER_SPEED = 1.0;
    private static final double KICKER_SPEED_STOP = 0;
     private SparkMax SparkyKicker = new SparkMax(IntakeConstants.k_rollerMotorId, MotorType.kBrushless);


    public Command kickerCommand() {
    return Commands.run(() -> SparkyKicker.set(KICKER_SPEED), this)
        .withName("kicker.Run");

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
}


