package frc.robot.subsystems.StateManager;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.StateConstants;

public enum State {
  SHOOTING(StateConstants.hub),
  PASS_LEFT_SIDE(StateConstants.passLeftTarget),
  PASS_RIGHT_SIDE(StateConstants.passRightTarget),
  PASS_DEAD_ZONE(new Pose2d());

  public Pose2d targetPose;

  private State(Pose2d target) {
    this.targetPose = target;
  }
}
