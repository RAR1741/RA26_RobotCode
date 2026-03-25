package frc.robot.subsystems.StateManager;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.StateConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.Zone;

public class StateManager extends SubsystemBase {
  private static StateManager instance;
  private State state;

  // Corners of zone as an array
  private Translation2d[] decapitationZoneTranslation = new Translation2d[4];
  private Translation2d[] passZoneOneTranslation = StateConstants.passZoneOne;
  private Translation2d[] passZoneTwoTranslation = StateConstants.passZoneTwo;
  private Translation2d[] passZoneThreeTranslation = StateConstants.passZoneThree;
  private Translation2d[] passZoneFourTranslation = StateConstants.passZoneFour;
  private Translation2d[] passZoneFiveTranslation = StateConstants.passZoneFive;

  private Zone decapitationZone;
  private Zone passZoneOne;
  private Zone passZoneTwo;
  private Zone passZoneThree;
  private Zone passZoneFour;
  private Zone passZoneFive;

  private Alliance alliance;
  private Pose2d pose;
  private CommandSwerveDrivetrain drivetrain;
  private Superstructure superstructure;

  public final Trigger hasValidTarget = new Trigger(() -> !state.equals(State.PASS_DEAD_ZONE));
  public final Trigger inDecapitationZone = new Trigger(() -> OperationStates.inDecapitationZone);

  public static class OperationStates {
    public static boolean inDecapitationZone = false;
    public static boolean inPassZone1 = false;
    public static boolean inPassZone2 = false;
    public static boolean inPassZone3 = false;
    public static boolean inPassZone4 = false;
    public static boolean inPassZone5 = false;
  }

  public static StateManager initalize(CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    instance = new StateManager();
    instance.drivetrain = drivetrain;
    instance.superstructure = superstructure;
    return instance;
  }

  public static StateManager getInstance(CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    if (instance != null) {
      return instance;
    } else {
      return initalize(drivetrain, superstructure);
    }
  }

  private StateManager() {
    this.decapitationZone = new Zone();
    this.passZoneOne = new Zone(passZoneOneTranslation);
    this.passZoneTwo = new Zone(passZoneTwoTranslation);
    this.passZoneThree = new Zone(passZoneThreeTranslation);
    this.passZoneFour = new Zone(passZoneFourTranslation);
    this.passZoneFive = new Zone(passZoneFiveTranslation);

    passZoneOne.logPoints("passZone1");
    passZoneTwo.logPoints("passZone2");
    passZoneThree.logPoints("passZone3");
    passZoneFour.logPoints("passZone4");
    passZoneFive.logPoints("passZone5");

    this.state = State.SHOOTING;
  }

  private void setState(State state) {
    this.state = state;
  }

  public Pose2d getTargetPose() {
    // Mirror pose if Red Alliance
    if (alliance != null && alliance.equals(Alliance.Red)) {
      return state.targetPose.rotateAround(
          StateConstants.centerField, new Rotation2d(Math.PI));
    }
    return state.targetPose;
  }

  public State getState() {
    return this.state;
  }

  public boolean canTurretMove() {
    // return (TurretConstants.canMove && getState() != State.PASS_DEAD_ZONE);
    return (true && getState() != State.PASS_DEAD_ZONE);
  }

  @Override
  public void periodic() {
    // Make pass zones red alliance if needed
    if (alliance == null && DriverStation.getAlliance().isPresent()) {
      alliance = DriverStation.getAlliance().get();

      if (alliance.equals(Alliance.Red)) {
        passZoneOne.updateZone(toRedAlliance(passZoneOneTranslation));
        passZoneTwo.updateZone(toRedAlliance(passZoneTwoTranslation));
        passZoneThree.updateZone(toRedAlliance(passZoneThreeTranslation));
        passZoneFour.updateZone(toRedAlliance(passZoneFourTranslation));
        passZoneFive.updateZone(toRedAlliance(passZoneFiveTranslation));

        passZoneOne.logPoints("passZone1");
        passZoneTwo.logPoints("passZone2");
        passZoneThree.logPoints("passZone3");
        passZoneFour.logPoints("passZone4");
        passZoneFive.logPoints("passZone5");
      }
    }

    updateDecapitationZone();
    decapitationZone.logPoints("DecapitationZone");

    pose = drivetrain.getState().Pose;

    OperationStates.inDecapitationZone = decapitationZone.contains(pose);
    OperationStates.inPassZone1 = passZoneOne.contains(pose);
    OperationStates.inPassZone2 = passZoneTwo.contains(pose);
    OperationStates.inPassZone3 = passZoneThree.contains(pose);
    OperationStates.inPassZone4 = passZoneFour.contains(pose);
    OperationStates.inPassZone5 = passZoneFive.contains(pose);

    // Set State
    if (OperationStates.inPassZone1 || OperationStates.inPassZone3) {
      setState(State.PASS_RIGHT_SIDE);
    } else if (OperationStates.inPassZone2 || OperationStates.inPassZone4) {
      setState(State.PASS_LEFT_SIDE);
    } else if (OperationStates.inPassZone5) {
      setState(State.PASS_DEAD_ZONE);
    } else {
      setState(State.SHOOTING);
    }

    // Print out the current zone
    if (OperationStates.inPassZone1)
      passZoneOne.logPoints("currentZone");
    else if (OperationStates.inPassZone2)
      passZoneTwo.logPoints("currentZone");
    else if (OperationStates.inPassZone3)
      passZoneThree.logPoints("currentZone");
    else if (OperationStates.inPassZone4)
      passZoneFour.logPoints("currentZone");
    else if (OperationStates.inPassZone5)
      passZoneFive.logPoints("currentZone");
    else
      Logger.recordOutput("Zones/currentZone", new Pose2d[] { new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d() });

    // Logging
    Logger.recordOutput(
        "StateManager/OperationStates/InDecapitationZone", OperationStates.inDecapitationZone);
    Logger.recordOutput(
        "StateManager/OperationStates/InPassZone1", OperationStates.inPassZone1);
    Logger.recordOutput(
        "StateManager/OperationStates/InPassZone2", OperationStates.inPassZone2);
    Logger.recordOutput(
        "StateManager/OperationStates/InPassZone3", OperationStates.inPassZone3);
    Logger.recordOutput(
        "StateManager/OperationStates/InPassZone4", OperationStates.inPassZone4);
    Logger.recordOutput(
        "StateManager/OperationStates/InPassZone5", OperationStates.inPassZone5);

    Logger.recordOutput("StateManager/State", state);
    Logger.recordOutput("StateManager/StateTargetPose", getTargetPose());

    Logger.recordOutput(
        "StateManager/IsReady/hasValidTarget", hasValidTarget.getAsBoolean());
    Logger.recordOutput(
        "StateManager/IsReady/Shooter", superstructure.shooter.isAtTarget.getAsBoolean());
    Logger.recordOutput(
        "StateManager/IsReady/Turret", superstructure.turret.isAtTarget.getAsBoolean());
    Logger.recordOutput(
        "StateManager/IsReady/Hood", superstructure.hood.isAtTarget.getAsBoolean());
  }

  // Update zone based off the closest trench and robot velocity
  private void updateDecapitationZone() {
    Pose2d robotPose = drivetrain.getState().Pose;
    Translation2d robotTranslation = robotPose.getTranslation();

    Translation2d closest = robotTranslation.nearest(StateConstants.trenchList);

    // box dimensions scale with velocity
    var fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
        drivetrain.getState().Speeds, robotPose.getRotation());

    // dimensions of no auto score zone
    double boxXDim = 1 + HoodConstants.boxXMultiplier * Math.abs(fieldRelative.vxMetersPerSecond);
    double boxYDim = StateConstants.trenchWidth
        + HoodConstants.boxYMultiplier * Math.abs(fieldRelative.vyMetersPerSecond);

    Translation2d topLeft = new Translation2d(closest.getX() - boxXDim, closest.getY() + boxYDim);
    Translation2d topRight = new Translation2d(closest.getX() + boxXDim, closest.getY() + boxYDim);
    Translation2d botLeft = new Translation2d(closest.getX() - boxXDim, closest.getY() - boxYDim);
    Translation2d botRight = new Translation2d(closest.getX() + boxXDim, closest.getY() - boxYDim);

    decapitationZoneTranslation[0] = topLeft;
    decapitationZoneTranslation[1] = topRight;
    decapitationZoneTranslation[2] = botLeft;
    decapitationZoneTranslation[3] = botRight;

    decapitationZone.updateZone(decapitationZoneTranslation);
  }

  private Translation2d[] toRedAlliance(Translation2d[] blueAllianceTranslation) {
    Translation2d[] redAllianceTranslation = new Translation2d[4];
    for (int i = 0; i < blueAllianceTranslation.length; i++) {
      redAllianceTranslation[i] = blueAllianceTranslation[i].rotateAround(
          StateConstants.centerField, new Rotation2d(Math.PI));
    }
    return redAllianceTranslation;
  }
}
