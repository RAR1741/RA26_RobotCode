package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;

public final class Simulation {
  private SimulatedArena m_arena;
  private CommandSwerveDrivetrain m_swerve;

  public Simulation(CommandSwerveDrivetrain swerveDrive) {
    m_swerve = swerveDrive;
    m_arena = SimulatedArena.getInstance();
  }

  public void init() {
    m_arena.resetFieldForAuto();
  }

  public void periodic() {
    m_arena.simulationPeriodic();

    Logger.recordOutput("FieldSimulation/Fuel", m_arena.getGamePiecesArrayByType("Fuel"));

    Logger.recordOutput("Sim/RobotPose", m_swerve.getState().Pose);
  }

  public static Command fireFuel(CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    return Commands.runOnce(() -> {
      SimulatedArena arena = SimulatedArena.getInstance();

      GamePieceProjectile fuel = new RebuiltFuelOnFly(
          drivetrain.getState().Pose.getTranslation(),
          new Translation2d(
              superstructure.turret.getTurretTranslation().getX() * -1,
              superstructure.turret.getTurretTranslation().getY() * -1),
          drivetrain.getState().Speeds,
          drivetrain.getState().Pose.getRotation().rotateBy(superstructure.getAimRotation3d().toRotation2d()),
          superstructure.turret.getTurretTranslation().getMeasureZ(),

          // Full tangential velocity applied to the fuel as we shoot it
          // (this can be scaled down to simulate whatever spin we actually have)
          superstructure.shooter.getTangentialVelocity().times(1.0),
          superstructure.getHoodAngle());

      // Configure callbacks to visualize the flight trajectory of the projectile
      fuel.withProjectileTrajectoryDisplayCallBack(
          // Callback for when the note will eventually hit the target (if configured)
          (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileSuccessfulShot",
              pose3ds.toArray(Pose3d[]::new)),
          // Callback for when the note will eventually miss the target, or if no target
          // is configured
          (pose3ds) -> Logger.recordOutput("FieldSimulation/Shooter/ProjectileUnsuccessfulShot",
              pose3ds.toArray(Pose3d[]::new)));

      arena.addGamePieceProjectile(fuel);
    });
  }
}
