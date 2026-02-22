package frc.robot.subsystems;
import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.parser.SwerveParser;
import swervelib.simulation.SwerveIMUSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.GyroSimulation;
import swervelib.telemetry.SwerveDriveTelemetry;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import swervelib.imu.SwerveIMU;
import swervelib.math.SwerveMath;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import frc.robot.Telemetry;
import swervelib.SwerveInputStream;
import org.littletonrobotics.junction.Logger;

public class SwerveSystem extends SubsystemBase {
    SwerveDriveKinematics m_kinematics;
    SwerveDriveOdometry m_odometry;
    SwerveIMU m_gyro;
    SwerveIMUSimulation m_simGyro;
    SwerveDrive swerveDrive;
    SwerveModule[] swerveModules;
    SwerveDriveOdometry odometry;
    SwerveParser parser;
    SwerveInputStream driveInputStream;
    LimelightSystem m_limelight;

    public SwerveSystem() {
        this.m_kinematics = Constants.SwerveDriveConstants.k_kinematics;
        this.m_limelight = Constants.SimulationConstants.k_isInSimulation ? null : new LimelightSystem(swerveDrive);

        File swerveDir = new File(Filesystem.getDeployDirectory(), "swerve"); 
        
        try {
            this.parser = new SwerveParser(swerveDir);
            this.swerveDrive = this.parser.createSwerveDrive(Constants.SwerveDriveConstants.k_maxSpeed);
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }

        this.swerveDrive.setModuleEncoderAutoSynchronize(true, 3);
        this.swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        this.swerveDrive.setHeadingCorrection(false);
        this.swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);

        this.m_gyro = this.swerveDrive.getGyro();
        this.m_simGyro = new SwerveIMUSimulation(new GyroSimulation(0, 0));

        Command onEnable = Commands.runOnce(() -> {
            Telemetry.log("running...");
            // Record that swerve was enabled
            Logger.recordOutput("Swerve/onEnabled", 1.0);
            if (Constants.SimulationConstants.k_isInSimulation){}
            else{
                m_limelight.onEnabled();
            }
        });

        RobotModeTriggers.teleop().onTrue(onEnable);
        RobotModeTriggers.autonomous().onTrue(onEnable);
        RobotModeTriggers.test().onTrue(onEnable);

        this.odometry = new SwerveDriveOdometry(
            this.m_kinematics,
            Constants.SimulationConstants.k_isInSimulation ? this.m_simGyro.getGyroRotation3d().toRotation2d() : this.m_gyro.getRotation3d().toRotation2d(),
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
            new Pose2d(0, 0, new Rotation2d())   
        );
    }

    @Override
    public void periodic(){
        m_limelight.periodic();
        // Publish useful telemetry for debugging and logging
        try {
            // Robot chassis velocities
            Telemetry.logNumber("Robot/vx_mps", swerveDrive.getRobotVelocity().vxMetersPerSecond);
            Telemetry.logNumber("Robot/vy_mps", swerveDrive.getRobotVelocity().vyMetersPerSecond);

            Logger.recordOutput("Robot/vx_mps", swerveDrive.getRobotVelocity().vxMetersPerSecond);
            Logger.recordOutput("Robot/vy_mps", swerveDrive.getRobotVelocity().vyMetersPerSecond);

            // Odometry pose
            Pose2d pose = this.odometry.getPoseMeters();
            Telemetry.logNumber("Robot/poseX_m", pose.getX());
            Telemetry.logNumber("Robot/poseY_m", pose.getY());
            Telemetry.logNumber("Robot/poseHeading_deg", pose.getRotation().getDegrees());

            Logger.recordOutput("Robot/poseX_m", pose.getX());
            Logger.recordOutput("Robot/poseY_m", pose.getY());
            Logger.recordOutput("Robot/poseHeading_deg", pose.getRotation().getDegrees());
        } catch (Exception ex) {
            // Avoid throwing from periodic if telemetry fails; still print for visibility
            Telemetry.logString("Swerve/telemetryError", ex.getMessage() == null ? "unknown" : ex.getMessage());
        }
        
    }

    @Override
    public void simulationPeriodic() {}

    public Command driveCommand(double translationX, double translationY,
      double angularRotationX) {
        return run(() -> {
        // Make the robot move
        swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
            translationX * swerveDrive.getMaximumChassisVelocity(),
            translationY * swerveDrive.getMaximumChassisVelocity()), 0.8),
            Math.pow(angularRotationX, 3) * swerveDrive.getMaximumChassisAngularVelocity(),
            true,
            false);
        });
    
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    public void driveSpeedCommand(double speedX, double speedY, double rot, boolean basedOnField) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveDriveConstants.k_kinematics.toSwerveModuleStates(
            basedOnField
            ? ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rot,
                Constants.SimulationConstants.k_isInSimulation ? this.m_simGyro.getGyroRotation3d().toRotation2d() : m_gyro.getRotation3d().toRotation2d()
            ) : new ChassisSpeeds(speedX, speedY, rot)
        );

        double maxBoostSpeed = Constants.SwerveDriveConstants.k_maxSpeed * Constants.SwerveDriveConstants.k_boostScaler;

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxBoostSpeed);

        SwerveModule[] modules = this.swerveDrive.getModules();
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(swerveModuleStates[i], false, false);
        }
    }

    public SwerveDrive getSwerveDrive() {
        return this.swerveDrive;
    }

    public SwerveDriveOdometry getOdometry(){
        return this.odometry;
    }
}
