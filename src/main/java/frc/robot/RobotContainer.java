// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.controls.DriverControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private final Telemetry logger = new Telemetry();

  public final CommandSwerveDrivetrain m_swerve = TunerConstants.createDrivetrain();

  public final SparkFlex hopperFloor; // ID 40
  public final SparkFlex kicker; // ID 41

  private static final CANBus kCANBus = new CANBus("Drivetrain");

  public final TalonFX hood; // ID 51
  public final TalonFX shooterPrimary; // ID 52
  public final TalonFX shooterSecondary; // ID 53

  public RobotContainer() {
    hopperFloor = new SparkFlex(40, MotorType.kBrushless);
    // hopperFloor.configure(new SparkBaseConfig().inverted(true), null, null);
    hopperFloor.setInverted(true);
    kicker = new SparkFlex(41, MotorType.kBrushless);

    shooterPrimary = new TalonFX(52, kCANBus);
    shooterSecondary = new TalonFX(53, kCANBus);

    // Shooter configs
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    shooterPrimary.getConfigurator().apply(shooterConfig);
    shooterSecondary.getConfigurator().apply(shooterConfig);

    shooterSecondary.setControl(new Follower(shooterPrimary.getDeviceID(), MotorAlignmentValue.Opposed));

    hood = new TalonFX(51, kCANBus);
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.CurrentLimits.SupplyCurrentLimit = 2.0;
    hood.getConfigurator().apply(hoodConfig);

    configureBindings();
  }

  private void configureBindings() {
    DriverControls.configure(Constants.ControllerConstants.kDriverControllerPort, m_swerve, logger);

    CommandXboxController oppController = new CommandXboxController(1);

    oppController.a().onTrue(Commands.runOnce(() -> {
      hopperFloor.set(0.5);
      kicker.set(0.5);
    })).onFalse(Commands.runOnce(() -> {
      hopperFloor.set(0);
      kicker.set(0);
    }));

    oppController.b().onTrue(Commands.runOnce(() -> {
      shooterPrimary.setControl(new DutyCycleOut(0.45));
      // shooterPrimary.setControl(new DutyCycleOut(0.67));
    })).onFalse(Commands.runOnce(() -> {
      shooterPrimary.setControl(new DutyCycleOut(0));
    }));

    double hoodSpeed = 0.5;

    oppController.x().onTrue(Commands.runOnce(() -> {
      hood.setControl(new DutyCycleOut(hoodSpeed));
    })).onFalse(Commands.runOnce(() -> {
      hood.setControl(new DutyCycleOut(0));
    }));

    oppController.y().onTrue(Commands.runOnce(() -> {
      hood.setControl(new DutyCycleOut(-hoodSpeed));
    })).onFalse(Commands.runOnce(() -> {
      hood.setControl(new DutyCycleOut(0));
    }));
  }

  public Command getAutonomousCommand() {
    return Commands.none();

    // Simple drive forward auton
    // final var idle = new SwerveRequest.Idle();
    // return Commands.sequence(
    // // Reset our field centric heading to match the robot
    // // facing away from our alliance station wall (0 deg).
    // m_swerve.runOnce(() -> m_swerve.seedFieldCentric(Rotation2d.kZero)),
    // // Then slowly drive forward (away from us) for 5 seconds.
    // m_swerve.applyRequest(() -> drive.withVelocityX(0.5)
    // .withVelocityY(0)
    // .withRotationalRate(0))
    // .withTimeout(5.0),
    // // Finally idle for the rest of auton
    // m_swerve.applyRequest(() -> idle));
  }

  public CommandSwerveDrivetrain getSwerveSystem() {
    return m_swerve;
  }
}
