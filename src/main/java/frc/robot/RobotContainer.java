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
import frc.robot.controls.OperatorControls;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
  private final Telemetry logger = new Telemetry();

  private final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
  private final Superstructure superstructure = new Superstructure();

  public final SparkFlex hopperFloor; // ID 40
  public final SparkFlex kicker; // ID 41

  private static final CANBus kCANBus = new CANBus("Drivetrain");

  public final TalonFX hood; // ID 51
  public final TalonFX shooterPrimary; // ID 52
  public final TalonFX shooterSecondary; // ID 53

  public final TurretSubsystem turret = new TurretSubsystem();

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
    buildNamedAutoCommands();
  }

  private void configureBindings() {
    DriverControls.configure(Constants.ControllerConstants.kDriverControllerPort, swerve, superstructure, logger);
    OperatorControls.configure(Constants.ControllerConstants.kOperatorControllerPort, swerve, superstructure);
  }

  private void buildNamedAutoCommands() {
    // Add any auto commands to the NamedCommands here
    // NamedCommands.registerCommand("driveForwards",
    // drivebase.driveForward().withTimeout(2)
    // .withName("Auto.driveForwards"));
  }

  public Command getAutonomousCommand() {
    return Commands.none();

    // Simple drive forward auton
    // final var idle = new SwerveRequest.Idle();
    // return Commands.sequence(
    // // Reset our field centric heading to match the robot
    // // facing away from our alliance station wall (0 deg).
    // swerve.runOnce(() -> swerve.seedFieldCentric(Rotation2d.kZero)),
    // // Then slowly drive forward (away from us) for 5 seconds.
    // swerve.applyRequest(() -> drive.withVelocityX(0.5)
    // .withVelocityY(0)
    // .withRotationalRate(0))
    // .withTimeout(5.0),
    // // Finally idle for the rest of auton
    // swerve.applyRequest(() -> idle));
  }

  public CommandSwerveDrivetrain getSwerveSystem() {
    return swerve;
  }
}
