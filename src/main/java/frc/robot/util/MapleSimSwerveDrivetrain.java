package frc.robot.util;

// Copyright 2021-2025 Iron Maple 5516
// Original Source:
// https://github.com/Shenzhen-Robotics-Alliance/CTRE-Swerve-MapleSim
//
// This code is licensed under MIT license (see https://mit-license.org/)

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Injects Maple-Sim simulation data into a CTRE Phoenix 6 swerve drivetrain.
 *
 * <p>Retrieves physics simulation results from Maple-Sim and injects them into the CTRE
 * SimState objects (TalonFX, CANcoder, Pigeon2), replacing the stock CTRE SimSwerveDrivetrain.
 */
public class MapleSimSwerveDrivetrain {
    private final Pigeon2SimState pigeonSim;
    private final SimSwerveModule[] simModules;
    public final SwerveDriveSimulation mapleSimDrive;

    /**
     * Constructs a Maple-Sim backed drivetrain simulation.
     *
     * @param simPeriod            time period of one simulation step
     * @param robotMassWithBumpers total mass including bumpers
     * @param bumperLengthX        bumper length along X (forward/back)
     * @param bumperWidthY         bumper width along Y (left/right)
     * @param driveMotorModel      DCMotor model for drive motors
     * @param steerMotorModel      DCMotor model for steer motors
     * @param wheelCOF             coefficient of friction of drive wheels
     * @param moduleLocations      module locations FL, FR, BL, BR
     * @param pigeon               Pigeon2 from the drivetrain
     * @param modules              SwerveModules from SwerveDrivetrain.getModules()
     * @param moduleConstants      per-module constants starting with Front Left
     */
    @SuppressWarnings("unchecked")
    public MapleSimSwerveDrivetrain(
            Time simPeriod,
            Mass robotMassWithBumpers,
            Distance bumperLengthX,
            Distance bumperWidthY,
            DCMotor driveMotorModel,
            DCMotor steerMotorModel,
            double wheelCOF,
            Translation2d[] moduleLocations,
            Pigeon2 pigeon,
            SwerveModule<TalonFX, TalonFX, CANcoder>[] modules,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConstants) {
        this.pigeonSim = pigeon.getSimState();
        simModules = new SimSwerveModule[moduleConstants.length];

        DriveTrainSimulationConfig simulationConfig = DriveTrainSimulationConfig.Default()
                .withRobotMass(robotMassWithBumpers)
                .withBumperSize(bumperLengthX, bumperWidthY)
                .withGyro(COTS.ofPigeon2())
                .withCustomModuleTranslations(moduleLocations)
                .withSwerveModule(new SwerveModuleSimulationConfig(
                        driveMotorModel,
                        steerMotorModel,
                        moduleConstants[0].DriveMotorGearRatio,
                        moduleConstants[0].SteerMotorGearRatio,
                        Volts.of(moduleConstants[0].DriveFrictionVoltage),
                        Volts.of(moduleConstants[0].SteerFrictionVoltage),
                        Meters.of(moduleConstants[0].WheelRadius),
                        KilogramSquareMeters.of(moduleConstants[0].SteerInertia),
                        wheelCOF));

        mapleSimDrive = new SwerveDriveSimulation(simulationConfig, new Pose2d());

        SwerveModuleSimulation[] moduleSimulations = mapleSimDrive.getModules();
        for (int i = 0; i < simModules.length; i++) {
            simModules[i] = new SimSwerveModule(moduleConstants[0], moduleSimulations[i], modules[i]);
        }

        SimulatedArena.overrideSimulationTimings(simPeriod, 1);
        SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
    }

    /**
     * Runs one simulation step: advances the MapleSim physics engine and injects
     * results (positions, velocities, heading) into the CTRE SimState objects.
     * Called from a Notifier at the configured sim loop period.
     */
    public void update() {
        SimulatedArena.getInstance().simulationPeriodic();
        pigeonSim.setRawYaw(
                mapleSimDrive.getSimulatedDriveTrainPose().getRotation().getMeasure());
        pigeonSim.setAngularVelocityZ(RadiansPerSecond.of(
                mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative().omegaRadiansPerSecond));
    }

    /** Represents the simulation of a single swerve module. */
    protected static class SimSwerveModule {
        public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                moduleConstant;
        public final SwerveModuleSimulation moduleSimulation;

        public SimSwerveModule(
                SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> moduleConstant,
                SwerveModuleSimulation moduleSimulation,
                SwerveModule<TalonFX, TalonFX, CANcoder> module) {
            this.moduleConstant = moduleConstant;
            this.moduleSimulation = moduleSimulation;
            moduleSimulation.useDriveMotorController(
                    new TalonFXMotorControllerSim(module.getDriveMotor()));
            moduleSimulation.useSteerMotorController(
                    new TalonFXMotorControllerWithRemoteCanCoderSim(module.getSteerMotor(), module.getEncoder()));
        }
    }

    /** Bridges a TalonFX SimState with the Maple-Sim motor controller interface. */
    public static class TalonFXMotorControllerSim implements SimulatedMotorController {
        private final TalonFXSimState talonFXSimState;

        public TalonFXMotorControllerSim(TalonFX talonFX) {
            this.talonFXSimState = talonFX.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            talonFXSimState.setRawRotorPosition(encoderAngle);
            talonFXSimState.setRotorVelocity(encoderVelocity);
            talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            return talonFXSimState.getMotorVoltageMeasure();
        }
    }

    /** Bridges a TalonFX + remote CANcoder SimState with the Maple-Sim motor controller interface. */
    public static class TalonFXMotorControllerWithRemoteCanCoderSim extends TalonFXMotorControllerSim {
        private final CANcoderSimState remoteCancoderSimState;

        public TalonFXMotorControllerWithRemoteCanCoderSim(TalonFX talonFX, CANcoder cancoder) {
            super(talonFX);
            this.remoteCancoderSimState = cancoder.getSimState();
        }

        @Override
        public Voltage updateControlSignal(
                Angle mechanismAngle,
                AngularVelocity mechanismVelocity,
                Angle encoderAngle,
                AngularVelocity encoderVelocity) {
            remoteCancoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
            remoteCancoderSimState.setRawPosition(mechanismAngle);
            remoteCancoderSimState.setVelocity(mechanismVelocity);
            return super.updateControlSignal(mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
        }
    }

    /**
     * Patches all module constants for simulation compatibility. Zeroes encoder offsets,
     * disables motor inversions, and replaces real-robot steer PID with sim-tuned gains.
     * No-op on real hardware.
     */
    @SuppressWarnings("rawtypes")
    public static SwerveModuleConstants[] regulateModuleConstantsForSimulation(
            SwerveModuleConstants[] moduleConstants) {
        for (SwerveModuleConstants moduleConstant : moduleConstants) {
            regulateModuleConstantForSimulation(moduleConstant);
        }
        return moduleConstants;
    }

    @SuppressWarnings("rawtypes")
    private static void regulateModuleConstantForSimulation(SwerveModuleConstants moduleConstants) {
        if (RobotBase.isReal()) return;

        moduleConstants
                .withEncoderOffset(0)
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
                .withSteerMotorGains(new Slot0Configs()
                        .withKP(70)
                        .withKI(0)
                        .withKD(4.5)
                        .withKS(0)
                        .withKV(1.91)
                        .withKA(0)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
                .withSteerMotorGearRatio(16.0)
                .withDriveFrictionVoltage(Volts.of(0.1))
                .withSteerFrictionVoltage(Volts.of(0.05))
                .withSteerInertia(KilogramSquareMeters.of(0.05));
    }
}
