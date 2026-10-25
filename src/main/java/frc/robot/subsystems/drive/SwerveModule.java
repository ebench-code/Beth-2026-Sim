package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    public final SparkMax moveMotor;
    public final SparkMax rotateMotor;
    public final CANcoder rotationEncoder;
    public final SwerveConfiguration swerveConfiguration;
    public final PIDController pidController;
    public final SwerveConfiguration config;

    public SwerveModule(
            SwerveConfiguration swerveConfiguration
    ) {
        this.config = swerveConfiguration;
        this.moveMotor = new SparkMax(swerveConfiguration.moveCANSparkMaxId, MotorType.kBrushless);
        this.rotateMotor = new SparkMax(swerveConfiguration.rotateCANSparkMaxId, MotorType.kBrushless);
        this.rotationEncoder = new CANcoder(swerveConfiguration.canCoderId);
        this.swerveConfiguration = swerveConfiguration;
        this.pidController = new PIDController(0.01, 0, 0.0001);
    }

    public Translation2d getPosition() {
        return this.swerveConfiguration.position;
    }

    public void init() {
        setupSparkMax(this.moveMotor, false);
        setupSparkMax(this.rotateMotor, true);

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoderConfig.MagnetSensor.MagnetOffset = swerveConfiguration.magneticOffset / 360.0;

        rotationEncoder.getConfigurator().apply(canCoderConfig);

        pidController.enableContinuousInput(-180, 180);
        pidController.setTolerance(5, 10);
    }

    private void setupSparkMax(SparkMax sparkMax, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted);
        config.smartCurrentLimit(30);
        config.idleMode(IdleMode.kBrake);
        config.voltageCompensation(12.0);

        sparkMax.clearFaults();
        sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void apply(SwerveModuleState state) {
        double encoderAbsoluteDegrees = rotationEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
        Rotation2d position = Rotation2d.fromDegrees(encoderAbsoluteDegrees);
        state = SwerveModuleState.optimize(state, position);

        double mps = MathUtil.clamp(state.speedMetersPerSecond, -1, 1);
        SmartDashboard.putNumber(String.format("%s mps", swerveConfiguration.name), mps);
        SmartDashboard.putNumber(String.format("%s deg", swerveConfiguration.name), state.angle.getDegrees());
        SmartDashboard.putNumber(String.format("%s encoder", swerveConfiguration.name), encoderAbsoluteDegrees);

        double pid = pidController.calculate(encoderAbsoluteDegrees, state.angle.getDegrees());
        SmartDashboard.putNumber(String.format("%s pid", swerveConfiguration.name), pid);
        SmartDashboard.putNumber(String.format("%s pid error", swerveConfiguration.name), pidController.getPositionError());
        if (!pidController.atSetpoint()) {
            rotateMotor.set(pid);
        }

        moveMotor.set(mps);
    }
}
