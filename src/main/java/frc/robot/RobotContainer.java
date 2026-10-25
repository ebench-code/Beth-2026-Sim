// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RobotInitCommand;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.PowerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class RobotContainer {
    private final Pigeon2 pigeon2;
    private final PowerSubsystem powerSubsystem;
    private final SwerveSubsystem swerveSubsystem;

    private final CommandXboxController m_driverController;
    private Rotation2d lastAngle = new Rotation2d();

    public RobotContainer() {
        pigeon2 = new Pigeon2(1);
        MountPoseConfigs mountConfig = new MountPoseConfigs();
        mountConfig.MountPoseYaw = 0.0;
        mountConfig.MountPosePitch = 90.0;
        mountConfig.MountPoseRoll = 0.0;
        pigeon2.getConfigurator().apply(mountConfig);
        pigeon2.setYaw(0);

        m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

        powerSubsystem = new PowerSubsystem();
        swerveSubsystem = new SwerveSubsystem();

        Supplier<Double> joystickRobotSpin = () -> -MathUtil.applyDeadband(m_driverController.getRightX(), 0.1);

        Supplier<Rotation2d> robotHeadingAngle = () -> {
            double yaw = pigeon2.getYaw().getValueAsDouble();
            while (yaw >= 360.0) {
                yaw -= 360.0;
            }
            while (yaw <= 0) {
                yaw += 360.0;
            }
            return Rotation2d.fromDegrees(yaw);
        };
        Supplier<Translation2d> joystickRobotMovement = () -> new Translation2d(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.2),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.2)
        );

        configureBindings();
        swerveSubsystem.setDefaultCommand(
                new SwerveCommand(
                        swerveSubsystem,
                        robotHeadingAngle,
                        joystickRobotSpin,
                        joystickRobotMovement
                )
        );
    }

    private void configureBindings() {
        m_driverController.start().onTrue(Commands.runOnce(() -> {
            pigeon2.setYaw(0);
            lastAngle = Rotation2d.fromDegrees(0);
        }));

        m_driverController.povUp().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(0)));
        m_driverController.povUpLeft().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(45)));
        m_driverController.povLeft().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(90)));
        m_driverController.povDownLeft().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(135)));
        m_driverController.povDown().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(180)));
        m_driverController.povDownRight().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(-135)));
        m_driverController.povRight().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(-90)));
        m_driverController.povUpRight().onTrue(Commands.runOnce(() -> lastAngle = Rotation2d.fromDegrees(-45)));
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public RobotInitCommand getRobotInitCommand() {
        return new RobotInitCommand(
                swerveSubsystem,
                powerSubsystem
        );
    }
}
