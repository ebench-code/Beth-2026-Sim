// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  
  private Joystick m_stick;
  private static final int deviceID = 2;
  private SparkMax m_motor;
  private SparkClosedLoopController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    m_stick = new Joystick(0);

    // initialize motor
    m_motor = new SparkMax(deviceID, MotorType.kBrushless);

    m_pidController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kP = 0.04; 
    kI = 0;
    kD = 0; 
    kFF = 0.208; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // 2026 REV configuration model
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop
        .p(kP)
        .i(kI)
        .d(kD)
        .velocityFF(kFF)
        .outputRange(kMinOutput, kMaxOutput);

    m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
  }

  public Command startMotor() {
    return runOnce(() -> m_motor.set(0.5));
  }

  public Command stopMotor() {
    return runOnce(() -> m_motor.set(0));
  }

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
