// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {

  private CANSparkMax m_leftClimber;
  private CANSparkMax m_rightClimber;

  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_RightEncoder;

  private DigitalInput m_leftHallEffectSensor;
  private DigitalInput m_rightHallEffectSensor;

  //left is 1
  //right is 0

  public Climber() {

    m_leftClimber =
      new CANSparkMax(Constants.Climber.leftClimberID, CANSparkLowLevel.MotorType.kBrushless);
    m_rightClimber =
      new CANSparkMax(Constants.Climber.rightClimberID, CANSparkLowLevel.MotorType.kBrushless);
    
      m_leftHallEffectSensor = new DigitalInput(0);
      m_rightHallEffectSensor = new DigitalInput(1);
      
      m_leftEncoder = m_leftClimber.getEncoder();
      m_RightEncoder = m_rightClimber.getEncoder();

      m_leftClimber.enableSoftLimit(SoftLimitDirection.kForward, true);
      m_rightClimber.enableSoftLimit(SoftLimitDirection.kForward, true);
      m_leftClimber.enableSoftLimit(SoftLimitDirection.kReverse, true);
      m_rightClimber.enableSoftLimit(SoftLimitDirection.kReverse, true);
     
      m_leftClimber.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Climber.maxClimberHeight);
      m_leftClimber.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Climber.minClimberHeight);
      m_rightClimber.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Climber.maxClimberHeight);
      m_rightClimber.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Climber.minClimberHeight); 

      m_rightClimber.burnFlash();
      m_leftClimber.burnFlash();
  }

  public void IncreasePower()
  {
    if(m_RightEncoder.getPosition() < Constants.Climber.maxClimberHeight)
      m_rightClimber.set(0.5);

    if(m_leftEncoder.getPosition() < Constants.Climber.maxClimberHeight)
     m_leftClimber.set(0.5);
  }

  public void DecreasePower()
  {
    if(m_RightEncoder.getPosition() >0)
      m_rightClimber.set(-0.5);

    if(m_leftEncoder.getPosition() >0)
      m_leftClimber.set(-0.5);
  }

  public void NoPower()
  {
      m_rightClimber.set(0);
      m_leftClimber.set(0);
  }

  @Override
  public void periodic() {

    
    if(!m_rightHallEffectSensor.get())
      m_RightEncoder.setPosition(0);
    if(!m_leftHallEffectSensor.get())
      m_leftEncoder.setPosition(0); 
    
    SmartDashboard.putNumber("Sh/Climber Position Left", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Sh/Climber Position Right", m_RightEncoder.getPosition());
    SmartDashboard.putBoolean("Sh/Climber Left Magnet Sensor", m_leftHallEffectSensor.get());
    SmartDashboard.putBoolean("Sh/Climber Right Magnet Sensor", m_rightHallEffectSensor.get());
  }
}
