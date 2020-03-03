
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

public class DriveTrain extends Subsystem 
{

  public CANSparkMax m1_dt = new CANSparkMax(RobotMap.m1_neo, MotorType.kBrushless);
  public CANSparkMax m2_dt = new CANSparkMax(RobotMap.m2_neo, MotorType.kBrushless);
  public CANSparkMax m3_dt = new CANSparkMax(RobotMap.m3_neo, MotorType.kBrushless);
  public CANSparkMax m4_dt = new CANSparkMax(RobotMap.m4_neo, MotorType.kBrushless);

  public CANEncoder encoder = m1_dt.getEncoder();
  
  public SpeedControllerGroup rm_dt = new SpeedControllerGroup(m4_dt,m2_dt);
  public SpeedControllerGroup lm_dt = new SpeedControllerGroup(m1_dt,m3_dt);

  public DifferentialDrive m_Drive = new DifferentialDrive(lm_dt,rm_dt);


  @Override
  public void initDefaultCommand() {

    //setDefaultCommand(new Move());

    m1_dt.restoreFactoryDefaults();
    m2_dt.restoreFactoryDefaults();
    m3_dt.restoreFactoryDefaults();
    m4_dt.restoreFactoryDefaults();

    m1_dt.setIdleMode(IdleMode.kCoast);
    m2_dt.setIdleMode(IdleMode.kCoast);
    m3_dt.setIdleMode(IdleMode.kCoast);
    m4_dt.setIdleMode(IdleMode.kCoast);

    m1_dt.setOpenLoopRampRate(2);
    m1_dt.setClosedLoopRampRate(2);
    m2_dt.setOpenLoopRampRate(2);
    m2_dt.setClosedLoopRampRate(2);
    m3_dt.setOpenLoopRampRate(2);
    m3_dt.setClosedLoopRampRate(2);
    m4_dt.setOpenLoopRampRate(2);
    m4_dt.setClosedLoopRampRate(2);

    encoder.setPosition(0.0);

  }

  public void setPower(double Y, double X) {

    m_Drive.arcadeDrive(Y, X);

  }

}
