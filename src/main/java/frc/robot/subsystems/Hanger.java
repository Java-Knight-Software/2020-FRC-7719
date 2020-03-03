/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.Hang;

public class Hanger extends Subsystem {

  public CANSparkMax m1_hr = new CANSparkMax(RobotMap.m8_neo, MotorType.kBrushless);
  public CANSparkMax m2_hr = new CANSparkMax(RobotMap.m9_neo, MotorType.kBrushless);

  public CANEncoder encoder = m1_hr.getEncoder();

  public final double kDriveTick2Feet = 10.0 / 128 * 6 * Math.PI / 10;

  @Override
  public void initDefaultCommand() {

    setDefaultCommand(new Hang());

    m1_hr.restoreFactoryDefaults();
    m2_hr.restoreFactoryDefaults();

  }

  public void setPower(double power){
    m1_hr.set(power);
  }

}
