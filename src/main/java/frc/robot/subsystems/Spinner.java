/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotMap;
import frc.robot.commands.Spin;

public class Spinner extends Subsystem {

  public CANSparkMax m1_sp = new CANSparkMax(RobotMap.m7_neo, MotorType.kBrushless);

  public Color detectedColor;

  public ColorMatchResult match;

  public double counter;

  public String colorString;
  public String prevcolor;
  public String FMSColor;

  public final I2C.Port i2cPort = I2C.Port.kOnboard;

  public final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  public final ColorMatch m_colorMatcher = new ColorMatch();
  
  public final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  public final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  public final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  public final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);


  @Override
  public void initDefaultCommand() {

    setDefaultCommand(new Spin());

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);   

  }
}
