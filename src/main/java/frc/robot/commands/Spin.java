/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Spin extends Command {
  public Spin() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.Spinner);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    Robot.Spinner.detectedColor = Robot.Spinner.m_colorSensor.getColor();

    Robot.Spinner.match = Robot.Spinner.m_colorMatcher.matchClosestColor(Robot.Spinner.detectedColor);

    Robot.Spinner.FMSColor = DriverStation.getInstance().getGameSpecificMessage();


    /////// Detect Color ///////

    if (Robot.Spinner.match.color == Robot.Spinner.kBlueTarget) {

      Robot.Spinner.colorString = "Blue";

    } else if (Robot.Spinner.match.color == Robot.Spinner.kRedTarget) {

      Robot.Spinner.colorString = "Red";

    } else if (Robot.Spinner.match.color == Robot.Spinner.kGreenTarget) {

      Robot.Spinner.colorString = "Green";

    } else if (Robot.Spinner.match.color == Robot.Spinner.kYellowTarget) {

      Robot.Spinner.colorString = "Yellow";

    } else {

      Robot.Spinner.colorString = "Unknown";

    }


    /////// Color Counter ///////
    
    if(Robot.Spinner.colorString == "Blue" && Robot.Spinner.prevcolor == "Red") {

      Robot.Spinner.counter = Robot.Spinner.counter + 1;

    }

    if(Robot.Spinner.colorString == "Blue" && Robot.Spinner.prevcolor == "Green") {

      Robot.Spinner.counter = Robot.Spinner.counter + 1;

    }

    if(Robot.Spinner.colorString == "Blue" && Robot.Spinner.prevcolor == "Yellow") {

      Robot.Spinner.counter = Robot.Spinner.counter + 1;

    }
    
    if(Robot.Spinner.colorString == "Red" && Robot.Spinner.prevcolor == "Blue") {

      Robot.Spinner.counter = Robot.Spinner.counter + 1;

    }

    if(Robot.Spinner.colorString == "Red" && Robot.Spinner.prevcolor == "Green") {

      Robot.Spinner.counter = Robot.Spinner.counter + 1;

    }

    if(Robot.Spinner.colorString == "Red" && Robot.Spinner.prevcolor == "Yellow" ) {

      Robot.Spinner.counter = Robot.Spinner.counter + 1;

    }
    
    if(Robot.Spinner.colorString == "Green" && Robot.Spinner.prevcolor == "Red") {

      Robot.Spinner.counter = Robot.Spinner.counter + 1;

    }

    if(Robot.Spinner.colorString == "Green" && Robot.Spinner.prevcolor == "Blue") {

      Robot.Spinner.counter = Robot.Spinner.counter + 1;

    }

    if(Robot.Spinner.colorString == "Green" && Robot.Spinner.prevcolor == "Yellow" ) {

      Robot.Spinner.counter = Robot.Spinner.counter + 1;

    }
    
    if(Robot.Spinner.colorString == "Yellow" && Robot.Spinner.prevcolor == "Red" ) {

      Robot.Spinner.counter = Robot.Spinner.counter + 1;

    }

    if(Robot.Spinner.colorString == "Yellow" && Robot.Spinner.prevcolor == "Green" ) {

      Robot.Spinner.counter = Robot.Spinner.counter + 1;

    }

    if(Robot.Spinner.colorString == "Yellow" && Robot.Spinner.prevcolor == "Blue" ) {

      Robot.Spinner.counter = Robot.Spinner.counter + 1;

    }


    /////// Stage 3 ///////

    if(Robot.Spinner.FMSColor.length() > 0) {
  switch (Robot.Spinner.FMSColor.charAt(0))
  {
    case 'B' :

    if(Robot.OI.getP1Button(RobotMap.X) && Robot.Spinner.colorString == "Red") {

      //Robot.Spinner.m1_sp.set(0.0);

    }else if(Robot.OI.getP1Button(RobotMap.X) && Robot.Spinner.colorString != "Red") {

      //Robot.Spinner.m1_sp.set(0.1);

    }

      break;
    case 'G' :

    if(Robot.OI.getP1Button(RobotMap.X) && Robot.Spinner.colorString == "Yellow") {

      //Robot.Spinner.m1_sp.set(0.0);

    }else if(Robot.OI.getP1Button(RobotMap.X) && Robot.Spinner.colorString != "Yellow") {

      //Robot.Spinner.m1_sp.set(0.1);

    }
  
      break;
    case 'R' :

    if(Robot.OI.getP1Button(RobotMap.X) && Robot.Spinner.colorString == "Blue") {

      //Robot.Spinner.m1_sp.set(0.0);

    }else if(Robot.OI.getP1Button(RobotMap.X) && Robot.Spinner.colorString != "Blue") {

      //Robot.Spinner.m1_sp.set(0.1);

    }

      break;
    case 'Y' :

    if(Robot.OI.getP1Button(RobotMap.X) && Robot.Spinner.colorString == "Green") {

      //Robot.Spinner.m1_sp.set(0.0);

    }else if(Robot.OI.getP1Button(RobotMap.X) && Robot.Spinner.colorString != "Green") {

      //Robot.Spinner.m1_sp.set(0.1);

    }

      break;
    default :
      //This is corrupt data
      break;
  }
} else {

      /////// Stage 2 ///////

      if(Robot.OI.getP1Button(RobotMap.X) == true && Robot.Spinner.counter < 24) {

        //Robot.Spinner.m1_sp.set(0.1);
  
      }else if(Robot.OI.getP1Button(RobotMap.X) == true && Robot.Spinner.counter == 25) {
  
        //Robot.Spinner.m1_sp.set(0.0);
  
      }

}

    Robot.Spinner.prevcolor = Robot.Spinner.colorString;

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
