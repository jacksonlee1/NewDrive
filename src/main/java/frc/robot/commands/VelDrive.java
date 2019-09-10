/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DtMain;

public class VelDrive extends Command {
  public VelDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(new DtMain());
  }
public Double forward = Robot.oi.d1.getRawAxis(1);
public Double turn = Robot.oi.d1.getRawAxis(4);
Double fPow;
Double turnPow;
double maxVel = RobotMap.k.maxVel;
Double maxTurn = RobotMap.k.maxTurn;
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    fPow = forward*maxVel;
    turnPow = turn*maxTurn;
    RobotMap.drivetrain.Drive(fPow,turnPow);
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
