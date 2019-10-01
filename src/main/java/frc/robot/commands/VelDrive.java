/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
//import frc.robot.subsystems.DtMain;

public class VelDrive extends Command {
  public Double forward;
  public Double turn;
  Double fPow;
  Double turnPow;
  double maxVel = RobotMap.k.maxVel;
  Double maxTurn = RobotMap.k.maxTurn;

  public VelDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(RobotMap.drivetrain);
  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.gyro.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    forward = -Robot.oi.d1.getRawAxis(1);
    turn = Robot.oi.d1.getRawAxis(4);

    fPow = forward*maxVel / 12;
    turnPow = turn*maxTurn / 12;
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
