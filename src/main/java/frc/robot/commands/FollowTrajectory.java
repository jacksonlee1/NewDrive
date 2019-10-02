/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Logger;
import frc.robot.RobotMap;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;
import robot.pathfinder.follower.TankFollower;

public class FollowTrajectory extends Command {
  private final TankDriveTrajectory trajectory;
  private TankFollower follower;
  Logger log;
 
  public FollowTrajectory(TankDriveTrajectory trajectory) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(RobotMap.drivetrain);
    this.trajectory = trajectory;
    log = new Logger(this.getClass().getCanonicalName());
  }

  // Called just before this Command runs the first time
  // Note we made this method public! This is so that Commands that wrap around
  // this one have an easier time.
  @Override
  public void initialize() {
    RobotMap.leftEncoder.reset();
    RobotMap.rightEncoder.reset();
    RobotMap.gyro.reset();
    log.clear();
    
    follower = new TankFollower(trajectory, RobotMap.L_MOTOR, RobotMap.R_MOTOR, 
    RobotMap.L_DISTANCE_SOURCE, RobotMap.R_DISTANCE_SOURCE, RobotMap.TIMESTAMP_SOURCE,
    RobotMap.GYRO, RobotMap.k.dtKv, RobotMap.k.dtKa, RobotMap.k.dtKp, RobotMap.k.dtKd, RobotMap.k.dtGkP);
    follower.initialize();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

    follower.run();
    log.entry("Out Left", follower.lastLeftOutput());
    log.entry("Out Right", follower.lastRightOutput());
    log.entry("Vel Left", follower.lastLeftVelocity());
    log.entry("Vel Right", follower.lastRightVelocity());
    log.entry("Acc Left", follower.lastLeftAcceleration());
    log.entry("Acc Right", follower.lastRightAcceleration());
    log.entry("Err Left", follower.lastLeftError());
    log.entry("Err Right", follower.lastRightError());
    log.writeEntries();
      SmartDashboard.putNumber("Follower Left Output", follower.lastLeftOutput());
      SmartDashboard.putNumber("Follower Right Output", follower.lastRightOutput());

      SmartDashboard.putNumber("Follower Left Velocity", follower.lastLeftVelocity());
      SmartDashboard.putNumber("Follower Right Velocity", follower.lastRightVelocity());

      SmartDashboard.putNumber("Follower Left Acceleration", follower.lastLeftAcceleration());
      SmartDashboard.putNumber("Follower Right Acceleration", follower.lastRightAcceleration());

      SmartDashboard.putNumber("Follower Left Error", follower.lastLeftError());
      SmartDashboard.putNumber("Follower Right Error", follower.lastRightError());

      SmartDashboard.putNumber("Follower Left Error Derivative", follower.lastLeftDerivative());
      SmartDashboard.putNumber("Follower Right Error Derivative", follower.lastRightDerivative());

      SmartDashboard.putNumber("Follower Directional Error", follower.lastDirectionalError());

      SmartDashboard.putNumber("Left Encoder Value", RobotMap.leftEncoder.get());
      SmartDashboard.putNumber("Right Encoder Value", RobotMap.rightEncoder.get());
     
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !follower.isRunning();
  }

  // Called once after isFinished returns true
  @Override
  public void end() {
    log.closeFileWriter();
    follower.stop();
    RobotMap.drivetrain.end();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  public void interrupted() {
    end();
  }
}
