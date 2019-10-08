/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.dynasty;

/**
 * Add your docs here.
 */
public abstract class Follower5010 {
	@FunctionalInterface
	public interface VelocitySource {
		/**
		 * Gets velocity data from the source.<br>
		 * <br>
		 * The units of the value returned depend on the distance per pulse set on the source.
		 * 
		 * @return The angle the robot is facing
		 */
		public double getVelocity();
	}
}
