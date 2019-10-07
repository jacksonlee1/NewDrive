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
		 * Gets orientation/directional data from the source.<br>
		 * <br>
		 * Please note that the result should be in radians, with 0 representing right. The representation
		 * should be the same as the angles used to generate the trajectory.
		 * @return The angle the robot is facing
		 */
		public double getVelocity();
	}
}
