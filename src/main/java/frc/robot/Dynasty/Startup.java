/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.dynasty;

import frc.robot.RobotMap;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.path.PathType;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;

/**
 * Add your docs here.
 */
public class Startup {
    public Startup(int count){
     pathFinderWarmUp(count);
    }
    public static long pathFinderWarmUp(int count){
        for(int i = 0; i < count; i ++) {
            long start = System.currentTimeMillis();
            TrajectoryParams params = new TrajectoryParams();
            params.alpha = Math.random() * 200;
            params.isTank = true;
            params.pathType = PathType.QUINTIC_HERMITE;
            params.segmentCount = 500;
            params.waypoints = new Waypoint[] {
                new Waypoint(0, 0, Math.PI / 2),
                new Waypoint(Math.random() * 100, Math.random() * 100, Math.PI * 2 * Math.random()),
            };
            @SuppressWarnings("unused")
            TankDriveTrajectory trajectory = new TankDriveTrajectory(RobotMap.robotSpecs, params);
            if(i == count - 1) {
                return System.currentTimeMillis() - start;
            }
        }
        System.gc();
        return -1;
    }
}

