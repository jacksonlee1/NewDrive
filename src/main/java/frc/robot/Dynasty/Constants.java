/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Dynasty;



/**
 * Add your docs here.
 */
public class Constants {
    public Double dtLKp;
    public Double dtRKp;
    public Double dtLKd;
    public Double dtRKd;
    public double dtLeftFwdVint;
    public double dtLeftRevVint;
    public double dtRightFwdVint;
    public double dtRightRevVint;
    public Double maxVel;
    public Double maxTurn;

    public Constants(){
        dtLKp = .1;
        dtRKp = .1;
        dtLKd = .1;
        dtRKd = .1;
        dtLeftFwdVint = 1.1;
        dtLeftRevVint = -1.1; 
        dtRightFwdVint = 1.1;
        dtRightRevVint = -1.1;
        maxVel = 14.0;
        maxTurn = 7.0;
    }

}
