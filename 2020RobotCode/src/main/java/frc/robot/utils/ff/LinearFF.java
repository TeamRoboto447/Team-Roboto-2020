/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.ff;

/**
 * Add your docs here.
 */
public class LinearFF extends FFbase {
    double
        M,
        B;
    public LinearFF ( double kM, double kB){
        this.M = kM;
        this.B = kB;
    }
    @Override
    public double getFF(double setpoint, double processingVar){
       return this.M * setpoint + this.B;
    }
    @Override
    public void update(double kM, double kB){
        this.M = kM;
        this.B = kB;
    }
    public double getM(){
        return this.M;
    }
    public double getB(){
        return this.B;
    }
}