/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

/*
 * Add your docs here.
 */
public class PID {
    double 
        integral, 
        previousError, 
        P, 
        I, 
        D,
        FFm,
        FFb, 
        setpoint;
        

    public PID (double kSetpoint, double kP, double kI, double kD, double kFFm, double kFFb) {
        this.P = kP;
        this.I = kI;
        this.D = kD;
        this.FFm = kFFm;
        this.FFb = kFFb;
        this.setpoint = kSetpoint;
    }

    public double run(double processingVar, double iterTime) {
        final double error = this.setpoint - processingVar;
        this.integral += (error * iterTime);
        final double derivitive = (error - this.previousError) / iterTime;
        this.previousError = error;
        return this.P*error + this.I*this.integral + this.D*derivitive + this.feedforward();
    }

    private double feedforward(){
        return this.FFm * this.setpoint + this.FFb;
    } 
    
    public void resetIntegral(){
        integral = 0;
    } 
    
    public void updateP(double kP){
        this.P = kP;
    }

    public void updateI(double kI){
        this.I = kI;
    }

    public void updateD(double kD){
        this.D = kD;
    }

    public void updateFF(double kFFm, double kFFb){
        this.FFm = kFFm;
        this.FFb = kFFb;
    }

    public void updateSetpoint(double kSetpoint){
        this.setpoint = kSetpoint;
    }
}
