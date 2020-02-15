/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.utils.Logging;
import frc.robot.utils.ff.FFbase;
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
        setpoint,
        maxInt,
        minInt,
        izone;
    Timer 
        iterTime,
        integralTimer;
    FFbase FF;

    public PID (double kSetpoint, double kP, double kI,
     double kD, FFbase kFF,
     double minIntegral, double maxIntegral) {
        this(kSetpoint, kP, kI, kD, kFF);
        this.minInt=minIntegral;
        this.maxInt=maxIntegral;
    }
    public PID (double kSetpoint, double kP, double kI, double kD, FFbase kFF, double kiZone){
        this(kSetpoint, kP, kI, kD, kFF);
        this.izone = kiZone;
    }
    public PID (double kSetpoint, double kP, double kI, double kD, FFbase kFF,
     double kiZone, double minIntegral, double maxIntegral){
        this(kSetpoint, kP, kI, kD, kFF);
        this.izone=kiZone;
        this.minInt=minIntegral;
        this.maxInt=maxIntegral;
    }
    public PID (double kSetpoint, double kP, double kI, double kD, FFbase kFF) {
        this.P = kP;
        this.I = kI;
        this.D = kD;
        this.FF = kFF;
        this.setpoint = kSetpoint;
        this.iterTime = new Timer();
        this.iterTime.reset();
        this.iterTime.stop();
        this.integralTimer = new Timer();
        this.integralTimer.reset();
        this.minInt=0;
        this.maxInt=0;
        this.izone=0;        
    }

    public double run(double processingVar) {
        double iterTime = this.iterTime.get();
        this.iterTime.start();
        this.iterTime.reset();
        
        if (this.integralTimer.get()>=Constants.shooterPidIntegralResetTime){
            this.resetIntegral();
            this.integralTimer.reset();
        }

        Logging.info("PID time: "+iterTime,"PIDtime");

        final double error = this.setpoint - processingVar;
        if ( Math.abs(error) <= this.izone || this.izone==0){
            this.integral += (error * iterTime);
        }
        if (this.minInt != 0 && this.integral<this.minInt){
            this.integral = this.minInt;
        } else if (this.maxInt != 0 && this.integral>this.maxInt){
            this.integral = this.maxInt;
        }

        final double derivitive = (error - this.previousError) / iterTime;
        
        Logging.info("P:" + this.P*error + ",I:"+this.I*this.integral + ",D:"+this.D*derivitive,"PIDvals");

        this.previousError = error;
        return this.P*error + this.I*this.integral + this.D*derivitive + this.FF.getFF(setpoint,processingVar,iterTime);
    }
    
    public void resetIntegral() {
        Logging.debug("Integral reset prev val: "+this.integral, "shootIntegral");
        this.integral = 0;
        this.iterTime.reset();
        this.iterTime.stop();
        Logging.debug("Integral reset after val: "+this.integral, "shootIntegral");
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

    public void updateSetpoint(double kSetpoint){
        this.setpoint = kSetpoint;
    }

    public double getP() {
        return this.P;
    }

    public double getI() {
        return this.I;
    }

    public double getD() {
        return this.D;
    }
    
    public double getSetpoint() {
        return this.setpoint;
    }
    public void updateFF(FFbase kFF){
        this.FF = kFF;
    }
    public FFbase getFF(){
        return this.FF;
    }
}
