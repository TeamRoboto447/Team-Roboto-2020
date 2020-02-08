/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.utils.logging;
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
        setpoint,
        maxInt,
        minInt,
        minIntErr;
    Timer 
        iterTime,
        integralTimer;

    public PID (double kSetpoint, double kP, double kI,
    double kD, double kFFm, double kFFb,
    double minIntegral, double maxIntegral) {
        this.P = kP;
        this.I = kI;
        this.D = kD;
        this.FFm = kFFm;
        this.FFb = kFFb;
        this.setpoint = kSetpoint;
        this.iterTime = new Timer();
        this.iterTime.reset();
        this.iterTime.stop();
        this.integralTimer = new Timer();
        this.integralTimer.reset();
        this.minInt=minIntegral;
        this.maxInt=maxIntegral;
    }
    public PID (double kSetpoint, double kP, double kI, double kD, double kFFm, double kFFb) {
        this.P = kP;
        this.I = kI;
        this.D = kD;
        this.FFm = kFFm;
        this.FFb = kFFb;
        this.setpoint = kSetpoint;
        this.iterTime = new Timer();
        this.iterTime.reset();
        this.iterTime.stop();
        this.integralTimer = new Timer();
        this.integralTimer.reset();
        this.minInt=0;
        this.maxInt=0;
    }

    public double run(double processingVar) {
        double iterTime = this.iterTime.get();
        this.iterTime.start();
        this.iterTime.reset();
        
        if (this.integralTimer.get()>=Constants.pidIntegralResetTime){
            this.resetIntegral();
            this.integralTimer.reset();
        }

        logging.info("PID time: "+iterTime,"PIDtime");

        final double error = this.setpoint - processingVar;
        this.integral += (error * iterTime);
        final double derivitive = (error - this.previousError) / iterTime;
        
        if (this.minInt != 0 && this.integral<this.minInt){
            this.integral = this.minInt;
        } else if (this.maxInt != 0 && this.integral>this.maxInt){
            this.integral = this.maxInt;
        }
        
        logging.info("P:" + this.P*error + ",I:"+this.I*this.integral + ",D:"+this.D*derivitive,"PIDvals");

        this.previousError = error;
        return this.P*error + this.I*this.integral + this.D*derivitive + this.feedforward();
    }

    private double feedforward() {
        return this.FFm * this.setpoint + this.FFb;
    } 
    
    public void resetIntegral() {
        logging.debug("Integral reset prev val: "+this.integral, "shootIntegral");
        this.integral = 0;
        this.iterTime.reset();
        this.iterTime.stop();
        logging.debug("Integral reset after val: "+this.integral, "shootIntegral");
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
