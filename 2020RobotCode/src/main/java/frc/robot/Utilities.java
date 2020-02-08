package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Utilities {
    static NetworkTableEntry subsystemToDebug;

    public static double feetToEncoder(double target){
        return inchToEncoder(target * 12);
    }
    public static boolean marginOfError(double maxError, double setpoint, double processingVar){
        double error = Math.abs(setpoint - processingVar);
        return error <= maxError;
    }

    public static double inchToEncoder(double target) {
        double encoderRes = 4096;
        double wheelDiameter = 6;
        double circum = wheelDiameter * Math.PI;
        double encodePerInch = encoderRes/circum;
        double output = Math.round(encodePerInch * target);
        
        return output;
    }

    public static void adenLogging(String message, String name){
        
        if (subsystemToDebug.getString("errored").contains("everything;")){
            System.out.println(message);
        } else if (subsystemToDebug.getString("errored").contains(name+";")){
            System.out.println(message);
        }
    }

    public static void init() {
        subsystemToDebug = NetworkTableInstance.getDefault().getTable("adenLogging").getEntry("subsysToLog");
    }

}
