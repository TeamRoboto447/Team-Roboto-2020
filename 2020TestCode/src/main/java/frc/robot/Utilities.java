package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Utilities {
    static NetworkTableEntry subsystemToDebug;

    public static double encoderToInch(double target) {
        double encoderRes = 4096;
        double wheelDiameter = 6;
        double circum = wheelDiameter*3.1415926;
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