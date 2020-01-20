/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;;
/**
 * Add your docs here.
 */
public class logging {
    public static void errors(String message, String name){
        if (Constants.loggingLevel >= 0){
            adenLogging(message, name);
        }
    }
    public static void warnings(String message, String name){
        if (Constants.loggingLevel >= 1){
            adenLogging(message, name);
        }
    }
    public static void debug(String message, String name) {
        if (Constants.loggingLevel >= 2){
            adenLogging(message, name);
        }
    }
    public static void info(String message, String name){
        if (Constants.loggingLevel >= 3){
            adenLogging(message, name);
        }
    }
    private static void adenLogging(String message, String name){
        NetworkTableEntry subsystemToDebug = NetworkTableInstance.getDefault().getTable("adenLogging").getEntry("subsysToLog");
        if (subsystemToDebug.getString("errored").contains("everything;")){
            System.out.println(message);
        } else if (subsystemToDebug.getString("errored").contains(name+";")){
            System.out.println(message);
        }
    }
}
