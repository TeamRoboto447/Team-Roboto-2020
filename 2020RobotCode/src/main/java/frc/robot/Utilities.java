package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Utilities {
    static NetworkTableEntry subsystemToDebug;

    public static double adjustForDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        }
        return value;
    }

    public static double feetToEncoder(double target) {
        return inchToEncoder(target * 12);
    }

    public static boolean marginOfError(double maxError, double setpoint, double processingVar) {
        double error = Math.abs(setpoint - processingVar);
        return error <= maxError;
    }

    public static double inchToEncoder(double target) {
        double circum = Constants.wheelDiameter * Math.PI;
        double encodePerInch = Constants.encoderRes / circum;
        double output = Math.round(encodePerInch * target);

        return output;
    }

    public static double encoderToInch(double encoder) {
        double circum = Constants.wheelDiameter * Math.PI;
        double encodePerInch = Constants.encoderRes / circum;
        double output = encoder / encodePerInch;
        return output;
    }

    public static double meterToEncoder(double target) {
        double circum = Constants.wheelDiameterMeters * Math.PI;
        double encodePerMeter = Constants.encoderRes / circum;
        double output = Math.round(encodePerMeter * target);
        return output;
    }

    public static double encoderToMeter(double encoder) {
        double circum = Constants.wheelDiameterMeters * Math.PI;
        double encodePerMeter = Constants.encoderRes / circum;
        double output = encoder / encodePerMeter;
        return output;
    }

    public static double intputToOutput(double distance, String gear) {
        switch(gear.toLowerCase()) {
            case "low":
                distance *= Constants.lowGearRatio*Constants.thirdStageRatio;
                break;
            case "high":
                distance *= Constants.highGearRatio*Constants.thirdStageRatio;
                break;
            default:
                distance *= Constants.lowGearRatio*Constants.thirdStageRatio;
                break;
        }

        return distance;
    }
}