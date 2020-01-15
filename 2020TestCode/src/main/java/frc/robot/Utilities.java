package frc.robot;

public class Utilities {
    public static double encoderToInch(double target) {
        double encoderRes = 4096;
        double wheelDiameter = 6;
        double circum = wheelDiameter*3.1415926;
        double encodePerInch = encoderRes/circum;
        double output = Math.round(encodePerInch * target);
        
        return output;
    }

    public static void debug(String message) {
        // TODO
    }

    public static void logging(String message, String loggingType) {
        switch(loggingType) {
            case("INFO"):
                if(Constants.loggingLevel >= 3) {
                    System.out.println("INFO: "+message);
                }
                break;
            case("DEBUG"): 
                if(Constants.loggingLevel >= 2) {
                    System.out.println("DEBUG: "+message);
                }
                break;
            case("WARNING"): 
                if(Constants.loggingLevel >= 1) {
                    System.out.println("WARNING: "+message);
                }
                break;
            case("ERROR"): 
                if(Constants.loggingLevel >= 0) {
                    System.out.println("ERROR: "+message);
                }
                break;
            default: System.out.println("You attempted to use logging, but didn't use a valid log type (DEBUG, WARNING, INFO, WAT)");
        }
    }
}