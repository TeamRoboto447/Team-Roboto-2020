/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int loggingLevel = 4; //Logging level (-1 = no logging, 0 = errors only, 1 = +warnings, 2 = +debug, 3 = +info)

    public static final boolean bypassShooterPID = false;

    public static final int ballIntake = 0; // Ball intake spark, PWM channel 0.

    //Drive Motors
    public static final int 
      leftDrive = 1, // Left drive motor talon, CAN ID 0.
      leftDriveB = 2, // Left drive motor victor, CAN ID 1.
      rightDrive = 3, // Right drive motor talon, CAN ID 2.
      rightDriveB = 4; // Right drive motor victor, CAN ID 3.
      //shooterSparkMax = 4, //test motor
      //shooterSparkMax2 = 5; //test motor 2
    public static final double
      lowGearRatio = 25.9,
      highGearRatio = 8.63,
      thirdStageRatio = 1.41,
      wheelDiameter = 6.0;
  
    // PCM Channels on board 1 (pneumatics)
    public static final int 
      transmissionLow = 0, // Drive transmission low channel, PCM channel 0.
      transmissionHigh = 1; // Drive transmission high channel, PCM channel 1.
    
    public static final double
      drivekP = 0.1,
      drivekI = 0, //1e-4
      drivekD = 1,
      drivekIz = 0,
      drivekFF = 0,
      drivePIDMin = -0.5,
      drivePIDMax = 0.5; 
    
    public static final double
      shooterkP = 0.0003,//0.0003, //original vals are commented
      shooterkI = 0.0013815,//0.000931,
      shooterkD = 0.000016,//0.000024,
      shooterkFFm = 0.0001774, //0.000172,
      shooterkFFb = -0.003214,//-0.002191,
      speedkM = 0.0135274,
      speedkB = 0.6368884,
      pidIntegralResetTime = 2,
      distanceFromInnerToOuterPort = 2,
      maxInnerPortAjustmentAngle = Math.PI / 4;
    public static final double //values for turning PID
      turnkP = 0.01986, // 0.0207, //we had 2 sets of values, I (Aden) put the ones in I thought were write
      turnkI = 0.070042, //0.0414,
      turnkD = 0.001408, // 0.002588,
      turnkFFm = 0.0,
      turnkFFb = 0.0,
      turnTargetAjust = 0.9;
    public static final double
      turnToBallP = 0.003,
      turnToBallI = 0.0005,
      turnToBallD = 0.0,
      turnToBallFFm = 0.0,
      turnToBallFFb = 0.0,
      turnToBallThreshold = 25,
      turnToBallBaseSpeed = 0.7;
    public static final boolean Secret = true;
}
