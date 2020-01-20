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
    public static int loggingLevel = 3; //Logging level (-1 = no logging, 0 = errors only, 1 = +warnings, 2 = +debug, 3 = +info)

    public static final int ballIntake = 0; // Ball intake spark, PWM channel 0.

    //Drive Motors
    public static final int 
      leftDrive = 0, // Left drive motor talon, CAN ID 0.
      leftDriveB = 1, // Left drive motor victor, CAN ID 1.
      rightDrive = 2, // Right drive motor talon, CAN ID 2.
      rightDriveB = 3, // Right drive motor victor, CAN ID 3.
      testSparkMax = 4, //test motor
      testSparkMax2 = 5; //test motor 2
  
    // PCM Channels on board 1 (pneumatics)
    public static final int 
      transmissionLow = 0, // Drive transmission low channel, PCM channel 0.
      transmissionHigh = 1; // Drive transmission high channel, PCM channel 1.
}
