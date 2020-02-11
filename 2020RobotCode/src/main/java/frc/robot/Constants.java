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
    public static int loggingLevel = 4; // Logging level (-1 = no logging, 0 = errors only, 1 = +warnings, 2 = +debug, 3 = +info)

    public static final boolean bypassShooterPID = false;

    public static final int ballIntake = 0; // Ball intake spark, PWM channel 0.

    //Drive Motors
    public static final int 
      leftDrive = 3, // Left Drive Spark Max, CAN ID 1
      leftDriveB = 4, // Left Drive Spark Max, CAN ID 2.
      rightDrive = 1, // Right Drive Spark Max, CAN ID 3.
      rightDriveB = 2, // Right Drive Spark Max, CAN ID 4.
      turretSparkMax = 5, // Turret Rotation Spark Max, CAN ID 5.
      shooterSparkMaxLeft = 6, // Shooter Left Spark Max, CAN ID 6.
      shooterSparkMaxRight = 7; // Shooter Right Spark Max, CAN ID 7.

    public static final double
      // Gear ratios for calculating encoder ticks per rotation
      lowGearRatio = 25.9,
      highGearRatio = 8.63,
      thirdStageRatio = 1.41,
      // Wheel diameter for calculating inches per rotation
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
    
    // Shooter PID info
    public static final double
      shooterkP = 0.0003,//0.0003, //original vals are commented
      shooterkI = 0.0013815,//0.000931,
      shooterkD = 0.000016,//0.000024,
      shooterkFFm = 0.0001774, //0.000172,
      shooterkFFb = -0.003214,//-0.002191,
      shooterIZone = 100,
      speedkM = 0.0135274,
      speedkB = 0.6368884,
      shooterPidIntegralResetTime = 2,
      distanceFromInnerToOuterPort = 2,
      maxInnerPortAjustmentAngle = Math.PI / 4;

    // Turning PID info
    public static final double
      turretkP = 0.01,
      turretkI = 0.009,
      turretkD = 0.001,
      turretFFm = 0,
      turretFFb = 0,
      turretIZone = 10,
      turretSpinLimit = 190;

    public static final boolean Secret = false;
}
