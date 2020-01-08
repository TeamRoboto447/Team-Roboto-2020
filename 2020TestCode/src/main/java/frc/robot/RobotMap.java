package frc.robot;

public class RobotMap {
  public static final int ballIntake = 0; // Ball intake spark, PWM channel 0.

  //Drive Motors
  public static final int 
    leftDrive = 0, // Left drive motor talon, CAN ID 0.
    leftDriveB = 1, // Left drive motor victor, CAN ID 1.
    rightDrive = 2, // Right drive motor talon, CAN ID 2.
    rightDriveB = 3; // Right drive motor victor, CAN ID 3.

  // PCM Channels on board 1 (pneumatics)
  public static final int 
    transmissionLow = 0, // Drive transmission low channel, PCM channel 0.
    transmissionHigh = 1; // Drive transmission high channel, PCM channel 1.
}
