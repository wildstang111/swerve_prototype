package org.wildstang.yearly.subsystems.swerve;

public class SwerveUtils
{

   public static SwerveBaseState createBaseState()
   {
      WheelModuleState newFrontLeft = new WheelModuleState();
      WheelModuleState newFrontRight = new WheelModuleState();
      WheelModuleState newRearLeft = new WheelModuleState();
      WheelModuleState newRearRight = new WheelModuleState();

      return new SwerveBaseState(newFrontLeft, newFrontRight, newRearLeft, newRearRight);
   }
}
