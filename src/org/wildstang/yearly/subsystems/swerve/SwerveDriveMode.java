package org.wildstang.yearly.subsystems.swerve;

public class SwerveDriveMode implements SwerveMode
{
   int width = 24;
   int length = 30;
   double R = Math.sqrt((width * width) + (length * length));
   double LRscaleFactor = length / R;
   double WRscaleFactor = width / R;
   double RADIANS = 180 / Math.PI;

   @Override
   public SwerveBaseState calculateNewState(SwerveBaseState p_prevState, double... args)
   {
      
      double strafe = args[0];
      double fwd = args[1];
      double rotation = args[2];
      
      SwerveBaseState newState = SwerveUtils.createBaseState();

      WheelModuleState newFrontLeft = newState.getFrontLeft();
      WheelModuleState newFrontRight = newState.getFrontRight();
      WheelModuleState newRearLeft = newState.getRearLeft();
      WheelModuleState newRearRight = newState.getRearRight();

      double A = strafe - rotation * LRscaleFactor;
      double B = strafe + rotation * LRscaleFactor;
      double C = fwd - rotation * WRscaleFactor;
      double D = fwd + rotation * WRscaleFactor;
      
      double flSpeed = Math.sqrt((B * B) + (D * D));
      double frSpeed = Math.sqrt((B * B) + (C * C));
      double rlSpeed = Math.sqrt((A * A) + (D * D));
      double rrSpeed = Math.sqrt((A * A) + (C * C));
      
      double maxSpeed = Math.max(flSpeed, frSpeed);
      maxSpeed = Math.max(maxSpeed, rlSpeed);
      maxSpeed = Math.max(maxSpeed, rrSpeed);
      
      if (maxSpeed > 1)
      {
         flSpeed = flSpeed / maxSpeed;
         frSpeed = frSpeed / maxSpeed;
         rlSpeed = rlSpeed / maxSpeed;
         rrSpeed = rrSpeed / maxSpeed;
      }
      
      int frontLeftAngle;
      int frontRightAngle;
      int rearLeftAngle;
      int rearRightAngle;
      
      // Front left angle
      if (D == 0 && B == 0)
      {
         frontLeftAngle = 0;
      }
      else
      {
         frontLeftAngle = (int)(Math.atan2(D, B) * RADIANS);
         // ensure it's positive
         frontLeftAngle = (frontLeftAngle + 360) % 360;
      }
      
      // Front right angle
      if (C == 0 && B == 0)
      {
         frontRightAngle = 0;
      }
      else
      {
         frontRightAngle = (int)(Math.atan2(C, B) * RADIANS);
         // ensure it's positive
         frontRightAngle = (frontRightAngle + 360) % 360;
      }
      
      // Rear left angle
      if (D == 0 && A == 0)
      {
         rearLeftAngle = 0;
      }
      else
      {
         rearLeftAngle = (int)(Math.atan2(D, A) * RADIANS);
         // ensure it's positive
         rearLeftAngle = (rearLeftAngle + 360) % 360;
      }
      
      // Rear right angle
      if (C == 0 && A == 0)
      {
         rearRightAngle = 0;
      }
      else
      {
         rearRightAngle = (int)(Math.atan2(C, A) * RADIANS);
         // ensure it's positive
         rearRightAngle = (rearRightAngle + 360) % 360;
      }
      
      newFrontLeft.setRotationAngle(frontLeftAngle);
      newFrontRight.setRotationAngle(frontRightAngle);
      newRearLeft.setRotationAngle(rearLeftAngle);
      newRearRight.setRotationAngle(rearRightAngle);

      newFrontLeft.setSpeed(flSpeed);
      newFrontRight.setSpeed(frSpeed);
      newRearLeft.setSpeed(rlSpeed);
      newRearRight.setSpeed(rrSpeed);
      
      
      return newState;
   }

}
