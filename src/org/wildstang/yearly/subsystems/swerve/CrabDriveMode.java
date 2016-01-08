package org.wildstang.yearly.subsystems.swerve;

public class CrabDriveMode implements SwerveMode
{
   private final double c = -0.0802884041;
   private final double b = 60.81576;
   private final double a = -15.574181;

   private static final double SPEED_LIMIT = 1.0;
   private static final double DEADBAND_SPEED = 0.02;
   
   @Override
   public SwerveBaseState calculateNewState(SwerveBaseState p_prevState, double... args)
   {
      
      double headingX = args[0];
      double headingY = args[1];
      double rotation = args[2];
      
      SwerveBaseState newState = SwerveUtils.createBaseState();

      WheelModuleState newFrontLeft = newState.getFrontLeft();
      WheelModuleState newFrontRight = newState.getFrontRight();
      WheelModuleState newRearLeft = newState.getRearLeft();
      WheelModuleState newRearRight = newState.getRearRight();

      // Calculate the speed based on joystick position
      double motorSpeed = calculateSpeed(headingX, headingY);

      // Deadband check
      if (motorSpeed < DEADBAND_SPEED && motorSpeed > -DEADBAND_SPEED)
      {
         // If the calculated new speed is very low, leave the wheels at their current angle, and set the 
         // motor output to 0
         newFrontLeft.setRotationAngle(p_prevState.getFrontLeft().getRotationAngle());
         newFrontRight.setRotationAngle(p_prevState.getFrontRight().getRotationAngle());
         newRearLeft.setRotationAngle(p_prevState.getRearLeft().getRotationAngle());
         newRearRight.setRotationAngle(p_prevState.getRearRight().getRotationAngle());
 
         newFrontLeft.setSpeed(0.0);
         newFrontRight.setSpeed(0.0);
         newRearLeft.setSpeed(0.0);
         newRearRight.setSpeed(0.0);
      }
      else
      {
         // Crab drive is field oriented and does not allow the robot to rotate
         int currentHeadingAngle = (int) cartesianToDegrees(headingX, headingY);
   
         newFrontLeft.setRotationAngle(currentHeadingAngle);
         newFrontRight.setRotationAngle(currentHeadingAngle);
         newRearLeft.setRotationAngle(currentHeadingAngle);
         newRearRight.setRotationAngle(currentHeadingAngle);
   
         double leftMotorSpeed = motorSpeed;
         double rightMotorSpeed = motorSpeed;
         // Allow rotation while crabbing
         if (rotation < -DEADBAND_SPEED)
         {
            // Turning left
            leftMotorSpeed = leftMotorSpeed + (2 * rotation * leftMotorSpeed);
         }
         else if (rotation > DEADBAND_SPEED)
         {
            // Turning right
            rightMotorSpeed = rightMotorSpeed - (2 * rotation * rightMotorSpeed);
         }
   
         newFrontLeft.setSpeed(limitSpeed(leftMotorSpeed));
         newFrontRight.setSpeed(limitSpeed(rightMotorSpeed));
         newRearLeft.setSpeed(limitSpeed(leftMotorSpeed));
         newRearRight.setSpeed(limitSpeed(rightMotorSpeed));
      }

      return newState;
   }

   
   private double calculateSpeed(double headingX, double headingY)
   {
      double speed;
      
      speed = Math.sqrt((headingX * headingX) + (headingY * headingY));
      
      return speed;
   }


   private double limitSpeed(double speed)
   {
      double result = speed;
      
      if (speed < -SPEED_LIMIT)
      {
         result = -SPEED_LIMIT;
      }
      if (speed > SPEED_LIMIT)
      {
         result = SPEED_LIMIT;
      }
      
      return result;
   }
   
   double cartesianToDegrees(double x, double y)
   {
      double result = 0.0;

      if (x >= 0)
      {
         if (y >= 0)
         {
            if (y > x)
            {
               result = f(x / y);
            }
            else if (x == 0)
            {
               result = 0;
            }
            else
            {
               result = 90 - f(y / x);
            }
         }
         else if (-y <= x)
         {
            result = 90 + f(-y / x);
         }
         else
         {
            result = 180 - f(-x / y);
         }
      }
      else if (y <= 0)
      {
         if (y <= x)
         {
            result = 180 + f(x / y);
         }
         else
         {
            result = 270 - f(y / x);
         }
      }
      else if (y <= -x)
      {
         result = 270 + f(-y / x);
      }

      else
      {
         result = 360 - f(-x / y);
      }

      return result;
   }

   double f(double t)
   {
      return t * (a * t + b) + c;
   }

}
