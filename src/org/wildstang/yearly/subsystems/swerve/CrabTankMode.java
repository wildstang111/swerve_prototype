package org.wildstang.yearly.subsystems.swerve;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.Input;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.crio.outputs.WsVictor;
import org.wildstang.yearly.robot.WSInputs;
import org.wildstang.yearly.robot.WSOutputs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CrabTankMode implements SwerveMode
{
   double leftX;
   double leftY;
   double rightX;
   boolean button9, button10;
   double magnitude;
   double rotMagUR, rotMagUL, rotMagLR, rotMagLL;
   double encodeAngleUR, encodeAngleUL, encodeAngleLR, encodeAngleLL;
   double leftMag;
   double rightMag;
   double desiredAngleUR, desiredAngleUL, desiredAngleLR, desiredAngleLL,
         desiredAngle;
   final double DEADBAND = (Math.PI / 180);
   final double JOYSTICKDEADBAND = .02;
   final double HOMEROTATESPEED = .05;
   final double MAXROTATESPEED = .15;
   boolean isOpposite;
   WsVictor VictorURD, 
            VictorULD, 
            VictorURR, 
            VictorULR, 
            VictorLRD, 
            VictorLLD,
            VictorLRR, 
            VictorLLR;
   boolean HallEffectUR, HallEffectUL, HallEffectLR, HallEffectLL;
   double encoderOffsetUR = 0, encoderOffsetUL = 0, encoderOffsetLR = 0,
         encoderOffsetLL = 0;

   /*
    * Constructor should not take args to insure that it can be instantiated via
    * reflection.
    */
   public CrabTankMode()
   {

   }

//   @Override
//   public void selfTest()
//   {
//      // TODO Auto-generated method stub
//
//   }
//   
   @Override
   public SwerveBaseState calculateNewState(SwerveBaseState p_prevState,
   		double... args) {
	   leftX = args[0];
	   leftY = args[1];
	   rightX = args[2];
	   
	   SwerveBaseState newState = SwerveUtils.createBaseState();

	      WheelModuleState newFrontLeft = newState.getFrontLeft();
	      WheelModuleState newFrontRight = newState.getFrontRight();
	      WheelModuleState newRearLeft = newState.getRearLeft();
	      WheelModuleState newRearRight = newState.getRearRight();
	      
   	// TODO Auto-generated method stub
	      // magnitude - how powerful drive motors are running (-1 to 1)
	      // rotMag - how powerful the rotational motors are running (-1 to 1)

	      // leftX - driver Left Joystick X value (-1 to 1)
	      // leftY - driver Left Joystick Y Value (-1 to 1)

	      // encodeAngle - encoder angle readout (0 to 359.9)
	      // DEADBAND - final double that will be the max disparity between desired
	      // angle and the actual angle of the swerve modules

//	      if (button9 && button10)
//	      {
//	         phoneHome(HOMEROTATESPEED);
//	      }

//	      if (Math.abs(leftX) < JOYSTICKDEADBAND) leftX = 0;
//	      if (Math.abs(leftY) < JOYSTICKDEADBAND) leftY = 0;
//	      if (Math.abs(rightX) < JOYSTICKDEADBAND) rightX = 0;

	      if (leftX == 0 && leftY == 0 && rightX == 0)
	      { // if no controller input
	         magnitude = 0;
	         rotMagUR = 0;
	         rotMagUL = 0;
	         rotMagLR = 0;
	         rotMagLL = 0;
	      }
	      else
	      {
	         desiredAngle = Math.abs(getAngle(leftX, leftY));
	         Core.getStateTracker().addState("Desired angle", "Desired angle", desiredAngle);
//	         Core.getStateTracker().addState("Encoder angle", "Encoder angle", encodeAngleUL);
//	         desiredAngleUR = limitAngle(desiredAngle + encoderOffsetUR);
//	         desiredAngleUL = limitAngle(desiredAngle + encoderOffsetUL);
//	         desiredAngleLR = limitAngle(desiredAngle + encoderOffsetLR);
//	         desiredAngleLL = limitAngle(desiredAngle + encoderOffsetLL);

//	         Core.getStateTracker().addState("Encoder offset", "Encoder offset", encoderOffsetUL);
//	         Core.getStateTracker().addState("Target encoder angle", "Target encoder angle", desiredAngleUL);

	         magnitude = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2)); // here we get the raw magnitude from Pythagorean Theorem

//	         double angleDistance = getAngleDistance(encodeAngleUL, desiredAngleUL);
//	         Core.getStateTracker().addState("angleDistance", "angleDistance", angleDistance);
//	         if (angleDistance > Math.PI / 2)
//	         {
//	            isOpposite = true;
//	            magnitude *= -1;
//	         }
//	         else
//	         {
//	            isOpposite = false;
//	         }
//	         rotMagUR = getRotMag(encodeAngleUR, desiredAngleUR);
//	         rotMagUL = getRotMag(encodeAngleUL, desiredAngleUL);
//	         rotMagLR = getRotMag(encodeAngleLR, desiredAngleLR);
//	         rotMagLL = getRotMag(encodeAngleLL, desiredAngleLL);
	         
//	         rotMagUR = newGetRotMag(encodeAngleUR, desiredAngleUR);
//	         rotMagUL = newGetRotMag(encodeAngleUL, desiredAngleUL);
//	         rotMagLR = newGetRotMag(encodeAngleLR, desiredAngleLR);
//	         rotMagLL = newGetRotMag(encodeAngleLL, desiredAngleLL);

	         Core.getStateTracker().addState("Magnitude", "Magnitude", magnitude);
//	         Core.getStateTracker().addState("Rotation magnitude", "Rotation magnitude", rotMagUL);

	         if (Math.abs(magnitude) < .25)
	         {
	            boolean isPositive = true;
	            if (magnitude < 0)
	            {
	               isPositive = false;
	            }
	            magnitude = Math.pow(magnitude, 2) * 4;
	            if (!isPositive)
	            {
	               magnitude *= -1;
	            }
	         }

	         leftMag = adjustMagnitude(magnitude, -rightX, true);
	         rightMag = adjustMagnitude(magnitude, -rightX, false);
	      }
	      int setAngle =  (int) (desiredAngle*(180/Math.PI));
	      //if one drive side is less than the deadband, set the desired angle to the last value
	      if(Math.abs(leftMag)<JOYSTICKDEADBAND)
	    	  {
	    	  setAngle = p_prevState.getFrontLeft().getRotationAngle();
	    	  leftMag = 0;
	    	  rightMag = 0;
	    	  }
	      // assign motor values
	      newFrontRight.setSpeed(rightMag);
	      newFrontLeft.setSpeed(leftMag);
	      newFrontRight.setRotationAngle(setAngle);
	      newFrontLeft.setRotationAngle(setAngle);
//	      VictorURR.setValue(rotMagUR);
//	      VictorULR.setValue(rotMagUL);
	      newRearRight.setSpeed(rightMag);
	      newRearLeft.setSpeed(leftMag);
	      newRearRight.setRotationAngle(setAngle);
	      newRearLeft.setRotationAngle(setAngle);
//	      VictorLRR.setValue(rotMagLR);
//	      VictorLLR.setValue(rotMagLL);

	      // print out important values to dashboard
	      SmartDashboard.putNumber("Magnitude", magnitude);
	      SmartDashboard.putNumber("Left Mag", leftMag);
	      SmartDashboard.putNumber("Right Mag", rightMag);
	      SmartDashboard.putNumber("Desired angle", desiredAngle);
	      SmartDashboard.putNumber("Left X", leftX);
	      SmartDashboard.putNumber("Left Y", leftY);
//	      SmartDashboard.putNumber("encoder", encodeAngleUR);
//	      SmartDashboard.putNumber("rotMagUR", rotMagUR);
   	return newState;
   }

   // secondary method
   // Accounts for rotation in drive motors
   private double adjustMagnitude(double original, double rotation,
         boolean isLeft)
   {
      if (isLeft)
      {
         return limitMotor(original + (Math.abs(original) * rotation));
      }
      else
      {
         return limitMotor(original - (Math.abs(original) * rotation));
      }
   }

   // primary method, calls no others
   // limits motor output to 1
   private double limitMotor(double magnitude)
   {
      if (magnitude > 1)
      {
         return 1d;
      }
      else if (magnitude < -1)
      {
         return -1d;
      }
      else
      {
         return magnitude;
      }
   }

   // big method (2 primary (limitAngle/getAngleDistance), 1
   // secondary(getAbsAngleDistance))
   // sets mag and direction of rotation motor
   private double getRotMag(double actual, double desired)
   {
      double rotateMag;
      double oppositeDesired = limitAngle(Math.PI + desired);
      double angleDistance = getAngleDistance(desired, actual);
      if (isOpposite)
      {
         if (getAbsAngleDistance(oppositeDesired, actual) < 0)
         {
            rotateMag = -1d / 2;
         }
         else
         {
            rotateMag = 1d / 2;
         }
      }
      else
      {
         if (getAbsAngleDistance(desired, actual) < 0)
         {
            rotateMag = -1d / 2;
         }
         else
         {
            rotateMag = 1d / 2;
         }
      }

      rotateMag /= 2;

      if (Math.abs(angleDistance) < (Math.PI / 9))
      {
         // Deadband = 1 degree
         rotateMag *= (angleDistance / (Math.PI / 9));
         if (angleDistance < DEADBAND)
         {
            return 0;
         }
      }
      return rotateMag;
   }
   
   private double newGetRotMag(double actual, double desired)
   {
	   double distance = getAbsAngleDistance(desired, actual);
	   double rotateMag = MAXROTATESPEED;
	   
	   if(Math.abs(distance) < DEADBAND) return 0;
	   
	   if(distance < 0)
	   {
		   rotateMag *= -1;
	   }
	   
	   if(Math.abs(distance) < (Math.PI/9))
	   {
		   rotateMag *= (Math.abs(distance) / (Math.PI/9));
	   }
	   
	   return rotateMag;
	   
   }

   // secondary, 1 primary call
   // Finds angle of driver joystick
   private static double getAngle(double x, double y)
   {
      double angle = 0;
      if (y > 0)
      {
         angle = limitAngle(Math.atan(x / y));
      }
      else if (x >= 0 && y < 0)
      {
         angle = Math.PI - limitAngle(Math.atan(x / y));
      }
      else if (x <= 0 && y < 0)
      {
         angle = Math.PI + limitAngle(Math.atan(x / y));
      }
      else if (y == 0 && x >= 0)
      {
         angle = (Math.PI / 2);
      }
      else
      {
         angle = (Math.PI * 1.5);
      }

      return angle;
   }

   // primary
   // runs angle measures over (negative value goes to 359, above 360 goes to 1)
   private static double limitAngle(double oldAngle)
   {
      double newAngle = oldAngle;
      while (newAngle >= (2 * Math.PI))
      {
         newAngle -= (2 * Math.PI);
      }
      while (newAngle < 0)
      {
         newAngle += (2 * Math.PI);
      }

      return newAngle;
   }

   // secondary, 1 primary call
   // returns positive or negative angle distance
   private static double getAbsAngleDistance(double finalAngle, double initialAngle) {
      //returns the shortest raw angle between the two angles
      double diff = getAngleDistance(finalAngle, initialAngle);
      //ORIGINAL
      //if the final angle is less than 270 and initial is greater, then distance is negative
      //final = 5, initial = 355 FAIL
//    if (finalAngle < (Math.PI*1.5)) {
//       if (initialAngle > finalAngle) {
//       diff *= -1;
//     }
//    } 
      
	  //NEW
	  //if final is less than 180 and initial is less than 180 greater than final (AND GREATER THAN FINAL)
	  //then distance is negative
	  if (finalAngle <= (Math.PI)) {
		  if (initialAngle > finalAngle && initialAngle < finalAngle + Math.PI) {
		  diff *= -1;
	  	}
	  } 

      else {
       // this applies to an angle greater than 270 only
         //THIS LOGIC WORKS FOR ALL GREATER THAN 180, and was originally correct
      double oppositeFinal = finalAngle - (Math.PI);
      //if the initial is greater than final(between final and 360)
      //or between 0 and the opposite angle, the shortest distance between 
      //the final and initial is negative
      if (initialAngle > finalAngle || initialAngle < oppositeFinal) {
      diff *= -1;
      }
      }

      return diff;
       }
   // primary
   // returns magnitude of angle distance
   private static double getAngleDistance(double angle1, double angle2)
   {
      double diff = Math.abs(angle1 - angle2);
      if (diff > Math.PI)
      {
         diff = (Math.PI * 2) - diff;
      }
      return diff;
   }

   // primary
   // runs wheels until home position found
   private void setHomePosition(double rotateSpeed)
   {
      while (!HallEffectUR)
      {
         VictorURR.setValue(rotateSpeed);
      }
      while (!HallEffectUL)
      {
         VictorULR.setValue(rotateSpeed);
      }
      while (!HallEffectLR)
      {
         VictorLRR.setValue(rotateSpeed);
      }
      while (!HallEffectLL)
      {
         VictorLLR.setValue(rotateSpeed);
      }
   }

   // secondary, 1 primary call
   // sets home position & encoder offset
   private void phoneHome(double rotateSpeed)
   {
      setHomePosition(rotateSpeed);
      encoderOffsetUR = encodeAngleUR;
      encoderOffsetUL = encodeAngleUL;
      encoderOffsetLR = encodeAngleLR;
      encoderOffsetLL = encodeAngleLL;
   }


}
