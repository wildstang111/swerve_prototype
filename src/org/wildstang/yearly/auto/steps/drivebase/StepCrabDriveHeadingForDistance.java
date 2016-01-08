package org.wildstang.yearly.auto.steps.drivebase;

import java.util.Date;

import org.wildstang.framework.auto.steps.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.yearly.robot.SwerveInputs;

/**
 *
 * @author Billy
 */
public class StepCrabDriveHeadingForDistance extends AutoStep
{

   private double leftX, leftY, desiredAngle, distance, heading, throttle;
   private long timeToDrive = 0;
   private long start;
   private boolean firstRun = true;
   private long elapsed = 0;
   
   public StepCrabDriveHeadingForDistance(double throttle, double heading, double distance)
   {
      this.desiredAngle = heading;
      this.distance = distance;
      this.throttle = throttle;
      this.leftX = throttle * Math.cos(heading);
      this.leftY = throttle * Math.sin(heading);
   }

   @Override
   public void initialize()
   {
      timeToDrive = (long)distance;  // Calculate time to drive from the distance
   }

   @Override
   public void update()
   {
      if (firstRun)
      {
         start = System.currentTimeMillis();
         firstRun = false;
      }
//      long now = System.currentTimeMillis();
//      elapsed += now - lastStart;
//      lastStart = now;
//      
//      if (elapsed >= timeToDrive)
//      {
//         setFinished(true);
//      }
//      else
//      {
//      }

      long timeSinceStart = timeSinceStart();
      if (timeSinceStart < 1000)
      {
         ((AnalogInput)Core.getInputManager().getInput(SwerveInputs.DRV_HEADING_X.getName())).setValue(.5);
         ((AnalogInput)Core.getInputManager().getInput(SwerveInputs.DRV_HEADING_Y.getName())).setValue(.5);
      }
      else if (timeSinceStart >= 1000 && timeSinceStart < 2000)
      {
         ((AnalogInput)Core.getInputManager().getInput(SwerveInputs.DRV_HEADING_X.getName())).setValue(-0.5);
         ((AnalogInput)Core.getInputManager().getInput(SwerveInputs.DRV_HEADING_Y.getName())).setValue(.5);
      }
      else if (timeSinceStart >= 2000 && timeSinceStart < 3000)
      {
         ((AnalogInput)Core.getInputManager().getInput(SwerveInputs.DRV_HEADING_X.getName())).setValue(-.5);
         ((AnalogInput)Core.getInputManager().getInput(SwerveInputs.DRV_HEADING_Y.getName())).setValue(-.5);
      }
      else
      {
         ((AnalogInput)Core.getInputManager().getInput(SwerveInputs.DRV_HEADING_X.getName())).setValue(0.0);
         ((AnalogInput)Core.getInputManager().getInput(SwerveInputs.DRV_HEADING_Y.getName())).setValue(0.0);
      }
   }

   private long timeSinceStart()
   {
      return (new Date().getTime()) - start;
   }
   
   @Override
   public String toString()
   {
      return "Driving at " + desiredAngle + " degrees for " + distance + " distance at " + throttle + " speed.";
   }
}
