/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.wildstang.yearly.auto.steps.drivebase;

import org.wildstang.framework.auto.steps.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.yearly.robot.WSSubsystems;
import org.wildstang.yearly.subsystems.DriveBase;

/**
 *
 * @author Nathan
 */
public class StepWaitForDriveMotionProfile extends AutoStep
{

   public StepWaitForDriveMotionProfile()
   {
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void update()
   {
//      double distanceRemaining = ((DriveBase) Core.getSubsystemManager().getSubsystem(WSSubsystems.DRIVE_BASE.getName())).getDistanceRemaining();
//      double velocity = ((DriveBase) Core.getSubsystemManager().getSubsystem(WSSubsystems.DRIVE_BASE.getName())).getVelocity();
//      if ((distanceRemaining < 0.01) && (distanceRemaining > -0.01))
//      {
//         finished = true;
//      }
//      if ((distanceRemaining < 12.0) && (distanceRemaining > -12.0)
//            && (velocity < 0.10) && (velocity > -0.10))
//      {
//         finished = true;
//
//      }
   }

   @Override
   public String toString()
   {
      return "Wait for the motion profile to finish moving to target";
   }
}
