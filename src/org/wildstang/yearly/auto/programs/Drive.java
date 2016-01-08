package org.wildstang.yearly.auto.programs;

import org.wildstang.framework.auto.AutoProgram;
import org.wildstang.framework.auto.steps.AutoSerialStepGroup;
import org.wildstang.framework.core.Core;
import org.wildstang.yearly.auto.steps.drivebase.StepCrabDriveHeadingForDistance;

public class Drive extends AutoProgram
{
   protected final double DISTANCE = Core.getConfigManager().getConfig().getDouble(this.getClass().getName() + ".DistanceToDrive", 140.0);

   @Override
   protected void defineSteps()
   {

      AutoSerialStepGroup drive = new AutoSerialStepGroup("Drive");
      drive.addStep(new StepCrabDriveHeadingForDistance(0.5, 0.5, 2000));

      addStep(drive);
   }

   @Override
   public String toString()
   {
      return "Drive";
   }
}