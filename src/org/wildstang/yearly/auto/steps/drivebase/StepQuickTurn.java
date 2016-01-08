/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.wildstang.yearly.auto.steps.drivebase;

import org.wildstang.framework.auto.steps.AutoStep;
import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.InputManager;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.yearly.robot.WSInputs;
import org.wildstang.yearly.robot.WSSubsystems;
import org.wildstang.yearly.subsystems.DriveBase;

/**
 *
 * @author Joey
 */
public class StepQuickTurn extends AutoStep
{

   private double value, angle;
   private boolean shouldFinish = false;

   public StepQuickTurn(double relativeAngle)
   {
      this.value = relativeAngle;
   }

   @Override
   public void initialize()
   {

//      angle = ((DriveBase) Core.getSubsystemManager().getSubsystem(WSSubsystems.DRIVE_BASE.getName())).getGyroAngle()
//            + value;
//      ((DriveBase) Core.getSubsystemManager().getSubsystem(WSSubsystems.DRIVE_BASE.getName())).setThrottleValue(0);
//      ((DriveBase) Core.getSubsystemManager().getSubsystem(WSSubsystems.DRIVE_BASE.getName())).overrideHeadingValue(value < 0 ? 0.6
//            : -0.6);
// TODO
//      ((AnalogInput)Core.getInputManager().getInput(WSInputs.DRV_THROTTLE)).setValue(0.0);
//      ((AnalogInput)Core.getInputManager().getInput(WSInputs.DRV_HEADING)).setValue(value < 0 ? 0.6 : -0.6);

   }

   @Override
   public void update()
   {
      if (shouldFinish)
      {
         setFinished(true);
//         ((DriveBase) Core.getSubsystemManager().getSubsystem(WSSubsystems.DRIVE_BASE.getName())).overrideHeadingValue(0.0);
         return;
      }
//      double gyroAngle = ((DriveBase) Core.getSubsystemManager().getSubsystem(WSSubsystems.DRIVE_BASE.getName())).getGyroAngle();
//      if (value < 0)
//      {
//         if (angle > gyroAngle)
//         {
//            // TODO
////            ((AnalogInput)Core.getInputManager().getInput(WSInputs.DRV_THROTTLE)).setValue(0.0);
////            ((AnalogInput)Core.getInputManager().getInput(WSInputs.DRV_HEADING)).setValue(value < 0 ? 0.6 : -0.6);
//            shouldFinish = true;
//         }
//      }
      else
      {
//         if (angle < gyroAngle)
//         {
//            ((DriveBase) Core.getSubsystemManager().getSubsystem(WSSubsystems.DRIVE_BASE.getName())).overrideHeadingValue(0.0);
//
//            // TODO: What would the equivalent be to set an input value for a
//            // subsystem to react to, rather than set the output directly??
//            // InputManager.getInstance().getOiInput(Robot.DRIVER_JOYSTICK).set(JoystickAxisEnum.DRIVER_HEADING,
//            // new Double(0.0));
//            shouldFinish = true;
//         }
      }
   }

   @Override
   public String toString()
   {
      return "Turning using quickturn with a relative angle";
   }
}
