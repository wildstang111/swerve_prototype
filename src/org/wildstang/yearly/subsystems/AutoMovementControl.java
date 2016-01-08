package org.wildstang.yearly.subsystems;

import java.util.logging.Level;
import java.util.logging.Logger;

import org.wildstang.framework.auto.AutoMovement;
import org.wildstang.framework.io.Input;
import org.wildstang.framework.subsystems.Subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Joey
 */
public class AutoMovementControl implements Subsystem
{

   private static Logger s_log = Logger.getLogger(AutoMovementControl.class.getName());
   private AutoMovement runningProgram;
   private AutoMovement programToRun;
   private boolean programInProgress;

   private String m_name;

   public AutoMovementControl()
   {
      m_name = "AutoMovementControl";
   }

   @Override
   public String getName()
   {
      return m_name;
   }

   @Override
   public void init()
   {
      runningProgram = null;
      programToRun = null;
      programInProgress = false;
   }

   @Override
   public void update()
   {
      if (runningProgram != null)
      {
         runningProgram.update();
         if (runningProgram.isFinished())
         {
            cleanUpRunningProgram();
         }
      }
   }

   protected void startProgram()
   {
      runningProgram = programToRun;
      s_log.logp(Level.ALL, "AutoMovementCtrl", "Running Auto Movement", runningProgram.toString());
      runningProgram.initialize();
      SmartDashboard.putString("Auto Movement:", runningProgram.toString());
   }

   public void requestMovement(AutoMovement movement)
   {
      programToRun = movement;
      if (programInProgress && runningProgram != null)
      {
         // terminate current program and start new one.
         cancelProgram();
      }
      startProgram();
      programInProgress = true;
   }

   public void cancelMovement()
   {
      if (runningProgram != null)
      {
         cancelProgram();
      }
   }

   private void cancelProgram()
   {
      s_log.logp(Level.ALL, "AutoMovementCtrl", "Abort Auto Movement", runningProgram.toString());
      runningProgram.abort();
      cleanUpRunningProgram();
   }

   protected void cleanUpRunningProgram()
   {
      runningProgram.cleanup();
      runningProgram = null;
      programInProgress = false;
      SmartDashboard.putString("Auto Movement:", "");
   }

   @Override
   public void inputUpdate(Input source)
   {
   }

   @Override
   public void selfTest()
   {
   }

}
