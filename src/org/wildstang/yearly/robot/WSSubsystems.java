package org.wildstang.yearly.robot;

import org.wildstang.framework.core.Subsystems;
import org.wildstang.yearly.subsystems.DriveBase;
import org.wildstang.yearly.subsystems.HardwareTest;
import org.wildstang.yearly.subsystems.LED;
import org.wildstang.yearly.subsystems.Monitor;
import org.wildstang.yearly.subsystems.SwerveDrive;

public enum WSSubsystems implements Subsystems
{
//   DRIVE_BASE("Drive base", DriveBase.class),
   SWERVE_BASE("Swerve base", SwerveDrive.class),
//   HARDWARE_TEST("Hardware test", HardwareTest.class),
   LED("LED", LED.class),
   MONITOR("Monitor", Monitor.class);

   private String m_name;
   private Class m_class;

   WSSubsystems(String p_name, Class p_class)
   {
      m_name = p_name;
      m_class = p_class;
   }

   @Override
   public String getName()
   {
      return m_name;
   }

   @Override
   public Class getSubsystemClass()
   {
      return m_class;
   }


}
