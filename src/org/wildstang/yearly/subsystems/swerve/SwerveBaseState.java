package org.wildstang.yearly.subsystems.swerve;

public class SwerveBaseState
{
   private WheelModuleState[] m_modules = new WheelModuleState[4];

   public static final int FRONT_LEFT = 0;
   public static final int FRONT_RIGHT = 1;
   public static final int REAR_LEFT = 2;
   public static final int REAR_RIGHT = 3;
   
   public SwerveBaseState(WheelModuleState newFrontLeft, WheelModuleState newFrontRight, WheelModuleState newRearLeft, WheelModuleState newRearRight)
   {
      m_modules[FRONT_LEFT] = newFrontLeft;
      m_modules[FRONT_RIGHT] = newFrontRight;
      m_modules[REAR_LEFT] = newRearLeft;
      m_modules[REAR_RIGHT] = newRearRight;
   }

   public WheelModuleState getFrontLeft()
   {
      return m_modules[FRONT_LEFT];
   }

   public WheelModuleState setFrontLeft(WheelModuleState state)
   {
      return m_modules[FRONT_LEFT] = state;
   }

   public WheelModuleState getFrontRight()
   {
      return m_modules[FRONT_RIGHT];
   }

   public WheelModuleState setFrontRight(WheelModuleState state)
   {
      return m_modules[FRONT_RIGHT] = state;
   }

   public WheelModuleState getRearLeft()
   {
      return m_modules[REAR_LEFT];
   }

   public WheelModuleState setRearLeft(WheelModuleState state)
   {
      return m_modules[REAR_LEFT] = state;
   }

   public WheelModuleState getRearRight()
   {
      return m_modules[REAR_RIGHT];
   }
   
   public WheelModuleState setRearRight(WheelModuleState state)
   {
      return m_modules[REAR_RIGHT] = state;
   }

   
}
