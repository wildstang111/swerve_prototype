package org.wildstang.yearly.subsystems.swerve;

public class WheelModuleState
{
   private int m_rotationAngle;
   private double m_speed;
   
   public WheelModuleState()
   {
   }

   public int getRotationAngle()
   {
      return m_rotationAngle;
   }

   public void setRotationAngle(int p_rotationAngle)
   {
      m_rotationAngle = p_rotationAngle;
   }

   public double getSpeed()
   {
      return m_speed;
   }

   public void setSpeed(double p_speed)
   {
      m_speed = p_speed;
   }
   
   
}
