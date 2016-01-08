package org.wildstang.yearly.robot;

import org.wildstang.framework.core.Outputs;
import org.wildstang.framework.hardware.OutputConfig;
import org.wildstang.framework.io.outputs.OutputType;
import org.wildstang.hardware.crio.outputs.WSOutputType;
import org.wildstang.hardware.crio.outputs.WsDoubleSolenoidState;
import org.wildstang.hardware.crio.outputs.config.WsDoubleSolenoidConfig;
import org.wildstang.hardware.crio.outputs.config.WsI2COutputConfig;
import org.wildstang.hardware.crio.outputs.config.WsSolenoidConfig;
import org.wildstang.hardware.crio.outputs.config.WsVictorConfig;

import edu.wpi.first.wpilibj.I2C;

public enum WSOutputs implements Outputs
{
   FRONT_LEFT("Front left drive",            WSOutputType.VICTOR,    new WsVictorConfig(0, 0.0), true),
   FRONT_RIGHT("Front right drive",          WSOutputType.VICTOR,    new WsVictorConfig(1, 0.0), true),
   REAR_LEFT("Rear left drive",              WSOutputType.VICTOR,    new WsVictorConfig(2, 0.0), true),
   REAR_RIGHT("Rear right drive",            WSOutputType.VICTOR,    new WsVictorConfig(3, 0.0), true),
   FRONT_LEFT_ROT("Front left rotation",     WSOutputType.VICTOR,    new WsVictorConfig(4, 0.0), true),
   FRONT_RIGHT_ROT("Front right rotation",   WSOutputType.VICTOR,    new WsVictorConfig(5, 0.0), true),
   REAR_LEFT_ROT("Rear left rotation",       WSOutputType.VICTOR,    new WsVictorConfig(6, 0.0), true),
   REAR_RIGHT_ROT("Rear right rotation",     WSOutputType.VICTOR,    new WsVictorConfig(7, 0.0), true),

   LED("LEDs", WSOutputType.I2C, new WsI2COutputConfig(I2C.Port.kOnboard, 0x10), true),

   // Solenoids
   DOUBLE("Double solenoid", WSOutputType.SOLENOID_DOUBLE, new WsDoubleSolenoidConfig(1, 0, 1, WsDoubleSolenoidState.FORWARD), true),
   SINGLE("Single solenoid", WSOutputType.SOLENOID_SINGLE, new WsSolenoidConfig(1, 2, false), true);

   private String m_name;
   private OutputType m_type;
   private OutputConfig m_config;
   private boolean m_trackingState;

   WSOutputs(String p_name, OutputType p_type, OutputConfig p_config, boolean p_trackingState)
   {
      m_name = p_name;
      m_type = p_type;
      m_config = p_config;
      m_trackingState = p_trackingState;
   }
   
   
   @Override
   public String getName()
   {
      return m_name;
   }
   
   @Override
   public OutputType getType()
   {
      return m_type;
   }
   
   public OutputConfig getConfig()
   {
      return m_config;
   }

   public boolean isTrackingState()
   {
      return m_trackingState;
   }

}
