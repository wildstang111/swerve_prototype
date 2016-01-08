package org.wildstang.yearly.robot;

import org.wildstang.framework.core.Inputs;
import org.wildstang.framework.hardware.InputConfig;
import org.wildstang.framework.io.inputs.InputType;
import org.wildstang.hardware.JoystickConstants;
import org.wildstang.hardware.crio.inputs.WSInputType;
import org.wildstang.hardware.crio.inputs.config.WsAbsoluteEncoderConfig;
import org.wildstang.hardware.crio.inputs.config.WsAnalogInputConfig;
import org.wildstang.hardware.crio.inputs.config.WsDigitalInputConfig;
import org.wildstang.hardware.crio.inputs.config.WsI2CInputConfig;
import org.wildstang.hardware.crio.inputs.config.WsJSButtonInputConfig;
import org.wildstang.hardware.crio.inputs.config.WsJSJoystickInputConfig;

import edu.wpi.first.wpilibj.I2C.Port;

public enum SwerveInputs implements Inputs
{

   DRV_HEADING_X("Driver heading X", WSInputType.JS_JOYSTICK, new WsJSJoystickInputConfig(0, JoystickConstants.LEFT_JOYSTICK_X), true),
   DRV_HEADING_Y("Driver heading Y", WSInputType.JS_JOYSTICK, new WsJSJoystickInputConfig(0, JoystickConstants.LEFT_JOYSTICK_Y), true),
   DRV_ROTATION("Driver rotation", WSInputType.JS_JOYSTICK, new WsJSJoystickInputConfig(0, JoystickConstants.RIGHT_JOYSTICK_X), true),
   DRV_RIGHT_Y("Driver right Y", WSInputType.JS_JOYSTICK, new WsJSJoystickInputConfig(0, JoystickConstants.RIGHT_JOYSTICK_Y), true),
//   DRV_DPAD_Y("Driver DPad Y", WSInputType.JS_DPAD, 0, JoystickConstants.DPAD_Y, true),
//   DRV_DPAD_X("Driver DPad X", WSInputType.JS_DPAD, 0, JoystickConstants.DPAD_X, true),
   DRV_BUTTON_1("Driver button 1", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(0, 0), true),
   DRV_BUTTON_2("Driver button 2", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(0, 1), true),
   DRV_BUTTON_3("Driver button 3", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(0, 2), true),
   DRV_BUTTON_4("Driver button 4", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(0, 3), true),
   DRV_BUTTON_5("Driver button 5", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(0, 4), true),
   DRV_BUTTON_6("Driver button 6", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(0, 5), true),
   TURBO("Turbo (Driver 7)", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(0, 6), true),
   ANTI_TURBO("Antiturbo (Driver 8)", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(0, 7), true),
   HOME_BUTTON_1("Wheel home button 1", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(0, 8), true),
   HOME_BUTTON_2("Wheel home button 2", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(0, 9), true),
   DRV_BUTTON_11("Driver button 11", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(0, 10), true),
   DRV_BUTTON_12("Driver button 12", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(0, 11), true),
   
    // Manipulator Enums
//    MAN_RIGHT_JOYSTICK_Y("MANIPULATOR_BACK_ARM_CONTROL", WSInputType.JS_JOYSTICK, new WsJSJoystickInputConfig(1, JoystickConstants.RIGHT_JOYSTICK_Y), true),
//    MAN_RIGHT_JOYSTICK_X("MANIPULATOR_RIGHT_JOYSTICK_X", WSInputType.JS_JOYSTICK, new WsJSJoystickInputConfig(1, JoystickConstants.RIGHT_JOYSTICK_X), true),
//    MAN_LEFT_JOYSTICK_X("MANIPULATOR_LEFT_JOYSTICK_X", WSInputType.JS_JOYSTICK, new WsJSJoystickInputConfig(1, JoystickConstants.LEFT_JOYSTICK_X), true),
//    MAN_DPAD_Y("Manipulator DPad Y", WSInputType.JS_DPAD, 1, JoystickConstants.DPAD_Y, true),
//    MAN_DPAD_X("Manipulator DPad X", WSInputType.JS_DPAD, 1, JoystickConstants.DPAD_X, true),
//    MAN_BUTTON_1("Manipulator button 1", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(1, 0), true),
//    MAN_BUTTON_2("Manipulator button 2", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(1, 1), true),
//    MAN_BUTTON_3("Manipulator button 3", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(1, 2), true),
//    MAN_BUTTON_4("Manipulator button 4", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(1, 3), true),
//    MAN_BUTTON_5("Manipulator button 5", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(1, 4), true),
//    MAN_BUTTON_6("Manipulator button 6", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(1, 5), true),
//    MAN_BUTTON_7("Manipulator button 7", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(1, 6), true),
//    MAN_BUTTON_8("Manipulator button 8", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(1, 7), true),
//    MAN_BUTTON_9("Manipulator button 9", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(1, 8), true),
//    MAN_BUTTON_10("Manipulator button 10", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(1, 9), true),
//    MAN_BUTTON_11("Manipulator button 11", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(1, 10), true),
//    MAN_BUTTON_12("Manipulator button 12", WSInputType.JS_BUTTON, new WsJSButtonInputConfig(1, 11), true),
    
   FRONT_LEFT_HOME("Front Left Home Hall Effect", WSInputType.SWITCH, new WsDigitalInputConfig(0, true), true),
   FRONT_RIGHT_HOME("Front Right Home Hall Effect", WSInputType.SWITCH, new WsDigitalInputConfig(1, true), true),
   REAR_LEFT_HOME("Rear Left Home Hall Effect", WSInputType.SWITCH, new WsDigitalInputConfig(2, true), true),
   REAR_RIGHT_HOME("Rear Right Home Hall Effect", WSInputType.SWITCH, new WsDigitalInputConfig(3, true), true),
   
   FRONT_LEFT_ROT_ENCODER("Front left rotation encoder", WSInputType.ABSOLUTE_ENCODER, new WsAbsoluteEncoderConfig(0, 5), true),
   FRONT_RIGHT_ROT_ENCODER("Front right rotation encoder", WSInputType.ABSOLUTE_ENCODER, new WsAbsoluteEncoderConfig(1, 5), true),
   REAR_LEFT_ROT_ENCODER("Rear left rotation encoder", WSInputType.ABSOLUTE_ENCODER, new WsAbsoluteEncoderConfig(2, 5), true),
   REAR_RIGHT_ROT_ENCODER("Rear right rotation encoder", WSInputType.ABSOLUTE_ENCODER, new WsAbsoluteEncoderConfig(3, 5), true);
   
//   HALL_EFFECT("Lift hall effect sensors", WSInputType.HALL_EFFECT, new WsI2CInputConfig(Port.kMXP, 0x10), true);
//   LIMIT_SWITCH("Limit switch", WSInputType.SWITCH, 0, true),
//   POT("Pot", WSInputType.POT, new WsAnalogInputConfig(0), true);

   private final String m_name;
   private final InputType m_type;

   private InputConfig m_config = null;
   
   private boolean m_trackingState;
   
   SwerveInputs(String p_name, InputType p_type, InputConfig p_config, boolean p_trackingState)
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
   public InputType getType()
   {
      return m_type;
   }

   
   public InputConfig getConfig()
   {
      return m_config;
   }

   public boolean isTrackingState()
   {
      return m_trackingState;
   }


}
