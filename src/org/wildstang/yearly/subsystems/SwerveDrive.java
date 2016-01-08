package org.wildstang.yearly.subsystems;

import java.text.DecimalFormat;
import java.util.logging.Logger;

import org.wildstang.framework.core.Core;
import org.wildstang.framework.io.Input;
import org.wildstang.framework.io.inputs.AnalogInput;
import org.wildstang.framework.io.inputs.DigitalInput;
import org.wildstang.framework.subsystems.Subsystem;
import org.wildstang.hardware.crio.outputs.WsVictor;
import org.wildstang.yearly.robot.SwerveInputs;
import org.wildstang.yearly.robot.WSOutputs;
import org.wildstang.yearly.robot.WSSubsystems;
import org.wildstang.yearly.subsystems.swerve.CrabDriveMode;
import org.wildstang.yearly.subsystems.swerve.SwerveBaseState;
import org.wildstang.yearly.subsystems.swerve.SwerveDriveMode;
import org.wildstang.yearly.subsystems.swerve.WheelModuleState;

public class SwerveDrive implements Subsystem
{
   private static final int CRAB = 0;
   private static final int SWERVE = 1;

   boolean firstTime = true;
   
   private double m_joystickRotation = 0.0;
   private double m_headingX = 0.0;
   private double m_headingY = 0.0;

   private double m_flEncoder = 0.0;
   private double m_frEncoder = 0.0;
   private double m_rlEncoder = 0.0;
   private double m_rrEncoder = 0.0;
   

   private int m_mode = CRAB;
   private boolean m_recalcMode = false;

   // Drive motor outputs
   private WsVictor m_frontLeftDrive = null;
   private WsVictor m_frontRightDrive = null;
   private WsVictor m_rearLeftDrive = null;
   private WsVictor m_rearRightDrive = null;

   private WsVictor m_frontLeftRotate = null;
   private WsVictor m_frontRightRotate = null;
   private WsVictor m_rearLeftRotate = null;
   private WsVictor m_rearRightRotate = null;

   // Home position variables - state of the home switch, and calculated encoder offset
   private double m_frontLeftEncOffset = 0.0;
   private double m_frontRightEncOffset = 0.0;
   private double m_rearLeftEncOffset = 0.0;
   private double m_rearRightEncOffset = 0.0;
   
   private boolean m_flHome;
   private boolean m_frHome;
   private boolean m_rlHome;
   private boolean m_rrHome;
   
   
   private boolean m_homeButton1 = false;
   private boolean m_homeButton2 = false;
   
   // Speed output related variables
   private static final double DEFAULT_SCALE_FACTOR = 0.75;
   private static final double ANTI_TURBO_SCALE_FACTOR = 0.5;
   private static final double TURBO_SCALE_SCALE_FACTOR = 1.0;
   private boolean m_antiTurbo = false;
   private boolean m_turbo = false;
   private double m_scaleFactor = DEFAULT_SCALE_FACTOR;
   
   private CrabDriveMode m_crabDriveMode = new CrabDriveMode();
   private SwerveDriveMode m_swerveDriveMode = new SwerveDriveMode();
   private SwerveBaseState m_prevState = new SwerveBaseState(new WheelModuleState(), new WheelModuleState(), new WheelModuleState(), new WheelModuleState());
   
   private static final DecimalFormat s_format = new DecimalFormat("#.##");
   
   private static Logger s_log = Logger.getLogger(SwerveDrive.class.getName());
   private static final String s_className = "SwerveDrive";
   
   
   // TODO: These constants should be configurable via the config file
   // Minimum output for rotation motor controllers
   private static final double MIN_ROTATION_OUTPUT = 0.1;
   
   // The angle distance (degrees) between the target and current angle before stopping output to the rotation motors
   private static final int ROTATION_TARGET_TOLERANCE = 2;

   public SwerveDrive()
   {
   }
   
   @Override
   public void inputUpdate(Input p_source)
   {
      if (p_source.getName().equals(SwerveInputs.DRV_BUTTON_1.getName()))
      {
      }
      else if (p_source.getName().equals(SwerveInputs.DRV_BUTTON_2.getName()))
      {
      }
      else if (p_source.getName().equals(SwerveInputs.DRV_BUTTON_3.getName()))
      {
      }
      else if (p_source.getName().equals(SwerveInputs.DRV_BUTTON_4.getName()))
      {
      }
      else if (p_source.getName().equals(SwerveInputs.DRV_BUTTON_5.getName()))
      {
         // TODO: Assume button 1 changes drive mode
         m_recalcMode = true;
      }
      else if (p_source.getName().equals(SwerveInputs.DRV_BUTTON_6.getName()))
      {
      }
      else if (p_source.getName().equals(SwerveInputs.TURBO.getName()))
      {
         m_turbo = ((DigitalInput)p_source).getValue();
      }
      else if (p_source.getName().equals(SwerveInputs.ANTI_TURBO.getName()))
      {
         m_antiTurbo = ((DigitalInput)p_source).getValue();
      }
      else if (p_source.getName().equals(SwerveInputs.HOME_BUTTON_1.getName()))
      {
         m_homeButton1 = ((DigitalInput)p_source).getValue();
      }
      else if (p_source.getName().equals(SwerveInputs.HOME_BUTTON_2.getName()))
      {
         m_homeButton2 = ((DigitalInput)p_source).getValue();
      }
      else if (p_source.getName().equals(SwerveInputs.DRV_BUTTON_11.getName()))
      {
      }
      else if (p_source.getName().equals(SwerveInputs.DRV_BUTTON_12.getName()))
      {
      }
      else if (p_source.getName().equals(SwerveInputs.DRV_HEADING_X.getName()))
      {
         m_headingX = ((AnalogInput) p_source).getValue();
      }
      else if (p_source.getName().equals(SwerveInputs.DRV_HEADING_Y.getName()))
      {
         m_headingY = ((AnalogInput) p_source).getValue();
      }
      else if (p_source.getName().equals(SwerveInputs.DRV_ROTATION.getName()))
      {
         m_joystickRotation = ((AnalogInput) p_source).getValue();
      }
      else if (p_source.getName().equals(SwerveInputs.FRONT_LEFT_HOME.getName()))
      {
         m_flHome = ((DigitalInput) p_source).getValue();
      }
      else if (p_source.getName().equals(SwerveInputs.FRONT_RIGHT_HOME.getName()))
      {
         m_frHome = ((DigitalInput) p_source).getValue();
      }
      else if (p_source.getName().equals(SwerveInputs.REAR_LEFT_HOME.getName()))
      {
         m_rlHome = ((DigitalInput) p_source).getValue();
      }
      else if (p_source.getName().equals(SwerveInputs.REAR_RIGHT_HOME.getName()))
      {
         m_rrHome = ((DigitalInput) p_source).getValue();
      }
      else if (p_source.getName().equals(SwerveInputs.DRV_RIGHT_Y.getName()))
      {
      }
      else if (p_source.getName().equals(SwerveInputs.FRONT_LEFT_ROT_ENCODER.getName()))
      {
         m_flEncoder = ((AnalogInput) p_source).getValue();
      }
      else if (p_source.getName().equals(SwerveInputs.FRONT_RIGHT_ROT_ENCODER.getName()))
      {
         m_frEncoder = ((AnalogInput) p_source).getValue();
      }
      else if (p_source.getName().equals(SwerveInputs.REAR_LEFT_ROT_ENCODER.getName()))
      {
         m_rlEncoder = ((AnalogInput) p_source).getValue();
      }
      else if (p_source.getName().equals(SwerveInputs.REAR_RIGHT_ROT_ENCODER.getName()))
      {
         m_rrEncoder = ((AnalogInput) p_source).getValue();
      }
   }

   @Override
   public void init()
   {
      Core.getInputManager().getInput(SwerveInputs.DRV_BUTTON_1.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.DRV_BUTTON_2.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.DRV_BUTTON_3.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.DRV_BUTTON_4.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.DRV_BUTTON_5.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.DRV_BUTTON_6.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.TURBO.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.ANTI_TURBO.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.HOME_BUTTON_1.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.HOME_BUTTON_2.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.DRV_BUTTON_11.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.DRV_BUTTON_12.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.DRV_ROTATION.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.DRV_HEADING_Y.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.DRV_HEADING_X.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.DRV_RIGHT_Y.getName()).addInputListener(this);

      Core.getInputManager().getInput(SwerveInputs.FRONT_LEFT_HOME.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.FRONT_RIGHT_HOME.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.REAR_LEFT_HOME.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.REAR_RIGHT_HOME.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.FRONT_LEFT_ROT_ENCODER.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.FRONT_RIGHT_ROT_ENCODER.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.REAR_LEFT_ROT_ENCODER.getName()).addInputListener(this);
      Core.getInputManager().getInput(SwerveInputs.REAR_RIGHT_ROT_ENCODER.getName()).addInputListener(this);
      
      // Retrieve the drive motor outputs
      m_frontLeftDrive = (WsVictor)(Core.getOutputManager().getOutput(WSOutputs.FRONT_LEFT.getName()));
      m_frontRightDrive = (WsVictor)(Core.getOutputManager().getOutput(WSOutputs.FRONT_RIGHT.getName()));
      m_rearLeftDrive = (WsVictor)(Core.getOutputManager().getOutput(WSOutputs.REAR_LEFT.getName()));
      m_rearRightDrive = (WsVictor)(Core.getOutputManager().getOutput(WSOutputs.REAR_RIGHT.getName()));
      m_frontLeftRotate = (WsVictor)(Core.getOutputManager().getOutput(WSOutputs.FRONT_LEFT_ROT.getName()));
      m_frontRightRotate = (WsVictor)(Core.getOutputManager().getOutput(WSOutputs.FRONT_RIGHT_ROT.getName()));
      m_rearLeftRotate = (WsVictor)(Core.getOutputManager().getOutput(WSOutputs.REAR_LEFT_ROT.getName()));
      m_rearRightRotate = (WsVictor)(Core.getOutputManager().getOutput(WSOutputs.REAR_RIGHT_ROT.getName()));
      
   }

   @Override
   public void update()
   {
      // If this is the first update cycle, get the current encoder values so we know the offset
      // The assumption is that the home position has been set, prior to a code restart (eg. going on to the field)
      // We can't do this in init() as the inputs have not been read
      if (firstTime)
      {
         // Get the encoder offset for each wheel - this assumes they have been set to the home position
         m_frontLeftEncOffset = ((AnalogInput)Core.getInputManager().getInput(SwerveInputs.FRONT_LEFT_ROT_ENCODER.getName())).getValue();
         m_frontRightEncOffset = ((AnalogInput)Core.getInputManager().getInput(SwerveInputs.FRONT_RIGHT_ROT_ENCODER.getName())).getValue();
         m_rearLeftEncOffset = ((AnalogInput)Core.getInputManager().getInput(SwerveInputs.REAR_LEFT_ROT_ENCODER.getName())).getValue();
         m_rearRightEncOffset = ((AnalogInput)Core.getInputManager().getInput(SwerveInputs.REAR_RIGHT_ROT_ENCODER.getName())).getValue();
         
         firstTime = false;
      }
      
      long start = System.nanoTime();

      // If both 'home' buttons are pressed (select and start), move to the home position
      if (m_homeButton1 && m_homeButton2)
      {
         updateModulesForHome();
      }
      else
      {
   
         SwerveBaseState currentState = null;
         
         calculateSpeedScaleFactor();

         if (m_recalcMode)
         {
            recalculateDriveMode();
         }
         
         switch (m_mode)
         {
            case CRAB:
               currentState = m_crabDriveMode.calculateNewState(m_prevState, m_headingX, m_headingY, m_joystickRotation);
               break;
            case SWERVE:
               currentState = m_swerveDriveMode.calculateNewState(m_prevState, m_headingX, m_headingY, m_joystickRotation);
               break;
            // TODO: Add any more modes here
            default:
               currentState = m_crabDriveMode.calculateNewState(m_prevState, m_headingX, m_headingY, m_joystickRotation);
               break;
         }
         
         // Set values on the outputs for update
         updateModuleStates(currentState);

         m_prevState = currentState;
      }

      long end = System.nanoTime();
      s_log.fine("Swerve update time: " + (end - start) / 1000000 + "ms");
   }

   
   private void calculateSpeedScaleFactor()
   {
      if (m_antiTurbo)
      {
         m_scaleFactor = ANTI_TURBO_SCALE_FACTOR;
      }
      else if (m_turbo)
      {
         m_scaleFactor = TURBO_SCALE_SCALE_FACTOR;
      }
      else
      {
         m_scaleFactor = DEFAULT_SCALE_FACTOR;
      }
   }
   
   private void updateModuleStates(SwerveBaseState state)
   {
      // Calculate encoder target based on offsets
      int flAdjusted = adjustForEncoderOffset(state.getFrontLeft().getRotationAngle(), (int)m_frontLeftEncOffset);
      int frAdjusted = adjustForEncoderOffset(state.getFrontRight().getRotationAngle(), (int)m_frontRightEncOffset);
      int rlAdjusted = adjustForEncoderOffset(state.getRearLeft().getRotationAngle(), (int)m_rearLeftEncOffset);
      int rrAdjusted = adjustForEncoderOffset(state.getRearRight().getRotationAngle(), (int)m_rearRightEncOffset);

      // Set rotation speed
      m_frontLeftRotate.setValue(calculateRotationSpeed((int)m_flEncoder, flAdjusted, ROTATION_TARGET_TOLERANCE));
      m_frontRightRotate.setValue(calculateRotationSpeed((int)m_frEncoder, frAdjusted, ROTATION_TARGET_TOLERANCE));
      m_rearLeftRotate.setValue(calculateRotationSpeed((int)m_rlEncoder, rlAdjusted, ROTATION_TARGET_TOLERANCE));
      m_rearRightRotate.setValue(calculateRotationSpeed((int)m_rrEncoder, rrAdjusted, ROTATION_TARGET_TOLERANCE));

      // Set motor speed
      m_frontLeftDrive.setValue(state.getFrontLeft().getSpeed() * m_scaleFactor);
      m_frontRightDrive.setValue(state.getFrontRight().getSpeed() * m_scaleFactor);
      m_rearLeftDrive.setValue(state.getRearLeft().getSpeed() * m_scaleFactor);
      m_rearRightDrive.setValue(state.getRearRight().getSpeed() * m_scaleFactor);
   }

   
   private void updateModulesForHome()
   {
      double flRotation;
      double frRotation;
      double rlRotation;
      double rrRotation;

      // Rotate each wheel 1 degree more clockwise until they hit their home position
      if (!m_flHome)
      {
         flRotation = calculateRotationSpeed((int)m_flEncoder, (int)m_flEncoder + 1, 0);
      }
      else
      {
         m_frontLeftEncOffset = m_flEncoder;
         flRotation = 0.0;
      }

      if (!m_frHome)
      {
         frRotation = calculateRotationSpeed((int)m_frEncoder, (int)m_frEncoder + 1, 0);
      }
      else
      {
         m_frontRightEncOffset = m_frEncoder;
         frRotation = 0.0;
      }

      if (!m_rlHome)
      {
         rlRotation = calculateRotationSpeed((int)m_rlEncoder, (int)m_rlEncoder, 0);
      }
      else
      {
         m_rearLeftEncOffset = m_rlEncoder;
         rlRotation = 0.0;
      }

      if (!m_rrHome)
      {
         rrRotation = calculateRotationSpeed((int)m_rrEncoder, (int)m_rrEncoder + 1, 0);
      }
      else
      {
         m_rearRightEncOffset = m_rrEncoder;
         rrRotation = 0.0;
      }

      // Note: Don't adjust the target angle for the offset, since that's what we're trying to calculate here!
      
      m_frontLeftRotate.setValue(flRotation);
      m_frontRightRotate.setValue(frRotation);
      m_rearLeftRotate.setValue(rlRotation);
      m_rearRightRotate.setValue(rrRotation);

      // Set motor speed
      m_frontLeftDrive.setValue(0.0);
      m_frontRightDrive.setValue(0.0);
      m_rearLeftDrive.setValue(0.0);
      m_rearRightDrive.setValue(0.0);
   }
   
   
   private int adjustForEncoderOffset(int p_target, int p_offset)
   {
      return (p_target + p_offset) % 360;
   }
   
   
   private double calculateRotationSpeed(int p_prev, int p_target, int p_tolerance)
   {
      double rotationSpeed = 0.0;
      
      // Usually the angle changes will be small.  For large changes (> 180 difference)
      // follow the shortest path to the new position
      // Smooth the output so that it slows near the target position
      // Limit the minimum output to some percentage (20%?) to prevent stalling
      
      int diff = p_target - p_prev;
      int distanceToTarget = Math.abs(diff);
      int  dir = 1;
      
      if (p_prev > p_target)
      {
         if (distanceToTarget >= 180)
         {
            dir = 1;
         }
         else
         {
            dir = -1;
         }
      }
      else if (p_prev < p_target)
      {
         if (distanceToTarget >= 180)
         {
            dir = -1;
         }
         else
         {
            dir = 1;
         }
      }
      else
      {
         // Prev and target are equal - nowhere to go
      }
      
      // If we are going past half a rotation, go the shortest route
      // Direction has already been taken care of above
      if (distanceToTarget >= 180)
      {
         distanceToTarget = 360 - distanceToTarget;
      }

      // Determine the speed of the motor
      // Scale based on proportion of distance to travel of 180 degrees
      // - 180 degrees away results in full speed
      // - closer is slower
      // - limit minimum output to 15%
      rotationSpeed = (double)distanceToTarget / 180;

      // If we are within tolerance of the target angle, stop turning
      if (distanceToTarget <= p_tolerance)
      {
         rotationSpeed = 0.0;
      }
      // If we are below our minimum useful output, set it to the minimum
      else if (rotationSpeed < MIN_ROTATION_OUTPUT)
      {
         rotationSpeed = MIN_ROTATION_OUTPUT;
      }

      // Set the correct direction
      rotationSpeed *= dir;

      s_log.fine("Encoder: " + s_format.format(m_flEncoder) + "\tTarget: " + p_target + "\tDistance: " + distanceToTarget + "\tresult: " + s_format.format(rotationSpeed) + "\n");

      return rotationSpeed;
   }

   private void recalculateDriveMode()
   {
      // Set m_mode based on which button is pressed for different drive modes, if any
      // We can set it directly in inputUpdate(), or use this for more modes by combining buttons
      // for more values
      if (((DigitalInput)Core.getInputManager().getInput(SwerveInputs.DRV_BUTTON_5.getName())).getValue())
      {
         m_mode = SWERVE;
      }
      else
      {
         m_mode = CRAB;
      }
      
      m_recalcMode = false;
   }
   
   @Override
   public void selfTest()
   {
   }

   @Override
   public String getName()
   {
      return WSSubsystems.SWERVE_BASE.getName();
   }

}
