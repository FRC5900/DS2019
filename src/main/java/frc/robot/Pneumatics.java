/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;

enum CylinderStates 
{
  idle,
  wait_for_extend_cmd,
  wait_for_extend,
  wait_for_extend_release,
  wait_for_retract_cmd,
  wait_for_retract,  
  wait_for_retract_release
};

enum PressureStates 
{
  initialize, 
  wait_tanks_pressurize, 
  tanks_pressurized
};

public class Pneumatics 
{ 
  Joystick jstick;

  Compressor c = new Compressor(0);
  Solenoid Front_Lift = new Solenoid(0);
  Solenoid Rear_Lift = new Solenoid(1);
  Solenoid Ball_Grab = new Solenoid(3);
  Solenoid Ball_Gateway = new Solenoid(2); 

  Timer FrontTimer = new Timer();
  Timer RearTimer = new Timer();

  CylinderStates Front_Lift_State;
  CylinderStates Rear_Lift_State;
  CylinderStates BallGateway_State;

  final double CylinderActuationTime = 1.5;      // Need to measure this
  private  PressureStates pressure_state = PressureStates.initialize;
  private boolean pressure_ok;
 
  final double PressureMax = 2.0;                // change when ready to deploy
  private double PressureVolts;
  private int PressureDebounce;
  AnalogInput TankPressure = new AnalogInput(0);

  public Pneumatics( Joystick stick )
  {
    System.out.println( "Pneumatics Constructor");
    c.setClosedLoopControl(true);                // Turn on Compressor    
    jstick = stick;
  }
  
  public void Initialize()
  {    
    Front_Lift.set(false);
    Rear_Lift.set(false);
    Ball_Gateway.set(false);
    pressure_state = PressureStates.initialize;
    Front_Lift_State = CylinderStates.wait_for_extend_cmd;
    Rear_Lift_State = CylinderStates.wait_for_extend_cmd;
    BallGateway_State = CylinderStates.wait_for_extend_cmd;
  }


  /*****************************************************************/
  /* Cylinder_Controls() - checks pressure on air tanks and calls  */
  /*                       the controllers for front, rear and     */
  /*                       ball gateway cylinders.                 */
  /*****************************************************************/ 
  public void Cylinder_Controls()
  {
    Check_Air_Pressure();
    FrontLift_Controller();
    RearLift_Controller();
    BallGateway_Controller();   
  }


  /*****************************************************************/
  /* Check_Air_Pressure() - read Analog pressure sensor and        */
  /*                        debounce.                              */
  /*****************************************************************/ 
  private void Check_Air_Pressure()
  {
    PressureVolts = TankPressure.getVoltage();   // Read Analog Pressure Sensor 
    switch (pressure_state)
    {
      case initialize:        
        pressure_ok = false;
        SmartDashboard.putBoolean("PressureOK", pressure_ok);
        pressure_state = PressureStates.wait_tanks_pressurize;
        PressureDebounce = 0;
        break;
    
      case wait_tanks_pressurize:
        if( PressureVolts >= PressureMax )
        {
          if( ++PressureDebounce > 10 )
          {
            PressureDebounce = 0;  
            pressure_ok = true;
            SmartDashboard.putBoolean("Pressure", pressure_ok);
            pressure_state = PressureStates.tanks_pressurized;           
          } 
        }  
        else     
          PressureDebounce = 0;
        break;

      case tanks_pressurized:
        if( PressureVolts < PressureMax)
        {
          if( ++PressureDebounce > 10 )
          {
            PressureDebounce = 0;   
            pressure_ok = false;
            SmartDashboard.putBoolean("Pressure", pressure_ok);
            pressure_state = PressureStates.wait_tanks_pressurize;
          }  
        }
        else 
          PressureDebounce = 0;   
        break;

      default:
        pressure_state = PressureStates.initialize;
        break;
    } 
  }


  /*****************************************************************/
  /* FrontLift_Controller() - read leftstick button 4 to raise     */
  /*                            or lower FrontLift cylinder.       */
  /*****************************************************************/ 
  public void FrontLift_Controller()
  {
    switch (Front_Lift_State)
    {
      case idle:
        break;
    
      case wait_for_extend_cmd:
        if (jstick.getRawButton(4) == true && pressure_ok )
        {      
          Front_Lift.set(true);
          FrontTimer.reset();
          FrontTimer.start();
          Front_Lift_State = CylinderStates.wait_for_extend;
          SmartDashboard.putString("FrontLift", "Extending");
        }
        break;
    
      case wait_for_extend:
        if( FrontTimer.get() > CylinderActuationTime)
        {
          Front_Lift_State = CylinderStates.wait_for_extend_release;
          FrontTimer.stop();
          SmartDashboard.putString("FrontLift", "Extended");
        }
        break;
    
      case wait_for_extend_release:
        if (jstick.getRawButton(4) == false)
          Front_Lift_State = CylinderStates.wait_for_retract_cmd;
        break;
    
      case wait_for_retract_cmd:
        if (jstick.getRawButton(4) == true && pressure_ok)  
        {      
          Front_Lift.set(false);
          FrontTimer.reset();
          FrontTimer.start();
          Front_Lift_State = CylinderStates.wait_for_retract;
          SmartDashboard.putString("FrontLift", "Retracting");
        }
        break;
    
      case wait_for_retract:
        if( FrontTimer.get() > CylinderActuationTime)
        {
          Front_Lift_State = CylinderStates.wait_for_retract_release;
          FrontTimer.stop();
          SmartDashboard.putString("FrontLift", "Retracted");          
        }
        break;
            
      case wait_for_retract_release:
        if (jstick.getRawButton(4) == false)
          Front_Lift_State = CylinderStates.wait_for_extend_cmd;
        break;
    
      default:
        Front_Lift_State = CylinderStates.wait_for_extend_cmd;
        break;
    }
  }
  

  /*****************************************************************/
  /* RearLift_Controller() - read leftstick button 5 to raise      */
  /*                            or lower RearLift cylinder.        */
  /*****************************************************************/ 
  public void RearLift_Controller()
  {
	switch (Rear_Lift_State)
    {
      case idle:
        break;

      case wait_for_extend_cmd:
        if (jstick.getRawButton(5) == true && pressure_ok)
        {      
          Rear_Lift.set(true);
          RearTimer.reset();
          RearTimer.start();
          Rear_Lift_State = CylinderStates.wait_for_extend;
          SmartDashboard.putString("RearLift", "Extending");
        }
        break;

      case wait_for_extend:
        if( RearTimer.get() > CylinderActuationTime)
        {
          Rear_Lift_State = CylinderStates.wait_for_extend_release;
          RearTimer.stop();
          SmartDashboard.putString("RearLift", "Extended");
        }
        break;

      case wait_for_extend_release:
        if (jstick.getRawButton(5) == false)
          Rear_Lift_State = CylinderStates.wait_for_retract_cmd;
        break;

      case wait_for_retract_cmd:
        if (jstick.getRawButton(5) == true && pressure_ok)
        {      
          Rear_Lift.set(false);
          RearTimer.reset();
          RearTimer.start();
          Rear_Lift_State = CylinderStates.wait_for_retract;
          SmartDashboard.putString("RearLift", "Extending");
        }
        break;

      case wait_for_retract:
        if( RearTimer.get() > CylinderActuationTime)
        {
          Rear_Lift_State = CylinderStates.wait_for_retract_release;
          RearTimer.stop();
          SmartDashboard.putString("RearLift", "Retracted");          
        }
        break;
        
      case wait_for_retract_release:
        if (jstick.getRawButton(5) == false)
          Rear_Lift_State = CylinderStates.wait_for_extend_cmd;
        break;

      default:
        Rear_Lift_State = CylinderStates.wait_for_extend_cmd;
        break;
    }
  }


  /*****************************************************************/
  /* BallGateway_Controller() - read leftstick button 1 to raise   */
  /*                            or lower BallGateway cylinder.     */
  /*****************************************************************/   
  public void BallGateway_Controller() 
  {
    switch(BallGateway_State)
    {
      case wait_for_extend_cmd:
        if (jstick.getRawButton(1) == true)
        {
          Ball_Gateway.set(true);
          BallGateway_State = CylinderStates.wait_for_extend_release;
        } 
        break;
 
      case wait_for_extend_release:
        if (jstick.getRawButton(1) == false)
          BallGateway_State = CylinderStates.wait_for_retract_cmd;
        break;
 
      case wait_for_retract_cmd:
        if (jstick.getRawButton(1) == true)
        {
          Ball_Gateway.set(false);
          BallGateway_State = CylinderStates.wait_for_retract_release;
        }
        break;
 
      case wait_for_retract_release:
        if (jstick.getRawButton(1) == false)
          BallGateway_State = CylinderStates.wait_for_extend_cmd;
        break;
       
      default:
        Ball_Gateway.set(false);
        BallGateway_State = CylinderStates.wait_for_extend_cmd;
        break;
    }   
  }

}
