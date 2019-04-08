/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
  Solenoid Ball_Gateway = new Solenoid(1); 
  DoubleSolenoid Rear_Lift = new DoubleSolenoid(4, 5);
  DoubleSolenoid Rear_Lift2 = new DoubleSolenoid(6, 7);
 
  Timer FrontTimer = new Timer();
  Timer RearTimer = new Timer();

  CylinderStates Front_Lift_State;
  CylinderStates Rear_Lift_State;
  CylinderStates BallGateway_State;

  final double CylinderActuationTime = 1.25;      // Need to measure this
  private  PressureStates pressure_state = PressureStates.initialize;
  private boolean pressure_ok;
 
  final double PressureMax = 2.0;                // change when ready to deploy
  private double PressureVolts;
  private int PressureDebounce = 0;
  private int FrontLift_Debounce = 0;
  private int RearLift_Debounce = 0;
  AnalogInput TankPressure = new AnalogInput(0);

  public Pneumatics( Joystick stick )
  {
    System.out.println( "Pneumatics Constructor");
    c.setClosedLoopControl(true);                // Turn on Compressor    
    jstick = stick;
  }
  
  public void Initialize()
  {  
    c.setClosedLoopControl(true);   
    Front_Lift.set(false);
    Rear_Lift.set(DoubleSolenoid.Value.kReverse);
    Rear_Lift2.set(DoubleSolenoid.Value.kReverse);
    Ball_Gateway.set(false);
    pressure_state = PressureStates.initialize;
    SmartDashboard.putBoolean("PressureOK", pressure_ok);
    PressureDebounce = 0;
    FrontLift_Debounce = 0;
    RearLift_Debounce = 0;
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
            SmartDashboard.putBoolean("PressureOK", pressure_ok);
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
            SmartDashboard.putBoolean("PressureOK", pressure_ok);
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
          if( ++FrontLift_Debounce > 10 )
          {
            Front_Lift.set(true);
            FrontTimer.reset();
            FrontTimer.start();
            Front_Lift_State = CylinderStates.wait_for_extend;
            SmartDashboard.putString("FrontLift", "Extending");
          }
        }
        else 
          FrontLift_Debounce = 0;
        break;
    
      case wait_for_extend:
        if( FrontTimer.get() > CylinderActuationTime)
        {
          FrontLift_Debounce = 0;
          Front_Lift_State = CylinderStates.wait_for_extend_release;
          FrontTimer.stop();
          SmartDashboard.putString("FrontLift", "Extended");
        }
        break;
    
      case wait_for_extend_release:
        if (jstick.getRawButton(4) == false)
        {
          if ( ++FrontLift_Debounce > 10 )
          {
            FrontLift_Debounce = 0;
            Front_Lift_State = CylinderStates.wait_for_retract_cmd;
          }
        }
        else 
          FrontLift_Debounce = 0;
        break;
    
      case wait_for_retract_cmd:
        if (jstick.getRawButton(4) == true && pressure_ok)  
        {      
          if (++FrontLift_Debounce > 10)
          {
            Front_Lift.set(false);
            FrontTimer.reset();
            FrontTimer.start();
            Front_Lift_State = CylinderStates.wait_for_retract;
            SmartDashboard.putString("FrontLift", "Retracting");
          }
        }
        else 
          FrontLift_Debounce = 0;
        break;
    
      case wait_for_retract:
        if( FrontTimer.get() > CylinderActuationTime)
        {
          FrontLift_Debounce = 0;
          Front_Lift_State = CylinderStates.wait_for_retract_release;
          FrontTimer.stop();
          SmartDashboard.putString("FrontLift", "Retracted");          
        }
        break;
            
      case wait_for_retract_release:
        if (jstick.getRawButton(4) == false)
        {
          if (++FrontLift_Debounce > 10 )
          {
            FrontLift_Debounce = 0;
            Front_Lift_State = CylinderStates.wait_for_extend_cmd;
          }
        }
        else 
          FrontLift_Debounce = 0;
        break;
    
      default:
        FrontLift_Debounce = 0;
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
          if (++RearLift_Debounce > 10 )  
          {
            Rear_Lift.set(DoubleSolenoid.Value.kForward);
            Rear_Lift2.set(DoubleSolenoid.Value.kForward);
            RearTimer.reset();
            RearTimer.start();
            Rear_Lift_State = CylinderStates.wait_for_extend;
            SmartDashboard.putString("RearLift", "Extending");
          }  
        }
        else 
          RearLift_Debounce = 0;
        break;

      case wait_for_extend:
        if( RearTimer.get() > CylinderActuationTime)
        {
          RearLift_Debounce = 0;
          Rear_Lift_State = CylinderStates.wait_for_extend_release;
          RearTimer.stop();
          SmartDashboard.putString("RearLift", "Extended");
        }
        break;

      case wait_for_extend_release:
        if (jstick.getRawButton(5) == false)
        {
          if (++RearLift_Debounce > 10)
          {
            RearLift_Debounce = 0;
            Rear_Lift_State = CylinderStates.wait_for_retract_cmd;
          }
        }
        else 
          RearLift_Debounce = 0;
        break;

      case wait_for_retract_cmd:
        if (jstick.getRawButton(5) == true && pressure_ok)
        {  
          if ( ++RearLift_Debounce > 10 )    
          {
            Rear_Lift.set(DoubleSolenoid.Value.kReverse);
            Rear_Lift2.set(DoubleSolenoid.Value.kReverse);
            RearTimer.reset();
            RearTimer.start();
            Rear_Lift_State = CylinderStates.wait_for_retract;
            SmartDashboard.putString("RearLift", "Extending");
          }
        }
        else 
          RearLift_Debounce = 0;
        break;

      case wait_for_retract:
        if( RearTimer.get() > CylinderActuationTime)
        {
          RearLift_Debounce = 0;
          Rear_Lift_State = CylinderStates.wait_for_retract_release;
          RearTimer.stop();
          SmartDashboard.putString("RearLift", "Retracted");          
        }
        break;
        
      case wait_for_retract_release:
        if (jstick.getRawButton(5) == false)
        {
          if (++RearLift_Debounce > 10)
          {
            RearLift_Debounce = 0;
            Rear_Lift_State = CylinderStates.wait_for_extend_cmd;
          }  
        }
        else 
          RearLift_Debounce = 0;
        break;

      default:
        RearLift_Debounce = 0;
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
