/*-----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                              */
/* Open Source Software - may be modified and shared by FRC teams. The code    */
/* must be accompanied by the FIRST BSD license file in the root directory of  */
/* the project.                                                                */
/*                                                                             */
/* To keep from calling SmartDashboard every 20 msec., we will create a state  */
/* controller for each system check.  After each state change, call            */
/* SmartDashboard to update states.                                            */
/*                                                                             */
/* This class checks the pressure status.  If pressure voltages is above the   */
/* target value, then pressure_ok is set to true.  This can be used by the     */
/* robot to inhibit / enable pneumatic actuation. This class also times the    */
/* match.  The clock is reset when autonomous is initialized.  Once the clock  */
/* is above 90 seconds, then the system will signal driver that it is time to  */
/* climb.                                                                      */
/*-----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SystemStatus
{
  enum SystemStates 
  {
    wait_for_activation,
    initialize, 
    wait_for_elapsetime, 
    wait_for_false 
  };

  final double TargetCountDown = 90.0;           // signal driver when countdown is 45 seconds.
  private SystemStates game_state;
  private boolean climb_now;
  private Timer gclock = new Timer();


  public SystemStatus()
  {
    System.out.println( "SystemStatus Constructor");
    climb_now = false;
    SmartDashboard.putBoolean("Climb", climb_now);
    game_state = SystemStates.wait_for_activation;
  }

   
  /*****************************************************************/
  /* StartGameClock() - starts the timer gclock when match is      */
  /*                    started.                                   */
  /*****************************************************************/ 
  public void StartGameClock()
  {
    game_state = SystemStates.initialize;
  }


  /*****************************************************************/
  /* Time_To_Climb() - returns true if time remaining is less than */
  /*                   45 seconds.                                 */
  /*****************************************************************/ 
  public boolean Time_To_Climb()
  {
    return climb_now;
  }


  /*****************************************************************/
  /* Check_System_Status() - checks game clock to see if it's time */
  /*                         to climb.                             */
  /*****************************************************************/ 
  public void Check_System_Status()
  {
    switch (game_state)
    {
      case wait_for_activation:
        break;

      case initialize:
        gclock.reset();     
        gclock.start(); 
        climb_now = false;
        SmartDashboard.putBoolean("Climb", climb_now);
        game_state = SystemStates.wait_for_elapsetime;
        break;
    
      case wait_for_elapsetime:
        if( gclock.get() > TargetCountDown )
        {
          climb_now = true;
          SmartDashboard.putBoolean("Climb", climb_now);
          game_state = SystemStates.wait_for_activation;
        }
        break;

      default:
        climb_now = false;
        SmartDashboard.putBoolean("Climb", climb_now);
        game_state = SystemStates.wait_for_activation;
        break;
    }
  }
}
