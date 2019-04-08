/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* Left Joystick at USB Port 0                                                */  
/* Right Joystick at USB Port 1                                               */                              
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  public Joystick LeftStick;
  public Joystick RightStick;
  public DifferentialDrive myRobot;
  private SpeedController BallIntakeMotor;
  private SpeedControllerGroup m_Left;
  private SpeedControllerGroup m_Right;
  private SpeedControllerGroup m_Winch;
  private int WinchCounter = 0; 
  UsbCamera cam0;
  UsbCamera cam1;

  public SystemStatus sysstat = new SystemStatus();     // This object check game clock and pressure level
  public Pneumatics lifts;                              // This object controls lift cyclinders
 
  final double RobotMaxSpeed = 1;
  final double RobotMaxTurnSpeed = 0.85;
  final double RobotNormalSpeed = .80;
  final double RobotNormalTurnSpeed = 0.70;
 
  final double BallIntakeMaxSpeed = 1.0;
  final double WinchMaxSpeed = 1.0;
  final double DeadBand = 0.1;
  private Timer robot_timer;
  final double auto_robot_move_time = 3.0;
  final double auto_robot_move_speed = 0.4;
  enum Robot_Move_States 
  {
    idle, 
    set_up_move,
    wait_for_move_complete,
    finished
  };

  Robot_Move_States robot_move_state = Robot_Move_States.idle;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    SmartDashboard.putString("Mode", "robotInit");

    LeftStick = new Joystick(0);
    RightStick = new Joystick(1);
    lifts = new Pneumatics(LeftStick);           // Lift cyclinders, controlled by Left JoyStick
    lifts.Initialize();                          // Initialize lift cylinders

    m_Left = new SpeedControllerGroup( new Spark(0), new Spark(1));
    m_Right = new SpeedControllerGroup( new Spark(2), new Spark(3));
    m_Winch = new SpeedControllerGroup( new Spark(4), new Spark(5));
    BallIntakeMotor = new Spark(6);  
    myRobot = new DifferentialDrive(m_Left, m_Right);
   
    cam0 = CameraServer.getInstance().startAutomaticCapture();
    cam0.setResolution(640, 480);
    cam1 = CameraServer.getInstance().startAutomaticCapture(1);
    cam1.setResolution(640, 480);   
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    sysstat.Check_System_Status();               // This object check game clock
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() 
  {
    SmartDashboard.putString("Mode", "autonomousInit");
    sysstat.StartGameClock();                    // Start Game Clock and monitor game elapsed time
    robot_move_state = Robot_Move_States.idle;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {
    switch ( robot_move_state )
    {
      case idle:
      case finished:
        lifts.Cylinder_Controls();
        Drive_Controls();
        Winch_Controls();
        BallInTake_Controls();
        if( RightStick.getRawButton(3) == true )
        { 
          robot_move_state = Robot_Move_States.set_up_move;
        }
        break;
      
      case set_up_move:
      case wait_for_move_complete:
        Move_Robot_Controller();
        break;
    }   
  }

  @Override
  public void teleopInit() 
  {
    SmartDashboard.putString("Mode", "teleopInit");
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  { 
    lifts.Cylinder_Controls();
    Drive_Controls();   
    Winch_Controls();
    BallInTake_Controls();
  }

  @Override
  public void testInit() 
  {
    SmartDashboard.putString("Mode", "testInit");
    sysstat.StartGameClock();                    // Start Game Clock and monitor game elapsed time
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() 
  { 
    lifts.Cylinder_Controls();
    Drive_Ramp_Controls();                      // Test Only - need input from drivers
    Winch_Controls();
    BallInTake_Controls();
  }
  

  /*****************************************************************/
  /* Drive_Controls() - read rightstick to get speed               */
  /*****************************************************************/  
  public void Drive_Controls()
  {
    Double RobotActualSpeed;
    Double RobotActualTurnSpeed;
    Double TurnSpeed;
    Double DriveSpeed;

    if(RightStick.getRawButton(1))
    {
      RobotActualSpeed = RobotMaxSpeed;
      RobotActualTurnSpeed = RobotMaxTurnSpeed;
    }
    else 
    {
      RobotActualSpeed = RobotNormalSpeed;
      RobotActualTurnSpeed = RobotNormalTurnSpeed;
    }

    TurnSpeed = RightStick.getX();
    TurnSpeed = TurnSpeed * RobotActualTurnSpeed;
    if ((TurnSpeed < DeadBand) && (TurnSpeed > -DeadBand)) 
      TurnSpeed = 0.0;

    DriveSpeed = RightStick.getY();
    DriveSpeed = DriveSpeed * RobotActualSpeed;
    if ((DriveSpeed < DeadBand) &&  (DriveSpeed > -DeadBand)) 
      DriveSpeed = 0.0;  

    myRobot.arcadeDrive(-DriveSpeed, TurnSpeed);
  }


  /*****************************************************************/
  /* Drive_Ramp_Controls() - square the joystick input to create a */
  /*                         parabolic ramp.                       */
  /*****************************************************************/  
  public void Drive_Ramp_Controls()
  {
    Double RobotActualSpeed;
    Double RobotActualTurnSpeed;
    Double TurnSpeed;
    Double DriveSpeed;

    if(RightStick.getRawButton(1))
    {
      RobotActualSpeed = RobotMaxSpeed;
      RobotActualTurnSpeed = RobotMaxTurnSpeed;
    }
    else 
    {
      RobotActualSpeed = RobotNormalSpeed;
      RobotActualTurnSpeed = RobotNormalTurnSpeed;
    }

    TurnSpeed = RightStick.getX();
    if( TurnSpeed > 0 )
      TurnSpeed = Math.pow(TurnSpeed, 2);
    else 
      TurnSpeed = -Math.pow(TurnSpeed, 2);
    TurnSpeed = TurnSpeed * RobotActualTurnSpeed;
    if ((TurnSpeed < DeadBand) && (TurnSpeed > -DeadBand))
      TurnSpeed = 0.0;

    DriveSpeed = RightStick.getY();
    if( DriveSpeed > 0)
      DriveSpeed = Math.pow(DriveSpeed, 2);
    else
      DriveSpeed = -Math.pow(DriveSpeed, 2);
    DriveSpeed = DriveSpeed * RobotActualSpeed;
    if ((DriveSpeed < DeadBand) &&  (DriveSpeed > -DeadBand)) 
      DriveSpeed = 0.0;  

    myRobot.arcadeDrive(-DriveSpeed, TurnSpeed);
  }


  /*****************************************************************/
  /* BallInTake_Controls() - read leftstick to get speed           */
  /*****************************************************************/     
  public void BallInTake_Controls()
  {
    Double BallIntakeSpeed;

    BallIntakeSpeed = -LeftStick.getY();
    BallIntakeSpeed = BallIntakeSpeed * BallIntakeMaxSpeed;
    if ((BallIntakeSpeed < DeadBand) &&  (BallIntakeSpeed > -DeadBand)) 
      BallIntakeSpeed = 0.0;      
   
    BallIntakeMotor.set(BallIntakeSpeed);
    SmartDashboard.putNumber("BallIntake", BallIntakeSpeed);
  }


  /*****************************************************************/
  /* Winch_Controls() - read leftstick button 3 to move up and     */
  /*                    leftstick button 2 to move down.           */
  /*****************************************************************/   
  public void Winch_Controls()
  {
    if( LeftStick.getRawButton(3) == true )
    { 
      m_Winch.set( -WinchMaxSpeed );
      WinchCounter++;
    }
    else if( LeftStick.getRawButton(2) == true )
    {
      m_Winch.set( WinchMaxSpeed );
      if( WinchCounter > 0)
        WinchCounter--;
    }
    else
      m_Winch.set( 0.0 );

    SmartDashboard.putNumber("WinchCount", WinchCounter);
  }

  /*****************************************************************/
  /* Move_Robot_Controller() -                                     */
  /*                                                               */
  /*****************************************************************/   
  public void Move_Robot_Controller()
  {
    switch ( robot_move_state )
    {
      case idle:
        break;
      
      case set_up_move:
        robot_timer.reset();
        robot_timer.start();
        myRobot.arcadeDrive(-auto_robot_move_speed, 0.0);
        robot_move_state = Robot_Move_States.wait_for_move_complete;
        break;
     
      case wait_for_move_complete:
        if( robot_timer.get() > auto_robot_move_time )
        {
          myRobot.arcadeDrive(0.0, 0.0);         // Stop the robot
          robot_timer.stop();                    // Stop the timer
          robot_move_state = Robot_Move_States.finished;
        }
        else 
          myRobot.arcadeDrive(-auto_robot_move_speed, 0.0);
        break;
      
      case finished:
        break;

      default: 
        robot_move_state = Robot_Move_States.idle;
        break;

    }
  }
}
