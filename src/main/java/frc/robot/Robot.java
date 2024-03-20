// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.config.CTREConfigs;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //Names for autos
  private static final String kDoNothing = "Do Nothing";
  private static final String kShootNoteAuto = "Shoot Note";
  private static final String kShootAndBack = "Shoot Note and Back";
  
  //Declare variable for storing selected auto name
  private String m_autoSelected;
  
  //Create a sendable chooser
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Declare autonomous command
  private Command m_autonomousCommand;
  
  //Declare robot container variable
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //Set options for the sendable chooser
    m_chooser.setDefaultOption("Do Nothing", kDoNothing);
    m_chooser.addOption("Shoot Note", kShootNoteAuto);
    m_chooser.addOption("Shoot Note and Back", kShootAndBack);


    //Put the auto choices on the smart dashboard
    SmartDashboard.putData("Auto Choices", m_chooser);
    
    //Start camera feed
    final UsbCamera camera = CameraServer.startAutomaticCapture();

    //Initialize robot container object
    m_robotContainer = new RobotContainer();

  }



  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. 
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    //Initialize the string of the selected auto
    m_autoSelected = m_chooser.getSelected();
    //Print which auto was selected
    System.out.println("Auto selected: " + m_autoSelected);
    
    //set whichever auto command was chosen
    switch (m_autoSelected) {
      case kShootNoteAuto:
        m_autonomousCommand = m_robotContainer.shootNoteAuto;
        break;
      case kShootAndBack:
        m_autonomousCommand = m_robotContainer.fireAndBack;
    }
    // schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}


  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Constantly trying to get arm to current setpoint
    m_robotContainer.getArm().setArmAngle(m_robotContainer.getArm().getTargetAngle());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
