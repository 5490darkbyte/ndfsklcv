package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.drivetrain.Swerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.*;
import static frc.robot.Constants.*;

public class RobotContainer {
    //Create Xbox Controllers
    private final Joystick driveController = new Joystick(DRIVE_CONTROLLER_PORT);
    private final Joystick operatorController = new Joystick(OPERATOR_CONTROLLER_PORT);

    //Create Triggers for each button
    private final Trigger main_bX = new JoystickButton(driveController, 3);
    private final Trigger main_bA = new JoystickButton(driveController, 1);
    private final Trigger main_bB = new JoystickButton(driveController, 2);
    private final Trigger main_bY = new JoystickButton(driveController, 4);
    private final Trigger main_LBumper = new JoystickButton(operatorController, 5);


    private final Trigger op_bX = new JoystickButton(operatorController, 3);
    private final Trigger op_bA = new JoystickButton(operatorController, 1);
    private final Trigger op_bB = new JoystickButton(operatorController, 2);
    private final Trigger op_bY = new JoystickButton(operatorController, 4);
    private final Trigger op_LBumper = new JoystickButton(operatorController, 5);
    private final Trigger op_RBumper = new JoystickButton(operatorController, 6);


     //Button and Axis values for swerve controls
     private final int translationAxis = XboxController.Axis.kLeftY.value;
     private final int strafeAxis = XboxController.Axis.kLeftX.value;
     private final int rotationAxis = XboxController.Axis.kRightX.value;
     private final JoystickButton zeroGyro = new JoystickButton(driveController, XboxController.Button.kY.value);
     private final JoystickButton driverBumper = new JoystickButton(driveController, 6);
  

    //Declare Subsystem variables
    private Swerve swerveSubsystem;
    private Intake intakeSubsystem;
    private Shooter shooterSubsystem;
    private Arm armSubsystem;
    private Climber climberSubsystemL;
    private Climber climberSubsystemR;


    //Declare sequential command groups
    public TeleopSwerve DriveCommand;
    private SequentialCommandGroup shootNote;
    private SequentialCommandGroup intakeNote;
    private SequentialCommandGroup hiIntake;
    private SequentialCommandGroup climberUp;
    private SequentialCommandGroup ampDeposit;
    private SequentialCommandGroup feedNoteBack;
    private SequentialCommandGroup feedNoteForward;
    // private SequentialCommandGroup leftClimbUp;
    // private SequentialCommandGroup leftClimbDown;
    // private SequentialCommandGroup RightClimbUp;
    // private SequentialCommandGroup RightClimbDown;



    public SequentialCommandGroup shootNoteAuto;
    public SequentialCommandGroup autoDriveForward;
    public SequentialCommandGroup autoShootDrive;
    public SequentialCommandGroup autoAmpBlue;
    public SequentialCommandGroup autoAmpRed;


    public RobotContainer() {
        createSubsystems(); // Initialize all the subsystems
        createCommands(); // Initialize Command groups and add commands to them
        configureButtonBindings(); // Configure the button bindings
      }

    //Initialize subsystems
    private void createSubsystems() {
        swerveSubsystem = new Swerve();
        climberSubsystemL = new Climber(CLIMBER_WINCH_MOTOR_L);
        climberSubsystemR = new Climber(CLIMBER_WINCH_MOTOR_R);
        intakeSubsystem = new Intake(INTAKE_MOTOR, TOF_SENSOR_ID);
        shooterSubsystem = new Shooter(SHOOTER_LEFT, SHOOTER_RIGHT);
        armSubsystem = new Arm(ARM_MOTOR_LEFT, ARM_MOTOR_RIGHT); 
    }


    // Initialize Command groups and add commands to them
    private void createCommands() {
   
      //The default command will be automatically scheduled when no other commands are scheduled that require the subsystem.   
      swerveSubsystem.setDefaultCommand(
        new TeleopSwerve(
          swerveSubsystem, 
          () -> -driveController.getRawAxis(translationAxis), 
          () -> -driveController.getRawAxis(strafeAxis),
          () -> -driveController.getRawAxis(rotationAxis),
          () -> driveController.getRawButton(XboxController.Button.kLeftBumper.value),
          Constants.SwerveConstants.MaxSpeed, 1.0, 1.0));
          
      SmartDashboard.putNumber("Rotation after TeleopSwerve", driveController.getRawAxis(rotationAxis));



    
      //Put arm in shoot position, spin shooter at full power, unload intake into shooter (spin for 1.5 sec) while slightly lifting arm, stop shooter
      shootNote = new SequentialCommandGroup();  
      shootNote.addCommands(new InstantCommand(() -> armSubsystem.setPosition("SHOOT")));
      shootNote.addCommands(new InstantCommand(() -> shooterSubsystem.setPowers(1)));
      shootNote.addCommands(new UnloadCommand(intakeSubsystem, () -> operatorController.getRawButton(2), .4) //press B to cancel
        .alongWith(new InstantCommand(() -> armSubsystem.setPosition("POSTSHOOT"))));
      shootNote.addCommands(new StopShooterCommand(shooterSubsystem));
      shootNote.addCommands(new InstantCommand(() -> armSubsystem.setPosition("SHOOT")));

      //Put arm in intake position, spin intake until note in intake or B button pressed, put arm in shoot position
      intakeNote = new SequentialCommandGroup();
      intakeNote.addCommands(new InstantCommand(() -> armSubsystem.setPosition("INTAKE")));
      intakeNote.addCommands(new WaitCommand(1.8));
      intakeNote.addCommands(new IntakeCommand(intakeSubsystem, () -> operatorController.getRawButton(2), 0.5)); //press B to cancel
      intakeNote.addCommands(new InstantCommand(() -> armSubsystem.setPosition("SHOOT")));
      
      /* 
      //Put arm in high intake position, spin intake, put arm in shoot position
      hiIntake = new SequentialCommandGroup();
      hiIntake.addCommands(new InstantCommand(() -> armSubsystem.setPosition("HI_INTAKE")));
      hiIntake.addCommands(new IntakeCommand(intakeSubsystem, () -> operatorController.leftTrigger().getAsBoolean(), 0.4));
      hiIntake.addCommands(new InstantCommand(() -> armSubsystem.setPosition("SHOOT")));
      */

      //Put arm in amp position, spin intake for 1.5 seconds, put arm in shoot position
      ampDeposit = new SequentialCommandGroup();
      ampDeposit.addCommands(new InstantCommand(() -> armSubsystem.setPosition("AMP")));
      ampDeposit.addCommands(new WaitCommand(2.0));
      ampDeposit.addCommands(new UnloadCommand(intakeSubsystem, () -> operatorController.getRawButton(2), 0.4)); //press B to cancel
      ampDeposit.addCommands(new InstantCommand(() -> armSubsystem.setPosition("SHOOT")));

      
      //Put arm in climb position, climb with left climber
      // climberL = new SequentialCommandGroup();
      // climberL.addCommands(new InstantCommand(() -> armSubsystem.setPosition("CLIMB")));
      // climberL.addCommands(new ClimberCommand(climberSubsystemL,() -> operatorController.getRawButton(3)));

      // //Put arm in climb position, climb with right climber
      // climberR = new SequentialCommandGroup();
      // climberR.addCommands(new InstantCommand(() -> armSubsystem.setPosition("CLIMB")));
      // climberR.addCommands(new ClimberCommand(climberSubsystemR, () -> operatorController.getRawButton(2)));

      
      climberUp = new SequentialCommandGroup();
      climberUp.addCommands(new ClimberCommand(climberSubsystemR, -1.0, () -> main_LBumper.getAsBoolean()));
      climberUp.addCommands(new ClimberCommand(climberSubsystemR, -1.0, () -> main_LBumper.getAsBoolean()));
      climberUp.addCommands(new ClimberCommand(climberSubsystemL, 1.0, () -> main_LBumper.getAsBoolean()));
      climberUp.addCommands(new ClimberCommand(climberSubsystemL, 1.0, () -> main_LBumper.getAsBoolean()));

      //Run intake in reverse at .2 power for .025 seconds
      feedNoteBack = new SequentialCommandGroup();
      feedNoteBack.addCommands(new InstantCommand(() -> intakeSubsystem.setPower(-.2)));
      feedNoteBack.addCommands(new WaitCommand(0.025));
      feedNoteBack.addCommands(new InstantCommand(() -> intakeSubsystem.setPower(0.0)));

      //Run intake at .2 power for .025 seconds
      feedNoteForward = new SequentialCommandGroup();
      feedNoteForward .addCommands(new InstantCommand(() -> intakeSubsystem.setPower(0.2)));
      feedNoteForward .addCommands(new WaitCommand(0.025));
      feedNoteForward .addCommands(new InstantCommand(() -> intakeSubsystem.setPower(0.0)));


      shootNoteAuto = new SequentialCommandGroup();
      shootNoteAuto.addCommands(new InstantCommand(() -> shooterSubsystem.setPowers(1)));
      shootNoteAuto.addCommands(new UnloadCommand(intakeSubsystem, () -> operatorController.getRawButton(2), .4) //press B to cancel
        .alongWith(new InstantCommand(() -> armSubsystem.setPosition("POSTSHOOT"))));
      shootNoteAuto.addCommands(new StopShooterCommand(shooterSubsystem));


      /*
      //Auto command to drive robot forward
      autoDriveForward = new SequentialCommandGroup();
      autoDriveForward.addCommands(new TeleopSwerve(swerveSubsystem, () -> 0.5, () -> 0.0, () -> 0.0, () -> false, Constants.SwerveConstants.MaxSpeed, 1.0, 1.0));
      autoDriveForward.addCommands(new WaitCommand(2));
      autoDriveForward.addCommands(new TeleopSwerve(swerveSubsystem, () -> 0.0, () -> 0.0, () -> 0.0, () -> false, Constants.SwerveConstants.MaxSpeed, 1.0, 1.0));

       
      OTHER AUTO OPTIONS

      //Put arm in shoot position, spin shooter, unload intake into shooter, stop shooter
      shootNoteAuto = new SequentialCommandGroup();
      shootNoteAuto.addCommands(new InstantCommand(() -> armSubsystem.setPosition("SHOOT")));
      shootNoteAuto.addCommands(new InstantCommand(() -> shooterSubsystem.setPID(Units.rotationsPerMinuteToRadiansPerSecond(4500.0))));
      shootNoteAuto.addCommands(new WaitCommand(1.5));
      shootNoteAuto.addCommands(new UnloadCommand(intakeSubsystem, () -> false));
      shootNoteAuto.addCommands(new StopShooterCommand(shooterSubsystem));
      */
      
      //Run command to shoot note, drive forward
      // autoShootDrive = new SequentialCommandGroup();
      // autoShootDrive.addCommands(shootNoteAuto);
      // //Try this out
      // autoShootDrive.addCommands(new TeleopSwerve(swerveSubsystem, () -> -0.5, () -> 0.0, () -> 0.0, () -> false, Constants.SwerveConstants.MaxSpeed, 1.0, 1.0));
      // autoShootDrive.addCommands(new WaitCommand(2));
      // autoShootDrive.addCommands(new TeleopSwerve(swerveSubsystem, () -> 0.0, () -> 0.0, () -> 0.0, () -> false, Constants.SwerveConstants.MaxSpeed, 1.0, 1.0));

      /*
      //Run command to shoot note, drive to amp?
      autoAmpBlue = new SequentialCommandGroup();
      autoAmpBlue.addCommands(shootNoteAuto);
      //autoAmpBlue.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.4, 0.4), driveTrainSubsystem));
      autoAmpBlue.addCommands(new WaitCommand(0.5));
      //autoAmpBlue.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(-0.65, 0.65), driveTrainSubsystem));
      autoAmpBlue.addCommands(new WaitCommand(0.7));
      //autoAmpBlue.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.3, 0.3), driveTrainSubsystem));
      autoAmpBlue.addCommands(new WaitCommand(2.5));
      //autoAmpBlue.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.0, 0.0), driveTrainSubsystem));

      //Run command to shoot note, drive to amp?
      autoAmpRed = new SequentialCommandGroup();
      autoAmpRed.addCommands(shootNoteAuto);
      //autoAmpRed.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.4, 0.4), driveTrainSubsystem));
      autoAmpRed.addCommands(new WaitCommand(0.5));
      //autoAmpRed.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.65, -0.65), driveTrainSubsystem));
      autoAmpRed.addCommands(new WaitCommand(.9));
      //autoAmpRed.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.3, 0.3), driveTrainSubsystem));
      autoAmpRed.addCommands(new WaitCommand(2.5));
      //autoAmpRed.addCommands(new InstantCommand(() -> driveTrainSubsystem.drive(0.0, 0.0), driveTrainSubsystem));
      */
  }

  private void configureButtonBindings(){
    //Driver Buttons
    zeroGyro.onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));

    driverBumper.whileTrue(
      new TeleopSwerve(
          swerveSubsystem,
          () -> -driveController.getRawAxis(translationAxis),
          () -> -driveController.getRawAxis(strafeAxis),
          () -> -driveController.getRawAxis(rotationAxis),
          () -> driveController.getRawButton(XboxController.Button.kLeftBumper.value),
            2.5, 4.0, 3));
    

    main_bY.whileTrue(climberUp);
    
    //Operator Buttons

    //Press A to staTrt. Moves arm to intake position. Spins intake. Moves arm to Shoot position.
    //Cancel intake spinning with B (or cancels when TOF sensor is triggered)
    op_bA.onTrue(intakeNote);

    //Press Y to Start. Moves arm to shoot position. Feeds into Shooter. Spins shooter at full speed.
    //Cancel intake spinning with B
    op_bY.onTrue(shootNote);

    //Press X to start. Moves arm to amp deposit position. Spins intake backwards. Moves arm to Shoot position.
    //Cancel intake spinning with B
    op_bX.onTrue(ampDeposit);

    //Inch intake slightly back (Left Bumper) and slightly forward (Right Bumper)
    op_LBumper.onTrue(feedNoteBack);
    op_RBumper.onTrue(feedNoteForward);
  }


  public Arm getArm(){
    return armSubsystem;
}
    
}