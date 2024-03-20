// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class Arm extends SubsystemBase {

  //arm variables
  public boolean isBrake;
  public double current;
  public double currentAngle;
  public double velocity;
  public double targetAngle;
  public double appliedPower;
  public double relativePos_l;
  public double relativePos_r;

  private double ARM_MAX_ANGLE = Constants.ArmConstants.ArmMaxDegrees;
  private double ARM_MIN_ANGLE = Constants.ArmConstants.ArmMinDegrees;

  //Motor controllers for arm
  private CANSparkMax armMotor_l;
  private CANSparkMax armMotor_r;

  //WPILib ArmFeedForward for controlling arm
  private ArmFeedforward feedforward;

   //WPILib PID for controlling arm
   private PIDController armPIDController;
  
  //Encoder for arm absolute position and velocity*
  private DutyCycleEncoder ThroughBoreEncoder;

  private Climber lClimber;
  private Climber rClimber;
  public Arm(int IDleftMotor, int IDrightMotor, Climber lClimber, Climber rClimber) {   
    
    feedforward = new ArmFeedforward(0.0, .12, 0.0);
    armPIDController = new PIDController(0.012, 0.0, 0.0);

    this.lClimber = lClimber;
    this.rClimber = rClimber;
    //Initialize arm motor controller objects
    armMotor_l = new CANSparkMax(IDleftMotor, CANSparkLowLevel.MotorType.kBrushless);
    armMotor_r = new CANSparkMax(IDrightMotor, CANSparkLowLevel.MotorType.kBrushless);

    //Arm motors originally set to brake when in idle
    armMotor_l.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armMotor_r.setIdleMode(CANSparkMax.IdleMode.kBrake);
    isBrake = true;

    //Whether to invert the right motor (motors will spin in opposite directions)
    armMotor_r.setInverted(false);

    //PID constant settings
    //Keep the arm from trying to go through/under the superstructure to get from one position to another
    armPIDController.disableContinuousInput();

    //Allow PID to be at setpoint withing 1 degree of actual setpoint
    armPIDController.setTolerance(1.0);

    //Set the left motor to follow the right motors movements, but inverted
    armMotor_l.follow(armMotor_r, true);

    //Initialize arm through bore encoder
    ThroughBoreEncoder = new DutyCycleEncoder(0); //on DIO pin 0
    ThroughBoreEncoder.setDistancePerRotation(360.0);
    ThroughBoreEncoder.reset();

    //Set current limit for arm motors
    armMotor_r.setSmartCurrentLimit(Constants.ArmConstants.SmartCurrentLimit);
    armMotor_l.setSmartCurrentLimit(Constants.ArmConstants.SmartCurrentLimit);
  }

  @Override
  public void periodic() {
    updateInputs();
    SmartDashboard.putNumber("Arm Encoder getDistance", ThroughBoreEncoder.getDistance());
    SmartDashboard.putNumber("Arm Current (sum of both motors)", current);
    SmartDashboard.putNumber("Right Arm Motor Applied Power", appliedPower);
  }

  public void setArmAngle(double angle) {
    if (angle > ARM_MAX_ANGLE) {
      System.out.println("Arm angle is too big");
    } else if (angle < ARM_MIN_ANGLE) {
      System.out.println("Arm angle is too small");
    }
    else {
      if(lClimber.getAngle() > 10.0 || rClimber.getAngle() > 10.0){
      }
      else
        setAngle(angle);
    }

  }

  public void setPosition(String position) {
    if (position.equals("INTAKE")) {
      targetAngle = 226; 
    } else if (position.equals("AMP")) {
      targetAngle = 110;
    } else if (position.equals("SHOOT")) {
      targetAngle = 0;
    }else if (position.equals("HI_INTAKE")){
      targetAngle = 151;
    }
      else if (position.equals("POSTSHOOT")){
        targetAngle = 10;
      }
    }
  

  public double getAngle() {
    return currentAngle;
  }

  public double getTargetAngle(){
    return targetAngle;
  }



  private void updateInputs(){
    current = armMotor_r.getOutputCurrent()+ armMotor_l.getOutputCurrent();
    currentAngle = ThroughBoreEncoder.getDistance(); 
    appliedPower = armMotor_r.getAppliedOutput();
    relativePos_l = armMotor_l.getEncoder().getPosition();
    relativePos_r = armMotor_r.getEncoder().getPosition();
}


  private void setBreak(boolean brake){
    this.isBrake = brake;
    if (isBrake) {
        armMotor_l.setIdleMode(CANSparkMax.IdleMode.kBrake);
    } else {
        armMotor_l.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }
  }
  
  private void setAngle(double angle){
    targetAngle = angle;
    double maxVoltage = 3.4;
    double pidOutput = armPIDController.calculate(currentAngle, targetAngle);
    double feedforwardangle = feedforward.calculate(targetAngle-10, .15); 
    double motorVoltage = (pidOutput * maxVoltage);
    armMotor_r.setVoltage(MathUtil.clamp(motorVoltage+feedforwardangle, -8.0, 8.0)); //Neos can go to 12V
  }
}

