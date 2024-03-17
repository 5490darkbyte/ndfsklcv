package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.playingwithfusion.*;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;


public class Intake extends SubsystemBase {
    public boolean isBrake;
    public double curent;
    public double velocity;
    public boolean NoteSensed;
    public double position;
    public double appliedOutput;

    private CANSparkMax intakeMotor;
    private TimeOfFlight TOFSensor;
    private RelativeEncoder intakeMotorEncoder;


  public Intake(int IDintakeMotor, int TOFSensorID) {
    
    intakeMotor = new CANSparkMax(IDintakeMotor, MotorType.kBrushless);
    TOFSensor = new TimeOfFlight(TOFSensorID);
    TOFSensor.setRangingMode(RangingMode.Short, TOFSensorID);
    intakeMotorEncoder = intakeMotor.getEncoder();

    intakeMotor.setSmartCurrentLimit(100);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setInverted(true);

    intakeMotorEncoder.setPositionConversionFactor(INTAKE_RATIO * Math.PI * 2);
    intakeMotorEncoder.setVelocityConversionFactor(INTAKE_RATIO * Math.PI * 2 / 60);

    isBrake = false;
    

  }

  @Override
  public void periodic() {
    updateInputs();

  }

  public void setPower(double power) {
    intakeMotor.set(power);

  }


  private void updateInputs(){
    curent = intakeMotor.getOutputCurrent();
    velocity = intakeMotorEncoder.getVelocity();
    NoteSensed = TOFSensor.getRange() < NOTE_SENSOR_DISTANCE;
    position = intakeMotorEncoder.getPosition();
    appliedOutput = intakeMotor.getAppliedOutput();
  }

  public boolean isNoteSensed(){
    return NoteSensed;
  }

  private void setBrake(boolean brake) {
    if (brake) {
        intakeMotor.setIdleMode(IdleMode.kBrake);
    } else {
        intakeMotor.setIdleMode(IdleMode.kCoast);
    }
    isBrake = brake;
  }
}

