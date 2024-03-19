package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
    public boolean climberisBrake;
    public double climberCurrent;
    public double angularPos;
    public double appliedPower;

    private double modulatedPower;


    private TalonFX climberWinchMotor;
    private boolean isBrake;




  public Climber(int IDclimberWinchMotor, boolean isInverted) {
    climberWinchMotor = new TalonFX(IDclimberWinchMotor, "CANivore");
    climberWinchMotor.setNeutralMode(NeutralModeValue.Brake);
    isBrake = true;
    climberWinchMotor.setInverted(isInverted);
  }

  @Override
  public void periodic() {
    updateInputs();

  }

  public void setWinchPower(double power) {
    if(this.angularPos > 95.0 && power > 0){
      modulatedPower = 0.0;
    }
    else if(this.angularPos < -1.0 && power < 0){
      modulatedPower = 0.0;
    } 
    else 
      modulatedPower = power;
    climberWinchMotor.set(modulatedPower);
  }

  private void updateInputs(){
    climberisBrake = isBrake;
    climberCurrent = climberWinchMotor.getSupplyCurrent().getValueAsDouble();
    angularPos = climberWinchMotor.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("Climber Pos " + climberWinchMotor.getDeviceID(), angularPos);
    appliedPower = climberWinchMotor.getMotorVoltage().getValueAsDouble();
  }

  public void resetSensor(){
    climberWinchMotor.setPosition(0.0);
  }

  // public boolean isReady(){
  //   return (angular)
  // }

}
