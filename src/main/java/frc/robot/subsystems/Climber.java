package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
    public boolean climberisBrake;
    public double climberCurrent;
    public double angularPos;
    public double appliedPower;

    //*Motor needs to be changed*
    private CANSparkMax climberWinchMotor;

    private RelativeEncoder climberWinchMotorEncoder;

    private boolean isBrake;




  public Climber(int IDclimberWinchMotor) {
    
    climberWinchMotor = new CANSparkMax(IDclimberWinchMotor, MotorType.kBrushless);
    
    climberWinchMotorEncoder = climberWinchMotor.getEncoder();
   
    climberWinchMotor.setSmartCurrentLimit(40);
    climberWinchMotor.setIdleMode(IdleMode.kBrake);
    climberWinchMotor.setInverted(true);

    climberWinchMotorEncoder.setPositionConversionFactor(WINCH_RATIO * Math.PI * 2);
    climberWinchMotorEncoder.setVelocityConversionFactor(WINCH_RATIO * Math.PI * 2 / 60);
    climberWinchMotorEncoder.setPosition(0.0);
    isBrake = true;
   
  }

  @Override
  public void periodic() {
    updateInputs();

  }

  public void setWinchPower(double power) {
    climberWinchMotor.set(power);

  }

  private void updateInputs(){
    climberisBrake = isBrake;
    climberCurrent = climberWinchMotor.getOutputCurrent();
    appliedPower = climberWinchMotor.getAppliedOutput();
    angularPos = climberWinchMotorEncoder.getPosition();
  }

  private void setBreak(boolean brake){
    if (brake) {
        climberWinchMotor.setIdleMode(IdleMode.kBrake);
    } else {
        climberWinchMotor.setIdleMode(IdleMode.kCoast);
    }
    isBrake = brake;
}

}
