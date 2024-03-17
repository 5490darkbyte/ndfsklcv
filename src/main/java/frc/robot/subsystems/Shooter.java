package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import com.revrobotics.CANSparkBase.ControlType;

public class Shooter extends SubsystemBase {
    public double leftCurent;
    public double rightCurent;
    public double leftVelocity;
    public double rightVelocity;
    public double leftPosition;
    public double rightPosition;
    public double leftAppliedPower;
    public double rightApppliedPower;
    public double setPoint = 0;


    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private SparkPIDController rightPIDController;

    private RelativeEncoder rightMotorEncoder;

    private double p = 0.0009;
    private double i = 0.0;
    private double d = 0.0;

    public Shooter(int IDleftMotor, int IDrightMotor) {
        leftMotor = new CANSparkMax(IDleftMotor, MotorType.kBrushless);
        rightMotor = new CANSparkMax(IDrightMotor, MotorType.kBrushless);

        leftMotor.follow(rightMotor, true);

        rightPIDController = rightMotor.getPIDController();
        rightMotorEncoder = rightMotor.getEncoder();

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        rightPIDController.setOutputRange(-1, 1);

        leftMotor.setSmartCurrentLimit(19);
        rightMotor.setSmartCurrentLimit(19);

        rightMotorEncoder.setVelocityConversionFactor(SHOOTER_RATIO*2*Math.PI/60);
        rightMotorEncoder.setPositionConversionFactor(SHOOTER_RATIO*2*Math.PI);

        rightPIDController.setP(p);
        rightPIDController.setI(i);
        rightPIDController.setD(d);

        rightPIDController.setFeedbackDevice(rightMotorEncoder);

        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);

    }
  @Override
  public void periodic() {
    //Consistently update values from shooter motors controller and encoder
    updateInputs();
  }


  public void setPID(double speed){

  }


  private void updateInputs(){
    rightCurent = rightMotor.getOutputCurrent();
    rightVelocity = rightMotorEncoder.getVelocity();
    rightPosition = rightMotorEncoder.getPosition();
    rightApppliedPower = rightMotor.getAppliedOutput();
  }

  public void setPowers(double rightPower) {
    rightMotor.set(rightPower);
  }

  public void setRPS(double rightRPS) {
    rightPIDController.setReference(rightRPS, ControlType.kVelocity);
    this.setPoint = rightRPS;
  }
}
