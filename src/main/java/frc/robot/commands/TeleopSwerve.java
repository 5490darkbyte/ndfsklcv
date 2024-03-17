package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.drivetrain.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command{
    //Declare swerve drive object to be used
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private SlewRateLimiter translationSlewRateLimiter;
    private SlewRateLimiter strafeSlewRateLimiter;
    private SlewRateLimiter rotationSlewRateLimiter;

    private double maxSpeed;
    private double divideSpin;
    private double divideSpeed;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, 
    DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, double maxSpeed, double divideSpin, double divideSpeed) {
    translationSlewRateLimiter = new SlewRateLimiter(0.7); //originally 2.5
    strafeSlewRateLimiter = new SlewRateLimiter(0.7);
    rotationSlewRateLimiter = new SlewRateLimiter(0.7);
  
    this.s_Swerve = s_Swerve;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.maxSpeed = maxSpeed;
    this.divideSpin = divideSpin;
    this.divideSpeed = divideSpeed;

    addRequirements(s_Swerve);
  }

  @Override public void initialize() {}

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal =
        translationSlewRateLimiter.calculate(
            MathUtil.applyDeadband(-translationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    double strafeVal =
        strafeSlewRateLimiter.calculate(
            MathUtil.applyDeadband(-strafeSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    SmartDashboard.putNumber("Second Rotation Sup", rotationSup.getAsDouble());
    double rotationVal =
        rotationSlewRateLimiter.calculate(
            MathUtil.applyDeadband(-rotationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
       

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(maxSpeed/divideSpeed),
        rotationVal * (Constants.SwerveConstants.maxAngularVelocity/divideSpin),
        !robotCentricSup.getAsBoolean(),
        true, maxSpeed);

        SmartDashboard.putNumber("Rotation Sup", rotationVal);
        SmartDashboard.putNumber("Translation Sup", translationSup.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.stopWheels();
  }

  @Override
  public boolean isFinished() {
      return false;
  }

}
