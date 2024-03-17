package frc.robot.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics. SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;


public class Swerve extends SubsystemBase{
    private final Pigeon2 pigeonGyro;
    
    private SwerveDriveOdometry swerveDriveOdometry;
    private SwerveModule[] mSwerveMods;

    private Field2d field; 

    public Swerve(){
        pigeonGyro = new Pigeon2(Constants.SwerveConstants.pigeonID, "CANivore"); 
        pigeonGyro.clearStickyFaults();
        pigeonGyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
        new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
        new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
        new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        swerveDriveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositionArray());

        field = new Field2d();
      }

public SwerveModulePosition[] getPositionArray(){
    SwerveModulePosition[] array = {
        mSwerveMods[0].getModulePosition(), mSwerveMods[1].getModulePosition(), mSwerveMods[2].getModulePosition(), mSwerveMods[3].getModulePosition()};
    return array;
}

public void drive(
    Translation2d translation, 
    double rotation, 
    boolean fieldRelative, 
    boolean isOpenLoop, 
    double maxSpeed) 
    {
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), 
                    translation.getY(),
                    rotation,
                    getYaw()) 
                : new ChassisSpeeds(
                    translation.getX(), 
                    translation.getY(),
                    rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
        
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveDriveOdometry.getPoseMeters();
  }


  public void resetOdometry(Pose2d pose){
    swerveDriveOdometry.resetPosition(getYaw(), getPositionArray(), pose);
  }

public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroGyro() {
    pigeonGyro.setYaw(0);
  }

  public Rotation2d getYaw(){
    return (Constants.SwerveConstants.invertGyro)
        ? Rotation2d.fromDegrees(360.0 - pigeonGyro.getYaw().getValueAsDouble()) 
        : Rotation2d.fromDegrees(pigeonGyro.getYaw().getValueAsDouble());
  }

  public void stopWheels()
  {
    mSwerveMods[0].stop();
    mSwerveMods[1].stop();
    mSwerveMods[2].stop();
    mSwerveMods[3].stop();
  }

  @Override
  public void periodic() {
    swerveDriveOdometry.update(getYaw(), getPositionArray());
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      
      SmartDashboard.putNumber(
        "Mod " + mod.moduleNumber + " Motor Temperature", mod.getMotor().getMotorTemperature());
    }

    SmartDashboard.putNumber("Robot Heading", Math.IEEEremainder(pigeonGyro.getYaw().getValueAsDouble(), 360));
    SmartDashboard.putNumber("Robot Heading Actual", pigeonGyro.getYaw().getValueAsDouble());
  }
}

    


