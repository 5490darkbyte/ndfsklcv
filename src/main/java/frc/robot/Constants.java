package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.lib.config.SwerveModuleConstants;



public class Constants {
    // Xbox Controllers Port Indexes
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
   
    //PDP CAN ID
    public static final int PDP_CAN_ID = 0;

    // TOF Sensor Info
    public static final int TOF_SENSOR_ID = 14; //Will need to be changed
    public static final double NOTE_SENSOR_DISTANCE = 400.0; //Will need to be changed

    //Motor CAN IDs
    public static final int CLIMBER_WINCH_MOTOR_R = 21;
    public static final int CLIMBER_WINCH_MOTOR_L = 20;

    public static final int INTAKE_MOTOR = 19;

    public static final int ARM_MOTOR_LEFT = 15;
    public static final int ARM_MOTOR_RIGHT = 16;

    public static final int SHOOTER_LEFT = 17;
    public static final int SHOOTER_RIGHT = 18;

    //Arm CANcoder Offset

    //Speed of climber winch (between -1.0 and 1.0)
    public static final double WINCH_POWER = .5;

    // Ratios 
    // (get multipilied by 2*pi for position conversion factor)
    // (get multiplied by 2*pi/60 for velocity conversion factor)
    public static final double INTAKE_RATIO = (1.0/3.2727);
    public static final double SHOOTER_RATIO = (1.0/1.0);
    public static final double WINCH_RATIO = (1.0/48.0);



    public static final class SwerveConstants
    {
        //Deadband for Xbox controllers
        public static final double stickDeadband = 0.08;

        public static final int pigeonID = 13;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
    
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.5);
        public static final double wheelBase = Units.inchesToMeters(21.5);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;
    
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
    
        public static final double driveGearRatio = (8.14 / 1.0); // 6.75:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1
        

        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 40;
            
        /* Angle Motor PID Values */
        public static final double angleKP = 0.01;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKFF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;
        
        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 3; 
        public static final double maxAngularVelocity = 8; 

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final IdleMode driveNeutralMode = IdleMode.kCoast;

        /* Motor Inverts */
        public static final boolean driveInvert = true;
        public static final boolean angleInvert = false;

        /* Max Speeds */
        public static final double MaxSpeed = 3.7;
        public static final double turboMaxSpeed = 3.7;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;


        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
        public static final int driveMotorID = 3;
        public static final int angleMotorID = 4;
        public static final int canCoderID = 10;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(359.296875);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
        public static final int driveMotorID = 5;
        public static final int angleMotorID = 6;
        public static final int canCoderID = 11;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180-91.0546875);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 9;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(350.262);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
        public static final int driveMotorID = 7;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 12;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180+35);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

     public static final class ArmConstants
    {
        public static final double ArmMaxDegrees = 226.0;
        public static final double ArmMinDegrees = 0.0;
        public static final int SmartCurrentLimit  = 19;
    }
    
}
