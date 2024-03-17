package frc.robot.lib.config;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue; 
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import frc.robot.Constants;

//Set up a CANcoder configuration with the absolute sensor range from 0 to 1 (0 to 360) and the direction inverted
public final class CTREConfigs {
    public CANcoderConfiguration swerveCANcoderConfiguration; 

    public CTREConfigs()
    {
        swerveCANcoderConfiguration =  new CANcoderConfiguration();
        swerveCANcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCANcoderConfiguration.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderInvert;
    }
}
