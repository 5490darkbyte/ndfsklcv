package frc.robot.lib.math;

import edu.wpi.first.math.geometry.Rotation2d; 
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class OnboardModuleState {
    
    public static SwerveModuleState optimizeSwerve(SwerveModuleState desiredState, Rotation2d currentState)
    {
        double targetAngle = scopeAngle(currentState.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double deltaAngles = targetAngle - currentState.getDegrees();
        if(Math.abs(deltaAngles) > 90)
        {
            targetSpeed = -targetSpeed;
            targetAngle = deltaAngles > 90 ? (targetAngle -= 180) : (targetAngle += 180);  
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle)); 
    }

    private static double scopeAngle(double currAngle, double targetAngle)
    {
        double lowerBound, upperBound;
        double lowerOffset = currAngle % 360; // Eliminate useless angles
        if (lowerOffset >= 0)
        {
            lowerBound = currAngle - lowerOffset;
            upperBound = currAngle + (360 - lowerOffset);
        }
        else 
        {
            upperBound = currAngle - lowerOffset; 
            lowerBound = currAngle - (360 + lowerOffset);
        }
        /* Loop that forces the target angle above the lower bound without changing absolute angle */
        while (targetAngle < lowerBound) { targetAngle += 360; }
        /* Loop that forces the target angle below the upper bound without changing absolute angle */
        while (targetAngle > upperBound) { targetAngle -= 360; }
        /* if DeltaAngle is greater than 180, invert it within the aboslute angle range */
        if ((targetAngle - currAngle) > 180) targetAngle -= 360;
        /* if DeltaAngle is less than 180, invert it within the aboslute angle range */
        else if ((targetAngle - currAngle) < -180) targetAngle += 360;
        return targetAngle;
    }
}
