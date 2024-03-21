package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.drivetrain.Swerve;
 
public class RotateToHeadingCommand extends PIDCommand {
    private final Swerve swerveSubsystem;

    public RotateToHeadingCommand(double targetHeading, Swerve swerveSubsystem) {
        super(
            new PIDController(1, 0, 0),
            swerveSubsystem::getHeading,
            targetHeading,
            output -> swerveSubsystem.drive(new Translation2d(), output, true, true, 1.0),
            swerveSubsystem
        );
        this.swerveSubsystem = swerveSubsystem;
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(5);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}