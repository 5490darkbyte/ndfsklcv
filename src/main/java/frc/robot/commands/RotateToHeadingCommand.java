package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.drivetrain.Swerve;
 
public class RotateToHeadingCommand extends PIDCommand {
    private final Swerve swerveSubsystem;
    private final double targetHeading;
    private final double toleranceDegrees;

    public RotateToHeadingCommand(double targetHeading, Swerve swerveSubsystem, double toleranceDegrees) {
        super(
            new PIDController(1, 0, 0),
            swerveSubsystem::getHeading,
            targetHeading,
            output -> swerveSubsystem.drive(new Translation2d(), output, true, true, 1.0),
            swerveSubsystem
        );
        this.swerveSubsystem = swerveSubsystem;
        this.targetHeading = targetHeading;
        this.toleranceDegrees = toleranceDegrees;
        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(5);
    }

    @Override
    public void initialize() {
        super.initialize();
        getController().reset();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerveSubsystem.stopWheels();
    }


    @Override
    public boolean isFinished() {
        double currentHeading = swerveSubsystem.getHeading();
        return Math.abs(currentHeading - targetHeading) < toleranceDegrees;
    }

}