package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import static frc.robot.Constants.*;


public class ClimberCommand extends Command {
  private final Climber climberSubsystem;

  private BooleanSupplier cancel;

  private DoubleSupplier powerSup;
  private double power;

  public ClimberCommand(Climber climberSubsystem, DoubleSupplier powerSup, BooleanSupplier cancel) {
    this.climberSubsystem = climberSubsystem;
    this.cancel = cancel;
    this.powerSup = powerSup;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberSubsystem.setWinchPower(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    power = MathUtil.applyDeadband(powerSup.getAsDouble(), Constants.SwerveConstants.stickDeadband);
    climberSubsystem.setWinchPower(power*WINCH_POWER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.setWinchPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cancel.getAsBoolean();
  }

}
