package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climber;
import static frc.robot.Constants.*;


public class ClimberCommand extends Command {
  private final Climber climberSubsystem;

  private final BooleanSupplier cancel;

  private final double power;

  public ClimberCommand(Climber climberSubsystem, double power, BooleanSupplier cancel) {
    this.climberSubsystem = climberSubsystem;
    this.cancel = cancel;
    this.power = power;
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
