package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class StopShooterCommand extends Command {
  Shooter shooter;
  Intake intake;

  public StopShooterCommand(Shooter shooter) {
    addRequirements(shooter);
    shooter.setPowers(0);

    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setPowers(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
