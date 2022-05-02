package frc4388.robot.commands.StorageCommands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.BoomBoom;
import frc4388.robot.subsystems.Storage;
import frc4388.robot.subsystems.Turret;

public class ManageStorage extends CommandBase {

  // subsystems
  private Storage storage;
  private BoomBoom drum;
  private Turret turret;

  private Alliance alliance;
  private boolean rightColor;

  /** Creates a new ManageStorage. */
  public ManageStorage(Storage storage, BoomBoom drum, Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.storage = storage;
    this.drum = drum;
    this.turret = turret;

    rightColor = true;

    addRequirements(this.storage, this.drum, this.turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    checkColor();
    
    if (rightColor) {
      this.storage.manageStorage();
    } else {

      // * CommandScheduler.getInstance().schedule(new ExampleCommand());
      // * new ExampleCommand().schedule();
      // * new ExampleCommand().execute(); (accompanied by initialize and onFinished)
      
      new SpitOutWrongColor(this.storage, this.drum, this.turret); // ? is this how you run a command inside a command
    }
  }

  private void checkColor() {
    // this.alliance = this.storage.getColor();
    // rightColor = this.alliance.equals(Robot.alliance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
