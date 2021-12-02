package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

	private final DriveSubsystem driveSubsystem;
	private final PS4Controller driverController;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param driveSubsystem The subsystem used by this command.
	 */
	public DefaultDriveCommand(PS4Controller driverController, DriveSubsystem driveSubsystem) {

		this.driverController = driverController;
		this.driveSubsystem = driveSubsystem;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(driveSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		
		double leftWheel = driverController.getRawAxis(1) + 0.5*driverController.getRawAxis(0);
		double rightWheel = driverController.getRawAxis(1) - 0.5*driverController.getRawAxis(0);
		boolean boost = false;

		if (driverController.getRawButton(5) || driverController.getRawButton(6)){
			boost = true;
		}

		if (!boost) {
			driveSubsystem.setMotorSpeeds(leftWheel/2, rightWheel/2);
		} else {
			driveSubsystem.setMotorSpeeds(leftWheel, rightWheel);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
