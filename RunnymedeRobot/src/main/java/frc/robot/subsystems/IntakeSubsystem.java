package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class IntakeSubsystem extends SubsystemBase {

    // The motors on the top part of the intake thing.
    private final CANSparkMax intakeTopMotor = 
    		new CANSparkMax(DriveConstants.INTAKE_TOP_MOTOR_ADDRESS, MotorType.kBrushless);
    
    //The motors on the bottom part of the intake thing.
    private final CANSparkMax intakeBottomMotor =
    		new CANSparkMax(DriveConstants.INTAKE_BOTTOM_MOTOR_ADDRESS, MotorType.kBrushless);
  

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
    	//for carousel, this is unknown at the moment
        intakeTopMotor.setInverted(DriveConstants.INTAKE_TOP_MOTOR_REVERSED);//do we need a carousel motor reversed constant?
        intakeBottomMotor.setInverted(DriveConstants.INTAKE_BOTTOM_MOTOR_REVERSED);

    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return 0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public double getLeftEncoder() {
        return 0;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public double getRightEncoder() {
        return 0;
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
    }

    public void setMotorSpeed(double topSpeed, double bottomSpeed) {
        intakeTopMotor.set(topSpeed);
        intakeBottomMotor.set(bottomSpeed);
    }

    @Override
    public void periodic() {

        //        SmartDashboard.putNumber("Right Motor", rightMotors.get());
        //        SmartDashboard.putNumber("Left  Motor", leftMotors.get());
    }
}
