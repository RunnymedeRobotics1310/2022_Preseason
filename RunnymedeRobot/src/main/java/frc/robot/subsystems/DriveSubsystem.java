package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    // The motors on the left side of the drive.
    private final TalonSRX leftMotors =
            new TalonSRX(DriveConstants.LEFT_MOTOR_PORT);

    // The motors on the right side of the drive.
    private final TalonSRX rightMotors =
            new TalonSRX(DriveConstants.RIGHT_MOTOR_PORT);

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        leftMotors.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);
        rightMotors.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);

        leftMotors.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,  0);
        leftMotors.setSelectedSensorPosition(0, 0, 0);

        rightMotors.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,  0);
        rightMotors.setSelectedSensorPosition(0, 0, 0);

    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (leftMotors.getSelectedSensorPosition() + rightMotors.getSelectedSensorPosition()) / 2;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public double getLeftEncoder() {
        return leftMotors.getSelectedSensorPosition();
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public double getRightEncoder() {
        return rightMotors.getSelectedSensorPosition();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        leftMotors.setSelectedSensorPosition(0);
        rightMotors.setSelectedSensorPosition(0);
    }

    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {
        leftMotors.set(ControlMode.PercentOutput, leftSpeed);
        rightMotors.set(ControlMode.PercentOutput, rightSpeed);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Right Motor", rightMotors.getMotorOutputPercent());
        SmartDashboard.putNumber("Left  Motor", leftMotors.getMotorOutputPercent());
    }
}
