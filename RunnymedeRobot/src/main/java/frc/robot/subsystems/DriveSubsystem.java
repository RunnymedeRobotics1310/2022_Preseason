package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    // The motors on the left side of the drive.
    private final CANSparkMax leftMotors =
            new CANSparkMax(DriveConstants.LEFT_MOTOR_PORT, MotorType.kBrushless);

    // The motors on the right side of the drive.
    private final CANSparkMax rightMotors =
            new CANSparkMax(DriveConstants.RIGHT_MOTOR_PORT, MotorType.kBrushless);

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        leftMotors.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);
        rightMotors.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);

    }

    private AHRS navXGyro = new AHRS(Port.kMXP);

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

    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {
        leftMotors.set(leftSpeed);
        rightMotors.set(rightSpeed);
    }

    public double getHeading() {

        double angle = navXGyro.getAngle();
        angle = angle % 360;

        if (angle < 0) {
            angle += 360.0d;
        }

        return angle;
    }

    @Override
    public void periodic() {

        //        SmartDashboard.putNumber("Right Motor", rightMotors.get());
        //        SmartDashboard.putNumber("Left  Motor", leftMotors.get());
    }
}
