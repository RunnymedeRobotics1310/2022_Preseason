package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

	// The motors on the left side of the drive.
	private final MotorControllerGroup leftMotors = new MotorControllerGroup(
			new PWMSparkMax(DriveConstants.LEFT_MOTOR_PORT));

	// The motors on the right side of the drive.
	private final MotorControllerGroup rightMotors = new MotorControllerGroup(
			new PWMSparkMax(DriveConstants.RIGHT_MOTOR_PORT));

	// The robot's drive
	private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

	// The left-side drive encoder
	private final Encoder leftEncoder = new Encoder(DriveConstants.LEFT_ENCODER_PORTS[0],
			DriveConstants.LEFT_ENCODER_PORTS[1], DriveConstants.LEFT_ENCODER_REVERSED);

	// The right-side drive encoder
	private final Encoder rightEncoder = new Encoder(DriveConstants.RIGHT_ENCODER_PORTS[0],
			DriveConstants.RIGHT_ENCODER_PORTS[1], DriveConstants.RIGHT_ENCODER_REVERSED);

	// The gyro sensor
	private final Gyro gyro = new ADXRS450_Gyro();

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		// We need to invert one side of the drivetrain so that positive voltages
		// result in both sides moving forward. Depending on how your robot's
		// gearbox is constructed, you might have to invert the left side instead.
		rightMotors.setInverted(true);

		// Sets the distance per pulse for the encoders
		leftEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
		rightEncoder.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE);
	}

	public void setMotorSpeeds(double leftSpeed, double rightSpeed) {
		leftMotors.set(leftSpeed);
		rightMotors.set(rightSpeed);
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() {
		leftEncoder.reset();
		rightEncoder.reset();
	}

	/**
	 * Gets the average distance of the two encoders.
	 *
	 * @return the average of the two encoder readings
	 */
	public double getAverageEncoderDistance() {
		return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
	}

	/**
	 * Gets the left drive encoder.
	 *
	 * @return the left drive encoder
	 */
	public Encoder getLeftEncoder() {
		return leftEncoder;
	}

	/**
	 * Gets the right drive encoder.
	 *
	 * @return the right drive encoder
	 */
	public Encoder getRightEncoder() {
		return rightEncoder;
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		gyro.reset();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from 0 to 360
	 */
	public double getHeading() {

		double heading = gyro.getAngle();

		if (DriveConstants.GYRO_REVERSED) {
			heading *= -1.0;
		}

		heading = heading % 360; // Normalize the heading angle to standard compass headings.

		if (heading < 0) {
			heading += 360;
		}

		return heading;
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
	}
}
