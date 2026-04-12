package org.redtierobotics.robot2027.subsystems.drive;

import static edu.wpi.first.units.Units.Seconds;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import org.redtierobotics.lib.control.ControlConstants.PIDVConstants;
import org.redtierobotics.lib.control.ControlConstants.ProfiledPIDVConstants;
import org.redtierobotics.lib.subsystembases.SubsystemConstants;
import org.redtierobotics.robot2027.subsystems.drive.ctre.TestCtreDriveConstants;

public final class DriveConstants extends SubsystemConstants {
	public static final double EPSILON_TRANSLATION = 0.015; // cm
	public static final double EPSILON_ROTATION = Units.Degrees.of(1.5).in(Units.Radians);

	// Maximums
	public static final double MAX_SPEED = Units.MetersPerSecond.of(4.0).in(Units.MetersPerSecond);
	public static final double MAX_ACCEL =
			Units.MetersPerSecondPerSecond.of(9.0).in(Units.MetersPerSecondPerSecond);
	public static final double MAX_ROTATION_SPEED =
			Units.RotationsPerSecond.of(1.5).in(Units.RadiansPerSecond);
	public static final double MAX_ROTATION_ACCEL =
			Units.RotationsPerSecondPerSecond.of(4.0).in(Units.RadiansPerSecondPerSecond);

	// Swerve dimensions
	public static final double TRACK_WIDTH = Units.Inches.of(24).in(Units.Meters);
	public static final double WHEEL_BASE = Units.Inches.of(24).in(Units.Meters);
	public static final double WHEEL_DIAMETER =
			2 * TestCtreDriveConstants.kWheelRadius.in(Units.Meters);
	public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

	// Stability constants
	public static final double MAX_VELOCITY_STABLE = 10; // degrees per second
	public static final double MAX_PITCH_STABLE = 5; // degrees
	public static final LinearVelocity MAX_SPEED_SCORING_TRANSLATION =
			Units.Centimeters.of(15.0).per(Units.Seconds);
	public static final AngularVelocity MAX_ROTATION_SPEED_SCORING =
			Units.Degrees.of(7.0).per(Units.Seconds); // oh god
	public static final Time POSE_RESET_PREVENTION_TIME = Units.Seconds.of(0.15);

	// Trajectory and snap constants
	public static final PIDVConstants TRANSLATION_CONSTANTS = new PIDVConstants(10, 0.0, 0.1);
	public static final PIDVConstants ROTATION_CONSTANTS = new PIDVConstants(16.0, 0.0, 0.1);
	public static final ProfiledPIDVConstants PROFILED_ROTATION_CONSTANTS =
			new ProfiledPIDVConstants(
					16.0, 0.0, 0.1, new TrapezoidProfile.Constraints(MAX_ROTATION_SPEED, MAX_ROTATION_ACCEL));
	public static final double ACCELERATION_CONSTANT = 0.1;

	private static final APConstraints constraints =
			new APConstraints().withVelocity(MAX_SPEED).withAcceleration(MAX_ACCEL).withJerk(3.0);
	private static final APProfile profile =
			new APProfile(constraints)
					.withErrorXY(Units.Meters.of(EPSILON_TRANSLATION))
					.withErrorTheta(Units.Radians.of(EPSILON_ROTATION))
					.withBeelineRadius(Units.Centimeters.of(8));
	public static final Autopilot autoPilot = new Autopilot(profile);

	public static final double AUTO_ALIGN_TIMEOUT = 0.5;

	public static final Time DEBOUNCE_TIME = Seconds.of(0.05);
	public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.02);
}
