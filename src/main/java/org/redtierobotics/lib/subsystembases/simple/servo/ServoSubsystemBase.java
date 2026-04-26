package org.redtierobotics.lib.subsystembases.simple.servo;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.redtierobotics.lib.io.motor.MotorIO;
import org.redtierobotics.lib.io.motor.MotorInputs;
import org.redtierobotics.lib.subsystembases.simple.LoggedSubsystemBase;
import org.redtierobotics.lib.util.Util;

public abstract class ServoSubsystemBase extends LoggedSubsystemBase {
	protected MotorInputs inputs;
	protected MotorIO<?> io;
	protected ServoSubsystemConfig config;

	/**
	 * Constructs a subsystem with one servo motor
	 *
	 * @param <A> The servo type
	 * @param io The servo IO object
	 * @param inputs The servo loggable inputs object
	 * @param config Configs for the servo
	 */
	public <A extends MotorInputs & LoggableInputs> ServoSubsystemBase(
			MotorIO<A> io, A inputs, ServoSubsystemConfig config) {
		super();
		this.io = io;
		this.inputs = inputs;
		this.config = config;
		registerMainIO(io, inputs);
	}

	/** Follows a motion profile generated onboard */
	public Command followProfiledTrajectory(
			Supplier<Angle> positionSupplier,
			Supplier<AngularVelocity> velocitySupplier,
			Supplier<AngularAcceleration> accelerationSupplier,
			Supplier<Voltage> feedforwardSupplier) {
		return run(() ->
						io.setProfiledSetpoint(
								positionSupplier.get(),
								velocitySupplier.get(),
								accelerationSupplier.get(),
								feedforwardSupplier.get()))
				.andThen(idle());
	}

	/** Follows a motion profile generated onboard */
	public Command followProfiledTrajectory(
			Supplier<Angle> positionSupplier,
			Supplier<AngularVelocity> velocitySupplier,
			Supplier<AngularAcceleration> accelerationSupplier) {
		return run(() ->
						io.setProfiledSetpoint(
								positionSupplier.get(), velocitySupplier.get(), accelerationSupplier.get()))
				.andThen(idle());
	}

	/** Follows a motion profile generated onboard */
	public Command followProfiledTrajectory(
			Supplier<Angle> positionSupplier, Supplier<AngularVelocity> velocitySupplier) {
		return run(() -> io.setProfiledSetpoint(positionSupplier.get(), velocitySupplier.get()))
				.andThen(idle());
	}

	/** Follows a motion profile generated onboard */
	public Command followProfiledTrajectory(Supplier<Angle> positionSupplier) {
		return run(() -> io.setProfiledSetpoint(positionSupplier.get())).andThen(idle());
	}

	/** Uses a plain PID to move to a position, until it is in range */
	public Command moveToPositionBlocking(Angle position) {
		return runOnce(() -> io.setPosition(position))
				.andThen(Commands.waitUntil(isPositionInRange(position)));
	}

	/** Uses a trapezoidal profile to move to a position, until it is in range */
	public Command profiledMoveToPositionBlocking(Angle position) {
		return runOnce(() -> io.setProfiledPosition(position))
				.andThen(Commands.waitUntil(isPositionInRange(position)));
	}

	/** Accelerate to a velocity with a basic PID, until it is in range */
	public Command moveWithVelocityBlocking(AngularVelocity velocity) {
		return runOnce(() -> io.setVelocity(velocity))
				.andThen(Commands.waitUntil(isVelocityInRange(velocity)));
	}

	/** Uses a plain PID to move to a position, and holds it */
	public Command moveToPosition(Angle position) {
		return runOnce(() -> io.setPosition(position)).andThen(idle());
	}

	/** Uses a trapezoidal profile to move to a position, and holds it */
	public Command profiledMoveToPosition(Angle position) {
		return runOnce(() -> io.setProfiledPosition(position)).andThen(idle());
	}

	/** Accelerate to a velocity with a basic PID, and holds it */
	public Command moveWithVelocity(AngularVelocity velocity) {
		return runOnce(() -> io.setVelocity(velocity)).andThen(idle());
	}

	/** Applies a torque current to the motor, and holds it */
	public Command moveWithCurrent(Current current) {
		return runOnce(() -> io.setTorqueCurrent(current)).andThen(idle());
	}

	/** Applies a brake to the motor, and holds it */
	public Command brake() {
		return runOnce(() -> io.setBrake()).andThen(idle());
	}

	/** Sets the motor to coast, and holds it */
	public Command coast() {
		return runOnce(() -> io.setCoast()).andThen(idle());
	}

	/** Resets the position of the motor encoder */
	public Command resetPosition(Angle position) {
		return runOnce(() -> io.resetPosition(position));
	}

	/** Whether position is within a tolerance, specified in the config object */
	public Trigger isPositionInRange(Angle target) {
		return new Trigger(
						() -> Util.epsilonEquals(getCurrentPosition(), target, config.positionEpsilon))
				.debounce(config.debouncePeriod.in(Seconds));
	}

	/** Whether velocity is within a tolerance, specified in the config object */
	public Trigger isVelocityInRange(AngularVelocity target) {
		return new Trigger(
						() -> Util.epsilonEquals(getCurrentVelocity(), target, config.velocityEpsilon))
				.debounce(config.debouncePeriod.in(Seconds));
	}

	/** Returns the current position */
	public Angle getCurrentPosition() {
		return inputs.position;
	}

	/** Returns the current mechanism velocity */
	public AngularVelocity getCurrentVelocity() {
		return inputs.velocity;
	}
}
