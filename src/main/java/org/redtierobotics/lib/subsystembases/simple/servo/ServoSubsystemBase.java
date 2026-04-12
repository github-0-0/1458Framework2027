package org.redtierobotics.lib.subsystembases.simple.servo;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
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
	protected ServoSubsystemConstants constants;

	public <A extends MotorInputs & LoggableInputs> ServoSubsystemBase(
			MotorIO<A> io, A inputs, ServoSubsystemConstants constants) {
		super();
		this.io = io;
		this.inputs = inputs;
		this.constants = constants;
		registerMainIO(io, inputs);
	}

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

	public Command followProfiledTrajectory(
			Supplier<Angle> positionSupplier,
			Supplier<AngularVelocity> velocitySupplier,
			Supplier<AngularAcceleration> accelerationSupplier) {
		return run(() ->
						io.setProfiledSetpoint(
								positionSupplier.get(), velocitySupplier.get(), accelerationSupplier.get()))
				.andThen(idle());
	}

	public Command followProfiledTrajectory(
			Supplier<Angle> positionSupplier, Supplier<AngularVelocity> velocitySupplier) {
		return run(() -> io.setProfiledSetpoint(positionSupplier.get(), velocitySupplier.get()))
				.andThen(idle());
	}

	public Command followProfiledTrajectory(Supplier<Angle> positionSupplier) {
		return run(() -> io.setProfiledSetpoint(positionSupplier.get())).andThen(idle());
	}

	public Command moveToPositionBlocking(Angle position) {
		return runOnce(() -> io.setPosition(position)).until(isPositionInRange(position));
	}

	public Command profiledMoveToPositionBlocking(Angle position) {
		return runOnce(() -> io.setProfiledPosition(position)).until(isPositionInRange(position));
	}

	public Command moveWithVelocityBlocking(AngularVelocity velocity) {
		return runOnce(() -> io.setVelocity(velocity)).until(isVelocityInRange(velocity));
	}

	public Command moveToPosition(Angle position) {
		return runOnce(() -> io.setPosition(position)).andThen(idle());
	}

	public Command profiledMoveToPosition(Angle position) {
		return runOnce(() -> io.setProfiledPosition(position)).andThen(idle());
	}

	public Command moveWithVelocity(AngularVelocity velocity) {
		return runOnce(() -> io.setVelocity(velocity)).andThen(idle());
	}

	public Command moveWithCurrent(Current current) {
		return runOnce(() -> io.setTorqueCurrent(current)).andThen(idle());
	}

	public Command brake() {
		return runOnce(() -> io.setBrake()).andThen(idle());
	}

	public Command coast() {
		return runOnce(() -> io.setCoast()).andThen(idle());
	}

	public Command resetPosition(Angle position) {
		return runOnce(() -> io.resetPosition(position));
	}

	public Trigger isPositionInRange(Angle target) {
		return new Trigger(
						() -> Util.epsilonEquals(getCurrentPosition(), target, constants.positionEpsilon))
				.debounce(constants.debouncePeriod.in(Seconds));
	}

	public Trigger isVelocityInRange(AngularVelocity target) {
		return new Trigger(
						() -> Util.epsilonEquals(getCurrentVelocity(), target, constants.velocityEpsilon))
				.debounce(constants.debouncePeriod.in(Seconds));
	}

	public Angle getCurrentPosition() {
		return inputs.position;
	}

	public AngularVelocity getCurrentVelocity() {
		return inputs.velocity;
	}
}
