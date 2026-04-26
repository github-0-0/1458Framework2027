package org.redtierobotics.lib.io.motor;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.redtierobotics.lib.io.IO;

public interface MotorIO<I extends MotorInputs & LoggableInputs> extends IO<I> {
	/** Gets the name of the motor */
	String getName();

	/** {@inheritDoc} */
	void readInputs(I inputs);

	/** Sets the raw duty cycle */
	void setDutyCycle(double dutyCycle);

	/** Sets the raw voltage */
	void setVoltage(Voltage volts);

	/** Sets the motor targets for various derivatives of position */
	void setProfiledSetpoint(
			Angle position,
			AngularVelocity velocity,
			AngularAcceleration acceleration,
			int slot,
			Voltage feedforward);

	/** Sets the motor targets for various derivatives of position */
	default void setProfiledSetpoint(
			Angle position,
			AngularVelocity velocity,
			AngularAcceleration acceleration,
			Voltage feedforward) {
		setProfiledSetpoint(position, velocity, acceleration, 0, feedforward);
	}

	/** Sets the motor targets for various derivatives of position */
	default void setProfiledSetpoint(
			Angle position, AngularVelocity velocity, AngularAcceleration acceleration) {
		setProfiledSetpoint(position, velocity, acceleration, Volts.zero());
	}

	/** Sets the motor targets for various derivatives of position */
	default void setProfiledSetpoint(Angle position, AngularVelocity velocity) {
		setProfiledSetpoint(position, velocity, RotationsPerSecondPerSecond.zero());
	}

	/** Sets the motor targets for various derivatives of position */
	default void setProfiledSetpoint(Angle position) {
		setProfiledSetpoint(position, RotationsPerSecond.zero());
	}

	/** Sets the motor to brake mode */
	void setBrake();

	/** Sets the motor to coast mode */
	void setCoast();

	/** Uses a simple PID and feedforwards to move the motor to a position */
	void setPosition(Angle position);

	/** Uses a motion profile to move the motor to a position */
	void setProfiledPosition(Angle position);

	/** Uses a simple PID and feedforwards to accelerate the motor to a velocity */
	void setVelocity(AngularVelocity velocity);

	/** Sets the torque via stator current */
	void setTorqueCurrent(Current current);

	/** Resets internal or external encoder position */
	void resetPosition(Angle position);
}
