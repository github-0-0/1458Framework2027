package org.redtierobotics.lib.subsystembases.simple.servo;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import org.redtierobotics.lib.subsystembases.simple.SubsystemConfig;
import org.redtierobotics.lib.util.CanDevice;

/** A config for servo subsystems */
public abstract class ServoSubsystemConfig extends SubsystemConfig {
	public CanDevice main;
	public Angle positionEpsilon;
	public AngularVelocity velocityEpsilon;
	public Time debouncePeriod;

	/**
	 * Constructs a servo config object
	 *
	 * @param device The CAN device of the motor
	 * @param posEps The angular position tolerance for blocking position commands
	 * @param velEps The angular velocity tolerance for blocking velocity commands
	 * @param debounce The debounce period for blocking commands
	 */
	public ServoSubsystemConfig(
			CanDevice device, Angle posEps, AngularVelocity velEps, Time debounce) {
		this.main = device;
		this.positionEpsilon = posEps;
		this.velocityEpsilon = velEps;
		this.debouncePeriod = debounce;
	}
}
