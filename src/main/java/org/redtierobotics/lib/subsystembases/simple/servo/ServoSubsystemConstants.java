package org.redtierobotics.lib.subsystembases.simple.servo;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import org.redtierobotics.lib.subsystembases.simple.SubsystemConstants;
import org.redtierobotics.lib.util.CanDevice;

public abstract class ServoSubsystemConstants extends SubsystemConstants {
	public CanDevice main;
	public Angle positionEpsilon;
	public AngularVelocity velocityEpsilon;
	public Time debouncePeriod;

	public ServoSubsystemConstants(
			CanDevice device, Angle posEps, AngularVelocity velEps, Time debounce) {
		this.main = device;
		this.positionEpsilon = posEps;
		this.velocityEpsilon = velEps;
		this.debouncePeriod = debounce;
	}
}
