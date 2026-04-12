package org.redtierobotics.lib.sim.servo;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;

public class ServoSimState implements Cloneable {
	public MutAngle position = Radians.mutable(0);
	public MutAngularVelocity speed = RadiansPerSecond.mutable(0);

	public ServoSimState clone() {
		var s = new ServoSimState();
		s.position = position.mutableCopy();
		s.speed = speed.mutableCopy();
		return s;
	}
}
