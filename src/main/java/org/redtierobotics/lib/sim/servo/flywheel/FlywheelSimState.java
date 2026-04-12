package org.redtierobotics.lib.sim.servo.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.MutAngularAcceleration;
import org.redtierobotics.lib.sim.servo.ServoSimState;

public class FlywheelSimState extends ServoSimState {
	public MutAngularAcceleration accel = RadiansPerSecondPerSecond.mutable(0);

	public FlywheelSimState clone() {
		var s = (FlywheelSimState) super.clone();
		s.accel = accel.mutableCopy();
		return s;
	}
}
