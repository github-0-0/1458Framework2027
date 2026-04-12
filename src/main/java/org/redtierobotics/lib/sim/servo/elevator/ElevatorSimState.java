package org.redtierobotics.lib.sim.servo.elevator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import org.redtierobotics.lib.sim.servo.ServoSimState;

public class ElevatorSimState extends ServoSimState {
	public MutAngle position = Radians.mutable(0);
	public MutAngularVelocity speed = RadiansPerSecond.mutable(0);
}
