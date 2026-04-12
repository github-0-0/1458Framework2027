package org.redtierobotics.lib.io.gyro;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import org.redtierobotics.lib.io.Inputs;

public class GyroInputs extends Inputs {
	public MutAngle yaw = Radians.mutable(0);
	public MutAngle pitch = Radians.mutable(0);
	public MutAngle roll = Radians.mutable(0);
	public MutAngularVelocity yawVelocity = RotationsPerSecond.mutable(0);
	public MutAngularVelocity pitchVelocity = RotationsPerSecond.mutable(0);
	public MutAngularVelocity rollVelocity = RotationsPerSecond.mutable(0);
}
