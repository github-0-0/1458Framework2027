package org.redtierobotics.lib.io.encoder;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import org.littletonrobotics.junction.AutoLog;
import org.redtierobotics.lib.io.Inputs;

@AutoLog
public class EncoderInputs extends Inputs {
	public MutAngle position = Rotations.mutable(0);
	public MutAngularVelocity velocityRPS = RotationsPerSecond.mutable(0);
}
