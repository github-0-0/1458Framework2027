package org.redtierobotics.lib.subsystembases.complex;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public abstract class CompositeSubsystemBase extends SubsystemBase {
	protected Set<SubsystemBase> subsystems;

	protected String name;

	private MutTime latency = Seconds.mutable(0.0);

	public CompositeSubsystemBase(SubsystemBase... subsystems) {
		name = getName();
		this.subsystems = Set.of(subsystems);
	}

	@Override
	public void periodic() {
		double timestamp = Timer.getFPGATimestamp();

		Logger.recordOutput(
				getName() + "/currentCommand",
				(getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
		latency.mut_setMagnitude(Timer.getFPGATimestamp() - timestamp);
		Logger.recordOutput(getName() + "/latency", latency);
	}
}
