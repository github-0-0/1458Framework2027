package org.redtierobotics.lib.subsystembases.complex;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashSet;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public abstract class CompositeSubsystemBase extends SubsystemBase {
	protected Set<Subsystem> subsystems;

	protected String name;

	private MutTime latency = Seconds.mutable(0.0);

	/** A subsystem involving multiple sub-subsystems */
	public CompositeSubsystemBase(Subsystem... subsystems) {
		name = getName();
		this.subsystems = new HashSet<>();
		for (Subsystem subsystem : subsystems) {
			this.subsystems.add(subsystem);
		}
		this.subsystems.add(this);
	}

	@Override
	public void periodic() {
		double timestamp = Timer.getFPGATimestamp();

		// Record the current command
		Logger.recordOutput(
				getName() + "/currentCommand",
				(getCurrentCommand() == null) ? "None" : getCurrentCommand().getName());

		// Record latency
		latency.mut_setMagnitude(Timer.getFPGATimestamp() - timestamp);
		Logger.recordOutput(getName() + "/latency", latency);
	}
}
