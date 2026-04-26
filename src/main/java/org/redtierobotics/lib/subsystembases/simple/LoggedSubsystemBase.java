package org.redtierobotics.lib.subsystembases.simple;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashSet;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.redtierobotics.lib.io.IO;
import org.redtierobotics.lib.io.Inputs;

public abstract class LoggedSubsystemBase extends SubsystemBase {
	protected String name;
	protected Set<Runnable> ioRegistry;

	private MutTime latency = Seconds.mutable(0.0);

	public LoggedSubsystemBase() {
		super();
		name = getName();
		ioRegistry = new HashSet<>(1);
	}

	protected <I extends Inputs & LoggableInputs> void registerIO(String name, IO<I> io, I inputs) {
		String tableName = this.name + "/" + name;
		io.setLoggingKey(tableName);
		ioRegistry.add(
				() -> {
					io.readInputs(inputs);
					Logger.processInputs(tableName, inputs);
				});
	}

	protected <I extends Inputs & LoggableInputs> void registerMainIO(IO<I> io, I inputs) {
		io.setLoggingKey(name);
		ioRegistry.add(
				() -> {
					io.readInputs(inputs);
					Logger.processInputs(name, inputs);
				});
	}

	@Override
	public void periodic() {
		double timestamp = Timer.getFPGATimestamp();

		for (Runnable ioEntry : ioRegistry) {
			ioEntry.run();
		}

		Logger.recordOutput(
				getName() + "/currentCommand",
				(getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
		latency.mut_setMagnitude(Timer.getFPGATimestamp() - timestamp);
		Logger.recordOutput(getName() + "/latency", latency);
	}
}
