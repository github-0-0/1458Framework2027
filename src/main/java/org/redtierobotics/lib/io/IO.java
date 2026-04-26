package org.redtierobotics.lib.io;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IO<I extends Inputs & LoggableInputs> {
	/** Updates input values from sources, or replays them */
	void readInputs(I inputs);

	/** Sets the key for the extra logging that occurs outside of inputs */
	void setLoggingKey(String name);
}
