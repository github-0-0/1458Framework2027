package org.redtierobotics.lib.io;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IO<I extends Inputs & LoggableInputs> {
	void readInputs(I inputs);

	void setLoggingKey(String name);
}
