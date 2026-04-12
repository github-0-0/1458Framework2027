package org.redtierobotics.lib.sim.servo;

public interface ServoSim<S extends ServoSimState> {
	void applyInput(double input);

	void periodic(double dt);

	S getState();

	void setState(ServoSimState state);
}
