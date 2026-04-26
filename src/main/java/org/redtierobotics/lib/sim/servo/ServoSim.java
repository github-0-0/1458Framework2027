package org.redtierobotics.lib.sim.servo;

public interface ServoSim<S extends ServoSimState> {
	/** Applies a voltage to the simulation */
	void applyInput(double input);

	/** Steps the simulation by dt */
	void periodic(double dt);

	/** Gets the state of the simulation */
	S getState();

	/** Overrides the state of the simulation */
	void setState(ServoSimState state);
}
