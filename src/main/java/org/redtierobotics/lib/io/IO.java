package org.redtierobotics.lib.io;

public interface IO<I extends Inputs> {
	void readInputs(I inputs);

	void setSubsystemName(String name);
}
