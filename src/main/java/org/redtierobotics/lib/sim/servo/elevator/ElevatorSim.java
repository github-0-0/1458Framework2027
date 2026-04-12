package org.redtierobotics.lib.sim.servo.elevator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.redtierobotics.lib.sim.servo.ServoSim;
import org.redtierobotics.lib.sim.servo.ServoSimState;

public class ElevatorSim implements ServoSim<ElevatorSimState> {
	private edu.wpi.first.wpilibj.simulation.ElevatorSim sim;
	private ElevatorSimState state;

	public ElevatorSim(edu.wpi.first.wpilibj.simulation.ElevatorSim sim) {
		this.sim = sim;
		state = new ElevatorSimState();
	}

	@Override
	public void periodic(double dt) {
		sim.update(dt);
	}

	@Override
	public void applyInput(double input) {
		sim.setInputVoltage(input);
	}

	@Override
	public ElevatorSimState getState() {
		state.position.mut_setMagnitude(sim.getPositionMeters());
		state.speed.mut_setMagnitude(sim.getVelocityMetersPerSecond());

		return state;
	}

	@Override
	public void setState(ServoSimState state) {
		sim.setState(state.position.in(Radians), state.speed.in(RadiansPerSecond));
	}
}
