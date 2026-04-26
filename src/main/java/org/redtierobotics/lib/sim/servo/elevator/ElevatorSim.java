package org.redtierobotics.lib.sim.servo.elevator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import org.redtierobotics.lib.sim.servo.ServoSim;
import org.redtierobotics.lib.sim.servo.ServoSimState;

/** Simulates an elevator, or any similar linear subsystem with gravity */
public class ElevatorSim implements ServoSim<ElevatorSimState> {
	private edu.wpi.first.wpilibj.simulation.ElevatorSim sim;
	private ElevatorSimState state;

	/** Constructs an ElevatorSim from the wpilib equivalent */
	public ElevatorSim(edu.wpi.first.wpilibj.simulation.ElevatorSim sim) {
		this.sim = sim;
		state = new ElevatorSimState();
	}

	/** {@inheritDoc} */
	@Override
	public void periodic(double dt) {
		sim.update(dt);
	}

	/** {@inheritDoc} */
	@Override
	public void applyInput(double input) {
		sim.setInputVoltage(input);
	}

	/** {@inheritDoc} */
	@Override
	public ElevatorSimState getState() {
		state.position.mut_setMagnitude(Units.rotationsToRadians(sim.getPositionMeters()));
		state.speed.mut_setMagnitude(Units.rotationsToRadians(sim.getVelocityMetersPerSecond()));

		return state;
	}

	/** {@inheritDoc} */
	@Override
	public void setState(ServoSimState state) {
		sim.setState(state.position.in(Radians), state.speed.in(RadiansPerSecond));
	}
}
