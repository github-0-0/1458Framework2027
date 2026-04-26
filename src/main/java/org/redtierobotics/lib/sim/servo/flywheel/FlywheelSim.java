package org.redtierobotics.lib.sim.servo.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.redtierobotics.lib.sim.servo.ServoSim;
import org.redtierobotics.lib.sim.servo.ServoSimState;

/** Simulates a flywheel, or any similar subsystem with inertia */
public class FlywheelSim implements ServoSim<FlywheelSimState> {
	private edu.wpi.first.wpilibj.simulation.FlywheelSim sim;
	private FlywheelSimState state;

	/** Constructs an FlywheelSim from the wpilib equivalent */
	public FlywheelSim(edu.wpi.first.wpilibj.simulation.FlywheelSim sim) {
		this.sim = sim;
		state = new FlywheelSimState();
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
	public FlywheelSimState getState() {
		state.position.mut_acc(sim.getAngularVelocityRadPerSec() * 0.020);
		state.speed.mut_setMagnitude(sim.getAngularVelocityRadPerSec());
		state.accel.mut_setMagnitude(sim.getAngularAccelerationRadPerSecSq());

		return state;
	}

	/** {@inheritDoc} */
	@Override
	public void setState(ServoSimState state) {
		sim.setAngularVelocity(state.speed.in(RadiansPerSecond));
	}
}
