package org.redtierobotics.lib.sim.servo.dcmotor;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.redtierobotics.lib.sim.servo.ServoSim;
import org.redtierobotics.lib.sim.servo.ServoSimState;

public class MotorSim implements ServoSim<MotorSimState> {
	private DCMotorSim sim;
	private MotorSimState state;

	public MotorSim(DCMotorSim sim) {
		this.sim = sim;
		state = new MotorSimState();
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
	public MotorSimState getState() {
		state.position.mut_setMagnitude(sim.getAngularPositionRad());
		state.speed.mut_setMagnitude(sim.getAngularVelocityRadPerSec());

		return state;
	}

	@Override
	public void setState(ServoSimState state) {
		sim.setState(state.position.in(Radians), state.speed.in(RadiansPerSecond));
	}
}
