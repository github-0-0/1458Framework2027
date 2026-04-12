package org.redtierobotics.lib.sim.servo.arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.redtierobotics.lib.sim.servo.ServoSim;
import org.redtierobotics.lib.sim.servo.ServoSimState;

public class ArmSim implements ServoSim<ArmSimState> {
	private SingleJointedArmSim sim;
	private ArmSimState state;

	public ArmSim(SingleJointedArmSim sim) {
		this.sim = sim;
		state = new ArmSimState();
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
	public ArmSimState getState() {
		state.position.mut_setMagnitude(sim.getAngleRads());
		state.speed.mut_setMagnitude(sim.getVelocityRadPerSec());

		return state;
	}

	@Override
	public void setState(ServoSimState state) {
		sim.setState(state.position.in(Radians), state.speed.in(RadiansPerSecond));
	}
}
