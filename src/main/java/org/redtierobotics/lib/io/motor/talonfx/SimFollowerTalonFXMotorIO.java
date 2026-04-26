package org.redtierobotics.lib.io.motor.talonfx;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import org.redtierobotics.lib.util.CanDevice;

public class SimFollowerTalonFXMotorIO extends FollowerTalonFXMotorIO {
	protected SimTalonFXMotorIO<?> leader;
	protected MotorAlignmentValue alignmentValue;
	protected Notifier simNotifier;

	/** Simulation for followers */
	public SimFollowerTalonFXMotorIO(
			CanDevice device,
			CanDevice main,
			MotorAlignmentValue alignmentValue,
			TalonFXConfiguration config) {
		super(device, main, alignmentValue, config);
		this.alignmentValue = alignmentValue;

		mirrorLeaderState();
		simNotifier = new Notifier(this::mirrorLeaderState);
		simNotifier.startPeriodic(0.005);
	}

	/** Sets the leader IO for mirroring */
	public void setLeaderIO(SimTalonFXMotorIO<?> leader) {
		this.leader = leader;
	}

	/** Mirrors the leader position and velocity */
	private void mirrorLeaderState() {
		if (leader != null) {
			double rotorPosition = leader.lastRotations.get();
			double rotorVelocity = leader.lastRPS.get();

			var simState = motor.getSimState();
			simState.setRawRotorPosition(rotorPosition);
			simState.setRotorVelocity(rotorVelocity);
		}
	}

	/** {@inheritDoc} */
	@Override
	public void resetPosition(Angle position) {
		var simState = motor.getSimState();
		simState.setRawRotorPosition(position.in(Rotations));
		simState.setRotorVelocity(0.0);
	}
}
