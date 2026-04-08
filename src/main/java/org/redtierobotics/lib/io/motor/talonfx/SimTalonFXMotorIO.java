package org.redtierobotics.lib.io.motor.talonfx;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;
import org.redtierobotics.lib.util.CanDevice;

public class SimTalonFXMotorIO extends TalonFXMotorIO {
	protected DCMotorSim sim;
	private Notifier simNotifier = null;
	private double lastUpdateTimestamp = 0.0;
	private Optional<Double> overrideRPS = Optional.empty();
	private Optional<Double> overridePos = Optional.empty();

	// Used to handle mechanisms that wrap.
	private boolean invertVoltage = false;

	protected AtomicReference<Double> lastRotations = new AtomicReference<>((double) 0.0);
	protected AtomicReference<Double> lastRPS = new AtomicReference<>((double) 0.0);

	protected double ratio = 0.0;

	public SimTalonFXMotorIO(
			CanDevice device, TalonFXConfiguration config, DCMotorSim sim, double ratio) {
		super(device, config);
		this.ratio = ratio;
		simNotifier =
				new Notifier(
						() -> {
							updateSimState();
						});
		simNotifier.startPeriodic(0.005);
	}

	// Need to use rad of the mechanism itself.
	public void setPositionRad(double rad) {
		sim.setAngle(
				(config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive ? -1.0 : 1.0) * rad);
		Logger.recordOutput(name + "/Sim/setPositionRad", rad);
	}

	protected void updateSimState() {
		var simState = motor.getSimState();
		double simVoltage = addFriction(simState.getMotorVoltage(), 0.25);
		simVoltage = (invertVoltage) ? -simVoltage : simVoltage;
		sim.setInput(simVoltage);
		Logger.recordOutput(name + "/Sim/SimulatorVoltage", simVoltage);

		double timestamp = Timer.getFPGATimestamp();
		sim.update(timestamp - lastUpdateTimestamp);
		lastUpdateTimestamp = timestamp;

		overridePos.ifPresent(aDouble -> sim.setAngle(aDouble));

		// Find current state of sim in radians from 0 point
		double simPositionRads = sim.getAngularPositionRad();
		Logger.recordOutput(name + "/Sim/SimulatorPosition", simPositionRads);

		// Mutate rotor position
		double rotorPosition = Units.radiansToRotations(simPositionRads) / ratio;
		lastRotations.set(rotorPosition);
		simState.setRawRotorPosition(rotorPosition);
		Logger.recordOutput(name + "/Sim/setRawRotorPosition", rotorPosition);

		// Mutate rotor vel
		double rotorVel = Units.radiansToRotations(sim.getAngularVelocityRadPerSec()) / ratio;
		lastRPS.set(rotorVel);
		simState.setRotorVelocity(overrideRPS.isEmpty() ? rotorVel : overrideRPS.get());
		Logger.recordOutput(name + "/Sim/SimulatorVelocity", sim.getAngularVelocity());
	}

	protected double addFriction(double motorVoltage, double frictionVoltage) {
		if (Math.abs(motorVoltage) < frictionVoltage) {
			motorVoltage = 0.0;
		} else if (motorVoltage > 0.0) {
			motorVoltage -= frictionVoltage;
		} else {
			motorVoltage += frictionVoltage;
		}
		return motorVoltage;
	}

	@Override
	public void resetPosition(Angle position) {
		setPositionRad(position.in(Radians));
	}
}
