package org.redtierobotics.lib.io.motor.talonfx;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;
import org.redtierobotics.lib.sim.servo.ServoSim;
import org.redtierobotics.lib.sim.servo.ServoSimState;
import org.redtierobotics.lib.util.CanDevice;

public class SimTalonFXMotorIO<S extends ServoSim<?>> extends TalonFXMotorIO {
	protected S sim;
	private Notifier simNotifier = null;
	private double lastUpdateTimestamp = 0.0;

	// Used to handle mechanisms that wrap.
	private boolean invertVoltage = false;

	protected AtomicReference<Double> lastRotations = new AtomicReference<>((double) 0.0);
	protected AtomicReference<Double> lastRPS = new AtomicReference<>((double) 0.0);

	protected double ratio = 0.0;

	protected AtomicReference<ServoSimState> cache;

	public SimTalonFXMotorIO(CanDevice device, TalonFXConfiguration config, S sim, double ratio) {
		super(device, config);
		this.ratio = ratio;
		this.sim = sim;
		cache = new AtomicReference<>(sim.getState().clone());
		simNotifier =
				new Notifier(
						() -> {
							updateSimState();
						});
		simNotifier.startPeriodic(0.005);
	}

	public SimTalonFXMotorIO(CanDevice device, TalonFXConfiguration config, S sim) {
		this(
				device,
				config,
				sim,
				config.Feedback.RotorToSensorRatio * config.Feedback.SensorToMechanismRatio);
	}

	// Need to use rad of the mechanism itself.
	public void setPositionRad(double rad) {
		cache.getAndUpdate(
				state -> {
					state.position.mut_setMagnitude(
							(config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive ? -1.0 : 1.0) * rad);
					state.speed.mut_setMagnitude(sim.getState().speed.magnitude());
					return state;
				});
		sim.setState(cache.get());
		Logger.recordOutput(name + "/Sim/setPositionRad", rad);
	}

	protected void updateSimState() {
		var simState = motor.getSimState();
		double simVoltage = addFriction(simState.getMotorVoltage(), 0.25);
		simVoltage = (invertVoltage) ? -simVoltage : simVoltage;
		sim.applyInput(simVoltage);
		Logger.recordOutput(name + "/Sim/SimulatorVoltage", simVoltage);

		double timestamp = Timer.getFPGATimestamp();
		sim.periodic(timestamp - lastUpdateTimestamp);
		lastUpdateTimestamp = timestamp;

		// Find current state of sim in radians from 0 point
		double simPositionRads = sim.getState().position.magnitude();
		Logger.recordOutput(name + "/Sim/SimulatorPosition", simPositionRads);

		// Mutate rotor position
		double rotorPosition = Units.radiansToRotations(simPositionRads) * ratio;
		lastRotations.set(rotorPosition);
		simState.setRawRotorPosition(rotorPosition);
		Logger.recordOutput(name + "/Sim/setRawRotorPosition", rotorPosition);

		// Mutate rotor vel
		double rotorVel = Units.radiansToRotations(sim.getState().speed.magnitude()) * ratio;
		lastRPS.set(rotorVel);
		simState.setRotorVelocity(rotorVel);
		Logger.recordOutput(name + "/Sim/SimulatorVelocity", sim.getState().speed.magnitude());
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
