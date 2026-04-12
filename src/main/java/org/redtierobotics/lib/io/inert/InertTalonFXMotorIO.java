package org.redtierobotics.lib.io.inert;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.List;
import org.redtierobotics.lib.io.IO;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXInputsAutoLogged;

/** A class for overridden motors. Does not do anything. Very useless except for in 1 case. */
public class InertTalonFXMotorIO implements IO<TalonFXInputsAutoLogged> {
	protected String name = "";
	protected TalonFX motor;

	protected final StatusSignal<Angle> positionSignal;
	protected final StatusSignal<AngularVelocity> velocitySignal;
	protected final StatusSignal<Voltage> voltageSignal;
	protected final StatusSignal<Current> currentStatorSignal;
	protected final StatusSignal<Current> currentSupplySignal;
	protected final StatusSignal<Angle> rawRotorPositionSignal;
	protected final StatusSignal<Temperature> temperatureSignal;

	protected final List<BaseStatusSignal> signals;

	public InertTalonFXMotorIO(TalonFX motor) {
		this.motor = motor;
		positionSignal = motor.getPosition();
		velocitySignal = motor.getVelocity();
		voltageSignal = motor.getMotorVoltage();
		currentStatorSignal = motor.getStatorCurrent();
		currentSupplySignal = motor.getSupplyCurrent();
		rawRotorPositionSignal = motor.getRotorPosition();
		temperatureSignal = motor.getDeviceTemp();

		signals =
				List.of(
						positionSignal,
						velocitySignal,
						voltageSignal,
						currentStatorSignal,
						currentSupplySignal,
						rawRotorPositionSignal,
						temperatureSignal);
	}

	@Override
	public void readInputs(TalonFXInputsAutoLogged inputs) {
		BaseStatusSignal.refreshAll(signals);

		inputs.position.mut_replace(positionSignal.getValue());
		inputs.velocity.mut_replace(velocitySignal.getValue());
		inputs.appliedVoltage.mut_replace(voltageSignal.getValue());
		inputs.statorCurrent.mut_replace(currentStatorSignal.getValue());
		inputs.supplyCurrent.mut_replace(currentSupplySignal.getValue());
		inputs.rawRotorPosition.mut_replace(rawRotorPositionSignal.getValue());
		inputs.temperature.mut_replace(temperatureSignal.getValue());
	}

	@Override
	public void setLoggingKey(String name) {
		this.name = name;
	}
}
