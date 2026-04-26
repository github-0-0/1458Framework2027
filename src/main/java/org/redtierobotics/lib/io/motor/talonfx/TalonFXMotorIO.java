package org.redtierobotics.lib.io.motor.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.redtierobotics.lib.io.motor.MotorIO;
import org.redtierobotics.lib.util.CanDevice;
import org.redtierobotics.lib.util.CtreUtil;

public class TalonFXMotorIO implements MotorIO<TalonFXInputsAutoLogged> {
	protected final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
	protected final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0);
	protected final VoltageOut voltageControl = new VoltageOut(0.0);
	protected final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);
	protected final MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0.0);
	protected final DynamicMotionMagicVoltage dynamicMotionMagicVoltage =
			new DynamicMotionMagicVoltage(0.0, 0.0, 0.0);
	protected final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);
	protected final CoastOut coastControl = new CoastOut();
	protected final StaticBrake brakeControl = new StaticBrake();

	protected final StatusSignal<Angle> positionSignal;
	protected final StatusSignal<AngularVelocity> velocitySignal;
	protected final StatusSignal<Voltage> voltageSignal;
	protected final StatusSignal<Current> currentStatorSignal;
	protected final StatusSignal<Current> currentSupplySignal;
	protected final StatusSignal<Angle> rawRotorPositionSignal;
	protected final StatusSignal<Temperature> temperatureSignal;

	protected final List<BaseStatusSignal> signals;
	@AutoLogOutput protected ControlRequest request = coastControl;
	protected TalonFX motor;
	protected TalonFXConfiguration config;

	protected String name = "";

	protected LoggedNetworkNumber p;
	protected LoggedNetworkNumber i;
	protected LoggedNetworkNumber d;
	protected LoggedNetworkNumber v;
	protected LoggedNetworkNumber s;
	protected LoggedNetworkNumber a;
	protected LoggedNetworkNumber g;

	public TalonFXMotorIO(CanDevice device, TalonFXConfiguration config) {
		motor = new TalonFX(device.id(), device.bus());
		this.config = config;
		CtreUtil.applyConfiguration(motor, config);

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

	/** Sets the motor control request */
	public StatusCode setControl(ControlRequest request) {
		this.request = request;
		Logger.recordOutput(name + "/ControlRequest/Type", request.getName());
		return motor.setControl(request);
	}

	/** {@inheritDoc} */
	@Override
	public void setDutyCycle(double dutyCycle) {
		Logger.recordOutput(name + "/ControlRequest/DutyCycle", dutyCycle);
		setControl(dutyCycleControl.withOutput(dutyCycle));
	}

	/** {@inheritDoc} */
	@Override
	public void setVoltage(Voltage voltage) {
		Logger.recordOutput(name + "/ControlRequest/Voltage", voltage);
		setControl(voltageControl.withOutput(voltage));
	}

	/** {@inheritDoc} */
	@Override
	public void setProfiledSetpoint(
			Angle position,
			AngularVelocity velocity,
			AngularAcceleration acceleration,
			int slot,
			Voltage feedforward) {
		Logger.recordOutput(name + "/ControlRequest/ProfiledSetpoint/Position", position);
		Logger.recordOutput(name + "/ControlRequest/ProfiledSetpoint/Velocity", velocity);
		Logger.recordOutput(name + "/ControlRequest/ProfiledSetpoint/Acceleration", acceleration);
		Logger.recordOutput(name + "/ControlRequest/ProfiledSetpoint/slot", slot);
		Logger.recordOutput(name + "/ControlRequest/ProfiledSetpoint/Feedforward", feedforward);
		setControl(
				dynamicMotionMagicVoltage
						.withPosition(position)
						.withVelocity(velocity)
						.withAcceleration(acceleration)
						.withSlot(slot)
						.withFeedForward(feedforward));
	}

	/** {@inheritDoc} */
	@Override
	public void setBrake() {
		setControl(brakeControl);
	}

	/** {@inheritDoc} */
	@Override
	public void setCoast() {
		setControl(coastControl);
	}

	/** {@inheritDoc} */
	@Override
	public void setPosition(Angle position) {
		Logger.recordOutput(name + "/ControlRequest/Position", position);
		setControl(positionVoltageControl.withPosition(position));
	}

	/** {@inheritDoc} */
	@Override
	public void setProfiledPosition(Angle position) {
		Logger.recordOutput(name + "/ControlRequest/ProfiledPosition", position);
		setControl(motionMagicPositionControl.withPosition(position));
	}

	/** {@inheritDoc} */
	@Override
	public void setVelocity(AngularVelocity velocity) {
		Logger.recordOutput(name + "/ControlRequest/Velocity", velocity);
		setControl(velocityVoltageControl.withVelocity(velocity));
	}

	/** {@inheritDoc} */
	@Override
	public void setTorqueCurrent(Current current) {
		Logger.recordOutput(name + "/ControlRequest/TorqueCurrent", current);
		setControl(torqueCurrentFOC.withOutput(current));
	}

	/** {@inheritDoc} */
	@Override
	public void resetPosition(Angle position) {
		CtreUtil.tryUntilOk(() -> motor.setPosition(position), motor.getDeviceID());
		Logger.recordOutput(name + "/LastResetPosition", position);
	}

	/** {@inheritDoc} */
	@Override
	public void setLoggingKey(String name) {
		this.name = name;
		p = new LoggedNetworkNumber("Tuning/" + name + "/Gains/P", config.Slot0.kP);
		i = new LoggedNetworkNumber("Tuning/" + name + "/Gains/I", config.Slot0.kI);
		d = new LoggedNetworkNumber("Tuning/" + name + "/Gains/D", config.Slot0.kD);
		v = new LoggedNetworkNumber("Tuning/" + name + "/Gains/V", config.Slot0.kV);
		s = new LoggedNetworkNumber("Tuning/" + name + "/Gains/S", config.Slot0.kS);
		a = new LoggedNetworkNumber("Tuning/" + name + "/Gains/A", config.Slot0.kA);
		g = new LoggedNetworkNumber("Tuning/" + name + "/Gains/G", config.Slot0.kG);
	}

	/** {@inheritDoc} */
	@Override
	public String getName() {
		return name;
	}

	/** Sets the gains for the motor, including all PID and feedforward configs */
	public void setGainsSlot0(double p, double i, double d, double v, double s, double a, double g) {
		config.Slot0.withKP(p).withKI(i).withKD(d).withKV(v).withKS(s).withKA(a).withKG(g);
		CtreUtil.applyConfiguration(motor, config);
	}

	/** Gets values from NT and tunes gains with them */
	public void tuneGains() {
		setGainsSlot0(
				p.getAsDouble(),
				i.getAsDouble(),
				d.getAsDouble(),
				v.getAsDouble(),
				s.getAsDouble(),
				a.getAsDouble(),
				g.getAsDouble());
	}
}
