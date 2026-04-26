package org.redtierobotics.lib.io.motor.talonfx;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import org.redtierobotics.lib.util.CanDevice;
import org.redtierobotics.lib.util.CtreUtil;

public class FollowerTalonFXMotorIO extends TalonFXMotorIO {
	/** Creates a follower talon FX. It follows the main device. */
	public FollowerTalonFXMotorIO(
			CanDevice device,
			CanDevice main,
			MotorAlignmentValue alignmentValue,
			TalonFXConfiguration config) {
		super(device, config);
		CtreUtil.applyFollower(main.id(), motor, alignmentValue);
	}

	@Override
	public StatusCode setControl(ControlRequest request) {
		DriverStation.reportError("Cannot set control for a follower motor", true);
		return StatusCode.ControlModeNotValid;
	}

	@Override
	public void setDutyCycle(double dutyCycle) {
		DriverStation.reportError("Cannot set control for a follower motor", true);
	}

	@Override
	public void setVoltage(Voltage voltage) {
		DriverStation.reportError("Cannot set control for a follower motor", true);
	}

	@Override
	public void setProfiledSetpoint(
			Angle position,
			AngularVelocity velocity,
			AngularAcceleration acceleration,
			int slot,
			Voltage feedforward) {
		DriverStation.reportError("Cannot set control for a follower motor", true);
	}

	@Override
	public void setBrake() {
		DriverStation.reportError("Cannot set control for a follower motor", true);
	}

	@Override
	public void setCoast() {
		DriverStation.reportError("Cannot set control for a follower motor", true);
	}

	@Override
	public void setPosition(Angle position) {
		DriverStation.reportError("Cannot set control for a follower motor", true);
	}

	@Override
	public void setProfiledPosition(Angle position) {
		DriverStation.reportError("Cannot set control for a follower motor", true);
	}

	@Override
	public void setVelocity(AngularVelocity velocity) {
		DriverStation.reportError("Cannot set control for a follower motor", true);
	}

	@Override
	public void setTorqueCurrent(Current current) {
		DriverStation.reportError("Cannot set control for a follower motor", true);
	}
}
