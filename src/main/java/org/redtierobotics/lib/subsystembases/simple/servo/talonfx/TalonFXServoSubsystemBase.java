package org.redtierobotics.lib.subsystembases.simple.servo.talonfx;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXInputsAutoLogged;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXMotorIO;
import org.redtierobotics.lib.subsystembases.simple.servo.ServoSubsystemBase;

public class TalonFXServoSubsystemBase extends ServoSubsystemBase {
	/**
	 * Constructs a subsystem with one TalonFX motor
	 *
	 * @param io The TalonFX IO object
	 * @param config Configs for the TalonFX
	 */
	public TalonFXServoSubsystemBase(TalonFXMotorIO io, TalonFXServoSubsystemConfig config) {
		super(io, new TalonFXInputsAutoLogged(), config);
	}

	protected final Command manualTune = runOnce(() -> ((TalonFXMotorIO) io).tuneGains());

	/** Updates feedback gains based on network values */
	public Command manualTune() {
		return manualTune;
	}

	protected final SysIdRoutine sysIdRoutine =
			new SysIdRoutine(
					new SysIdRoutine.Config(
							null,
							null,
							null, // Use default config
							(state) -> Logger.recordOutput(getName() + "/Sysid", state.toString())),
					new SysIdRoutine.Mechanism(
							(voltage) -> io.setVoltage(voltage),
							null, // No log consumer, since data is recorded by AdvantageKit
							this));

	/**
	 * Runs sysid on the subsystem
	 *
	 * @param forward True for forward, false for reverse
	 * @param dynamic True for dynamic, false for quasistatic
	 * @return
	 */
	public Command sysid(boolean forward, boolean dynamic) {
		return dynamic
				? sysIdRoutine.dynamic(
						forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse)
				: sysIdRoutine.quasistatic(
						forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
	}
}
