package org.redtierobotics.lib.subsystembases.simple.servo.talonfx;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXInputsAutoLogged;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXMotorIO;
import org.redtierobotics.lib.subsystembases.simple.servo.ServoSubsystemBase;

public class TalonFXServoSubsystemBase extends ServoSubsystemBase {
	public TalonFXServoSubsystemBase(TalonFXMotorIO io, TalonFXServoSubsystemConstants constants) {
		super(io, new TalonFXInputsAutoLogged(), constants);
	}

	public final Command manualTune = runOnce(() -> ((TalonFXMotorIO) io).tuneGains());

	public Command manualTune() {
		return manualTune;
	}

	public final SysIdRoutine sysIdRoutine =
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

	public Command sysid(boolean forward, boolean dynamic) {
		return dynamic
				? sysIdRoutine.dynamic(
						forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse)
				: sysIdRoutine.quasistatic(
						forward ? SysIdRoutine.Direction.kForward : SysIdRoutine.Direction.kReverse);
	}
}
