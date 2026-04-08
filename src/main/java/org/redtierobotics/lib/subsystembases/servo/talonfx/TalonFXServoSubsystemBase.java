package org.redtierobotics.lib.subsystembases.servo.talonfx;

import org.redtierobotics.lib.io.motor.talonfx.TalonFXInputs;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXInputsAutoLogged;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXMotorIO;
import org.redtierobotics.lib.subsystembases.servo.ServoSubsystemBase;

public class TalonFXServoSubsystemBase
		extends ServoSubsystemBase<TalonFXInputs, TalonFXInputsAutoLogged, TalonFXMotorIO> {
	public TalonFXServoSubsystemBase(TalonFXServoSubsystemConstants constants) {
		super(
				new TalonFXInputsAutoLogged(),
				new TalonFXMotorIO(constants.main, constants.config),
				constants);
	}
}
