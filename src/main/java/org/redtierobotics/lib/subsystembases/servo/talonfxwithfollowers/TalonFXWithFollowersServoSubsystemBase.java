package org.redtierobotics.lib.subsystembases.servo.talonfxwithfollowers;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXInputs;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXInputsAutoLogged;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXMotorIO;
import org.redtierobotics.lib.subsystembases.servo.ServoSubsystemBase;

public class TalonFXWithFollowersServoSubsystemBase
		extends ServoSubsystemBase<TalonFXInputs, TalonFXInputsAutoLogged, TalonFXMotorIO> {
	protected TalonFXInputsAutoLogged[] followerInputs;
	protected TalonFXMotorIO[] followerIos;
	protected String[] keys;

	public TalonFXWithFollowersServoSubsystemBase(
			TalonFXWithFollowersServoSubsystemConstants constants) {
		super(
				new TalonFXInputsAutoLogged(),
				new TalonFXMotorIO(constants.main, constants.config),
				constants);
		
		keys = new String[followerInputs.length];
		for (int i = 0; i < followerInputs.length; i++) {
			keys[i] = getName() + "/follower" + i;
			followerIos[i].setSubsystemName(keys[i]);
		}
	}

	@Override
	public void periodic() {
		super.periodic();
		for (int i = 0; i < followerInputs.length; i++) {
			followerIos[i].readInputs(followerInputs[i]);
			Logger.processInputs(keys[i], followerInputs[i]);
		}
	}

	@Override
	public Command resetPosition(Angle position) {
		return runOnce(
				() -> {
					io.resetPosition(position);
					for (int i = 0; i < followerInputs.length; i++) {
						followerIos[i].resetPosition(position);
					}
				});
	}

	MutAngle positionCache = new MutAngle(0, 0, BaseUnits.AngleUnit);

	@Override
	public synchronized Angle getCurrentPosition() {
		positionCache.mut_replace(inputs.position);
		for (var followerInput : followerInputs) {
			positionCache.mut_plus(followerInput.position);
		}
		return positionCache.mut_divide(followerInputs.length + 1);
	}

	MutAngularVelocity velocityCache =
			new MutAngularVelocity(0, 0, BaseUnits.AngleUnit.per(BaseUnits.TimeUnit));

	@Override
	public synchronized AngularVelocity getCurrentVelocity() {
		velocityCache.mut_replace(inputs.velocity);
		for (var followerInput : followerInputs) {
			velocityCache.mut_plus(followerInput.velocity);
		}
		return velocityCache.mut_divide(followerInputs.length + 1);
	}
}
