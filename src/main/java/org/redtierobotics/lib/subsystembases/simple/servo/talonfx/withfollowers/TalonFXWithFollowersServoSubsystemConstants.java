package org.redtierobotics.lib.subsystembases.simple.servo.talonfx.withfollowers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

import java.util.List;

import org.redtierobotics.lib.subsystembases.simple.servo.talonfx.TalonFXServoSubsystemConstants;
import org.redtierobotics.lib.util.CanDevice;

public class TalonFXWithFollowersServoSubsystemConstants extends TalonFXServoSubsystemConstants {
	public List<CanDevice> followers;
	public List<MotorAlignmentValue> alignments;

	public TalonFXWithFollowersServoSubsystemConstants(
			CanDevice device,
			List<CanDevice> followers,
			Angle posEps,
			AngularVelocity velEps,
			Time debounce,
			TalonFXConfiguration config,
			List<MotorAlignmentValue> alignments) {
		super(device, posEps, velEps, debounce, config);
		this.followers = followers;
		this.alignments = alignments;
	}
}
