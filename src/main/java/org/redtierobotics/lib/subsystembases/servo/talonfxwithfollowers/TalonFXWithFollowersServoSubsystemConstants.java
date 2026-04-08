package org.redtierobotics.lib.subsystembases.servo.talonfxwithfollowers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import org.redtierobotics.lib.subsystembases.servo.talonfx.TalonFXServoSubsystemConstants;
import org.redtierobotics.lib.util.CanDevice;

public class TalonFXWithFollowersServoSubsystemConstants extends TalonFXServoSubsystemConstants {
	public CanDevice[] followers;
	public MotorAlignmentValue[] followerAlignmentValues;

	public TalonFXWithFollowersServoSubsystemConstants(
			CanDevice device,
			CanDevice[] followers,
			MotorAlignmentValue[] followerAlignmentValues,
			Angle posEps,
			AngularVelocity velEps,
			Time debounce,
			TalonFXConfiguration config) {
		super(device, posEps, velEps, debounce, config);
		this.followers = followers;
		this.followerAlignmentValues = followerAlignmentValues;
	}
}
