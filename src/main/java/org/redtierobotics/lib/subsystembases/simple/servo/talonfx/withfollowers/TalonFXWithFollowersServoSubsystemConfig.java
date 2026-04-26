package org.redtierobotics.lib.subsystembases.simple.servo.talonfx.withfollowers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import java.util.List;
import org.redtierobotics.lib.subsystembases.simple.servo.talonfx.TalonFXServoSubsystemConfig;
import org.redtierobotics.lib.util.CanDevice;

public class TalonFXWithFollowersServoSubsystemConfig extends TalonFXServoSubsystemConfig {
	public List<CanDevice> followers;
	public List<MotorAlignmentValue> alignments;

	/**
	 * Constructs a config object for a subsystem with multiple TalonFX's
	 *
	 * @param device The CAN device of the motor
	 * @param followers The CAN devices of all followers
	 * @param posEps The angular position tolerance for blocking position commands
	 * @param velEps The angular velocity tolerance for blocking velocity commands
	 * @param debounce The debounce period for blocking commands
	 * @param config The TalonFXConfiguration to apply to the motor
	 * @param MotorAlignmentValue The alignment values for all followers, in the same order as {@code
	 *     followers} was defined.
	 */
	public TalonFXWithFollowersServoSubsystemConfig(
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
