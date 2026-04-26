package org.redtierobotics.lib.subsystembases.simple.servo.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import org.redtierobotics.lib.subsystembases.simple.servo.ServoSubsystemConfig;
import org.redtierobotics.lib.util.CanDevice;

public class TalonFXServoSubsystemConfig extends ServoSubsystemConfig {
	public TalonFXConfiguration config;

	/**
	 * Constructs a TalonFX config object
	 *
	 * @param device The CAN device of the motor
	 * @param posEps The angular position tolerance for blocking position commands
	 * @param velEps The angular velocity tolerance for blocking velocity commands
	 * @param debounce The debounce period for blocking commands
	 * @param config The TalonFXConfiguration to apply to the motor
	 */
	public TalonFXServoSubsystemConfig(
			CanDevice device,
			Angle posEps,
			AngularVelocity velEps,
			Time debounce,
			TalonFXConfiguration config) {
		super(device, posEps, velEps, debounce);
		this.config = config;
	}
}
