package org.redtierobotics.lib.subsystembases.simple.servo.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import org.redtierobotics.lib.subsystembases.simple.servo.ServoSubsystemConstants;
import org.redtierobotics.lib.util.CanDevice;

public class TalonFXServoSubsystemConstants extends ServoSubsystemConstants {
	public TalonFXConfiguration config;

	public TalonFXServoSubsystemConstants(
			CanDevice device,
			Angle posEps,
			AngularVelocity velEps,
			Time debounce,
			TalonFXConfiguration config) {
		super(device, posEps, velEps, debounce);
		this.config = config;
	}
}
