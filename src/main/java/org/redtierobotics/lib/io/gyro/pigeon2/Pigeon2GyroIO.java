package org.redtierobotics.lib.io.gyro.pigeon2;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.redtierobotics.lib.io.gyro.GyroIO;

public class Pigeon2GyroIO implements GyroIO<Pigeon2GyroInputsAutoLogged> {
	protected String name = "";
	private final StatusSignal<Angle> yaw;
	private final StatusSignal<Angle> pitch;
	private final StatusSignal<Angle> roll;
	private final StatusSignal<AngularVelocity> yawVelocity;
	private final StatusSignal<AngularVelocity> pitchVelocity;
	private final StatusSignal<AngularVelocity> rollVelocity;

	private final Pigeon2 pigeon;

	public Pigeon2GyroIO(Pigeon2 pigeon) {
		this.pigeon = pigeon;
		yaw = pigeon.getYaw();
		yawVelocity = pigeon.getAngularVelocityZWorld();
		pitch = pigeon.getPitch();
		pitchVelocity = pigeon.getAngularVelocityYWorld();
		roll = pigeon.getRoll();
		rollVelocity = pigeon.getAngularVelocityXWorld();
	}

	@Override
	public void readInputs(Pigeon2GyroInputsAutoLogged inputs) {
		inputs.rotation = pigeon.getRotation3d();
		inputs.yaw.mut_replace(yaw.getValue());
		inputs.yawVelocity.mut_replace(yawVelocity.getValue());
		inputs.pitch.mut_replace(pitch.getValue());
		inputs.pitchVelocity.mut_replace(pitchVelocity.getValue());
		inputs.roll.mut_replace(roll.getValue());
		inputs.rollVelocity.mut_replace(rollVelocity.getValue());
	}

	@Override
	public void setLoggingKey(String name) {
		this.name = name;
	}
}
