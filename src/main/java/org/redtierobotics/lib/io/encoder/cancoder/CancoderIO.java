package org.redtierobotics.lib.io.encoder.cancoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import java.util.List;
import org.redtierobotics.lib.io.encoder.EncoderIO;

public class CancoderIO implements EncoderIO<CancoderIOInputsAutoLogged> {
	protected String name = "";
	public final CANcoder cancoder;

	public StatusSignal<Angle> positionSignal;
	public StatusSignal<AngularVelocity> velocitySignal;

	public List<BaseStatusSignal> signals;

	public CancoderIO(CANcoder cancoder) {
		this.cancoder = cancoder;
		positionSignal = cancoder.getPosition();
		velocitySignal = cancoder.getVelocity();
		signals = List.of(positionSignal, velocitySignal);
	}

	@Override
	public void readInputs(CancoderIOInputsAutoLogged inputs) {
		BaseStatusSignal.refreshAll(signals);
		inputs.position.mut_replace(positionSignal.getValue());
		inputs.velocityRPS.mut_replace(velocitySignal.getValue());
	}

	@Override
	public void setLoggingKey(String name) {
		this.name = name;
	}

	public void resetPosition(Angle newPosition) {
		cancoder.setPosition(newPosition);
	}

	@Override
	public void updateFrequency(Frequency frequency) {
		BaseStatusSignal.setUpdateFrequencyForAll(frequency, signals);
	}
}
