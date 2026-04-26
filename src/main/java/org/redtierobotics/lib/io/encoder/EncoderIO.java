package org.redtierobotics.lib.io.encoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Frequency;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.redtierobotics.lib.io.IO;

public interface EncoderIO<I extends EncoderInputs & LoggableInputs> extends IO<I> {
	/** Sets the update frequency of the encoder */
	void updateFrequency(Frequency frequency);

	/** Resets the position of the encoder */
	void resetPosition(Angle position);
}
