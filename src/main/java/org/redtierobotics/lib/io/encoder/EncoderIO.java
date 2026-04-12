package org.redtierobotics.lib.io.encoder;

import edu.wpi.first.units.measure.Frequency;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.redtierobotics.lib.io.IO;

public interface EncoderIO<I extends EncoderInputs & LoggableInputs> extends IO<I> {
	void updateFrequency(Frequency frequency);
}
