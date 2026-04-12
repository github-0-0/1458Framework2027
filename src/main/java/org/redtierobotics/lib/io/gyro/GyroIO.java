package org.redtierobotics.lib.io.gyro;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.redtierobotics.lib.io.IO;

public interface GyroIO<I extends GyroInputs & LoggableInputs> extends IO<I> {}
