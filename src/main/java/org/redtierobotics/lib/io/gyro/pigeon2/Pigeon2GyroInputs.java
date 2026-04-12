package org.redtierobotics.lib.io.gyro.pigeon2;

import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;
import org.redtierobotics.lib.io.gyro.GyroInputs;

@AutoLog
public class Pigeon2GyroInputs extends GyroInputs {
	public Rotation3d rotation = Rotation3d.kZero;
}
