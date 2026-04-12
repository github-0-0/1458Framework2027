package org.redtierobotics.robot2027.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;
import org.redtierobotics.lib.io.Inputs;

@AutoLog
public class DriveInputs extends Inputs {
	public Pose2d fieldPose = new Pose2d();
	public ChassisSpeeds fieldSpeeds = new ChassisSpeeds();
	public ChassisSpeeds robotSpeeds = new ChassisSpeeds();
}
