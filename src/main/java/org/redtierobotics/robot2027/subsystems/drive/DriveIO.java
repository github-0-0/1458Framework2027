package org.redtierobotics.robot2027.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.Logger;
import org.redtierobotics.lib.io.IO;
import org.redtierobotics.robot2027.subsystems.drive.ctre.CtreDrive;

public class DriveIO implements IO<DriveInputsAutoLogged> {
	public String name;
	public CtreDrive drive;
	public SwerveRequest request;

	public DriveIO(CtreDrive ctreDrive) {
		this.drive = ctreDrive;
		ctreDrive.setStateStdDevs(DriveConstants.STATE_STD_DEVS);
		ctreDrive.setDefaultCommand(
				ctreDrive.applyRequest(
						() -> {
							return request;
						}));
	}

	@Override
	public void readInputs(DriveInputsAutoLogged inputs) {
		var state = drive.getState();
		inputs.fieldPose = state.Pose;
		inputs.fieldSpeeds =
				ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
		inputs.robotSpeeds = state.Speeds;

		Logger.recordOutput(name + "/ModuleStates", state.ModuleStates);
		Logger.recordOutput(name + "/ModuleTargets", state.ModuleTargets);
	}

	public void setSwerveRequest(SwerveRequest request) {
		this.request = request;
		Logger.recordOutput(name + "/SwerveRequest", request.getClass().getSimpleName());
		drive.setControl(request);
	}

	public void addVisionMeasurement(Pose2d pose, double time) {
		drive.addVisionMeasurement(pose, time);
	}

	public void addVisionMeasurement(Pose2d pose, double time, Matrix<N3, N1> stdDevs) {
		drive.addVisionMeasurement(pose, time, stdDevs);
	}

	public SwerveRequest getSwerveRequest() {
		return request;
	}

	@Override
	public void setLoggingKey(String name) {
		this.name = name;
	}
}
