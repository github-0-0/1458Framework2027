package org.redtierobotics.robot2027.subsystems.drive;

import static edu.wpi.first.units.Units.Seconds;
import static org.redtierobotics.robot2027.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.redtierobotics.lib.control.ControlConstants.PIDVConstants;
import org.redtierobotics.lib.control.ControlConstants.ProfiledPIDVConstants;
import org.redtierobotics.lib.control.PIDVController;
import org.redtierobotics.lib.control.ProfiledPIDVController;
import org.redtierobotics.lib.io.encoder.cancoder.CancoderIO;
import org.redtierobotics.lib.io.encoder.cancoder.CancoderIOInputsAutoLogged;
import org.redtierobotics.lib.io.gyro.pigeon2.Pigeon2GyroIO;
import org.redtierobotics.lib.io.gyro.pigeon2.Pigeon2GyroInputsAutoLogged;
import org.redtierobotics.lib.io.inert.InertTalonFXMotorIO;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXInputsAutoLogged;
import org.redtierobotics.lib.subsystembases.simple.LoggedSubsystemBase;
import org.redtierobotics.lib.trajectory.RedTrajectory;
import org.redtierobotics.lib.util.Util;
import org.redtierobotics.lib.util.Util.JoystickVector;
import org.redtierobotics.robot2027.Constants;
import org.redtierobotics.robot2027.subsystems.drive.ctre.CtreDrive;
import org.redtierobotics.robot2027.subsystems.drive.ctre.TestCtreDriveConstants;

public class Drive extends LoggedSubsystemBase {
	private final CtreDrive drivetrain;

	private DriveInputsAutoLogged inputs;
	private DriveIO io;

	private List<InertTalonFXMotorIO> driveMotorIos = new ArrayList<>(4);
	private List<TalonFXInputsAutoLogged> driveMotorInputs = new ArrayList<>(4);
	private List<InertTalonFXMotorIO> steerMotorIos = new ArrayList<>(4);
	private List<TalonFXInputsAutoLogged> steerMotorInputs = new ArrayList<>(4);
	private List<CancoderIO> cancoderIos = new ArrayList<>(4);
	private List<CancoderIOInputsAutoLogged> cancoderInputs = new ArrayList<>(4);

	private Pigeon2GyroIO gyroIO;
	private Pigeon2GyroInputsAutoLogged gyroInputs;

	private Field2d field;

	public Drive() {
		super();
		drivetrain = TestCtreDriveConstants.createDrivetrain();
		io = new DriveIO(drivetrain);
		inputs = new DriveInputsAutoLogged();
		registerMainIO(io, inputs);

		Map<String, SwerveModule<TalonFX, TalonFX, CANcoder>> modules =
				Map.of(
						"FL", drivetrain.getModule(0),
						"FR", drivetrain.getModule(1),
						"BL", drivetrain.getModule(2),
						"BR", drivetrain.getModule(3));

		for (var entry : modules.entrySet()) {
			var module = entry.getValue();
			var driveMotorIO = new InertTalonFXMotorIO(module.getDriveMotor());
			var driveMotorInputs1 = new TalonFXInputsAutoLogged();
			var steerMotorIO = new InertTalonFXMotorIO(module.getSteerMotor());
			var steerMotorInputs1 = new TalonFXInputsAutoLogged();
			var cancoderIO = new CancoderIO(module.getEncoder());
			var cancoderInputs1 = new CancoderIOInputsAutoLogged();
			registerIO("/Modules/" + entry.getKey() + "/Drive", driveMotorIO, driveMotorInputs1);
			registerIO("/Modules/" + entry.getKey() + "/Azimuth", steerMotorIO, steerMotorInputs1);
			registerIO("/Modules/" + entry.getKey() + "/Cancoder", cancoderIO, cancoderInputs1);

			driveMotorIos.add(driveMotorIO);
			driveMotorInputs.add(driveMotorInputs1);
			steerMotorIos.add(steerMotorIO);
			steerMotorInputs.add(steerMotorInputs1);
			cancoderIos.add(cancoderIO);
			cancoderInputs.add(cancoderInputs1);
		}

		gyroIO = new Pigeon2GyroIO(drivetrain.getPigeon2());
		gyroInputs = new Pigeon2GyroInputsAutoLogged();
		registerIO("/Gyro", gyroIO, gyroInputs);

		field = new Field2d();
		SmartDashboard.putData("Drive/Field", field);
	}

	@Override
	public void periodic() {
		super.periodic();
		field.setRobotPose(getPose());
	}

	public SwerveDriveState getState() {
		return drivetrain.getState();
	}

	public Pose2d getPose() {
		return inputs.fieldPose;
	}

	public CtreDrive getCtreDrive() {
		return drivetrain;
	}

	@SuppressWarnings("unchecked")
	public <S extends SwerveRequest> Command drive(Supplier<S> init, Consumer<S> op) {
		return runOnce(() -> io.setSwerveRequest(init.get()))
				.andThen(run(() -> op.accept((S) io.getSwerveRequest())));
	}

	/** Open loop during teleop */
	public Command openLoopControl(
			DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotationSpeed) {
		JoystickVector stick = new JoystickVector();
		return drive(
						() -> new FieldCentric(),
						(request) -> {
							double rawX = xSpeed.getAsDouble();
							double rawY = ySpeed.getAsDouble();
							double rawOmega = rotationSpeed.getAsDouble();

							stick.x = rawX;
							stick.y = rawY;

							Util.applyRadialDeadband(stick, Constants.OperatorConstants.EPSILON);
							double xFancy = stick.x;
							double yFancy = stick.y;
							double rotFancy =
									MathUtil.applyDeadband(rawOmega, Constants.OperatorConstants.EPSILON);

							request
									.withVelocityX(xFancy * MAX_SPEED)
									.withVelocityY(yFancy * MAX_SPEED)
									.withRotationalRate(rotFancy * MAX_ROTATION_SPEED);
						})
				.withName("OpenLoop");
	}

	public Command autopilot(APTarget target) {
		return drive(
						() -> new FieldCentricFacingAngle(),
						(request) -> {
							APResult out = autoPilot.calculate(inputs.fieldPose, inputs.robotSpeeds, target);
							request
									.withVelocityX(out.vx())
									.withVelocityY(out.vy())
									.withTargetDirection(out.targetAngle());
						})
				.until(
						new Trigger(() -> autoPilot.atTarget(inputs.fieldPose, target))
								.debounce(DEBOUNCE_TIME.in(Seconds)));
	}

	public Command autoAlign(Pose2d pose) {
		PIDVController translationController = new PIDVController(TRANSLATION_CONSTANTS);
		ProfiledPIDVController thetaController =
				new ProfiledPIDVController(PROFILED_ROTATION_CONSTANTS);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		return drive(
						() -> {
							thetaController.setInitialSetpoint(
									inputs.fieldPose.getRotation().getRadians(),
									inputs.robotSpeeds.omegaRadiansPerSecond);
							return new ApplyFieldSpeeds();
						},
						(request) -> {
							var currentPose = inputs.fieldPose;
							var currentSpeeds = inputs.fieldSpeeds;

							// Calculate difference
							var delta = pose.getTranslation().minus(currentPose.getTranslation());

							// Magnitude target
							double vMagnitude =
									-MathUtil.clamp(
											translationController
													.setTarget(0)
													.setMeasurement(
															delta.getNorm(), // We are exactly where we are
															-Util.chassisSpeedsMagnitude(currentSpeeds)) // How fast we are going
													.getOutput(),
											-MAX_SPEED,
											MAX_SPEED);

							// The angle we are at relative to the target
							var deltaRotation = delta.getAngle();

							double rotation =
									MathUtil.clamp(
											thetaController
													.setTarget(pose.getRotation().getRadians()) // Theta target
													.setMeasurement(
															currentPose.getRotation().getRadians(),
															currentSpeeds
																	.omegaRadiansPerSecond) // We are where we are and we are as fast
													// as how
													// fast we are going
													.getOutput(),
											-MAX_ROTATION_SPEED,
											MAX_ROTATION_SPEED);

							ChassisSpeeds speeds =
									new ChassisSpeeds(
											vMagnitude * deltaRotation.getCos(), // convert from polar to rectangular
											vMagnitude * deltaRotation.getSin(),
											rotation);

							request.withSpeeds(speeds);
						})
				.until(
						new Trigger(
										() ->
												Math.abs(translationController.getError()) <= EPSILON_TRANSLATION
														&& MathUtil.isNear(thetaController.getError(), 0, EPSILON_ROTATION))
								.debounce(DEBOUNCE_TIME.in(Seconds)));
	}

	public Command trajectory(RedTrajectory trajectory) {
		var xController = new PIDVController(TRANSLATION_CONSTANTS);
		var yController = new PIDVController(TRANSLATION_CONSTANTS);
		var thetaController = new ProfiledPIDVController(PROFILED_ROTATION_CONSTANTS);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		var timer = new Timer();
		return drive(
						() -> {
							timer.start();
							thetaController.setInitialSetpoint(
									inputs.fieldPose.getRotation().getRadians(),
									inputs.fieldSpeeds.omegaRadiansPerSecond);
							return new ApplyFieldSpeeds();
						},
						(request) -> {
							var targetState = trajectory.advanceTo(timer.get());
							var currentPose = inputs.fieldPose;
							var currentSpeeds = inputs.fieldSpeeds;

							// gets the feedforwards for translation
							double vxFF = targetState.speeds.vxMetersPerSecond;
							double vyFF = targetState.speeds.vyMetersPerSecond;

							double xAccelFF = MathUtil.applyDeadband(targetState.accels.ax, MAX_ACCEL * 0.5);
							double yAccelFF = MathUtil.applyDeadband(targetState.accels.ay, MAX_ACCEL * 0.5);
							double angularAccel =
									MathUtil.applyDeadband(
											targetState.accels.alpha,
											MAX_ROTATION_ACCEL * 0.5); // what did i even want to accomplish from this

							// acceleration feedforwards
							xAccelFF += -angularAccel * targetState.pose.getRotation().getSin();
							yAccelFF += angularAccel * targetState.pose.getRotation().getCos();

							double vx =
									xController
											.setTarget(
													targetState.pose.getX(),
													targetState.speeds.vxMetersPerSecond) // Target setting
											.setMeasurement(
													currentPose.getX(),
													currentSpeeds.vxMetersPerSecond) // Measurement setting
											.getOutput(); // Output getting

							// same thing but for vy and rotation instead
							double vy =
									yController
											.setTarget(targetState.pose.getY(), targetState.speeds.vyMetersPerSecond)
											.setMeasurement(currentPose.getY(), currentSpeeds.vyMetersPerSecond)
											.getOutput();

							double rotation =
									thetaController
											.setTarget(
													targetState.pose.getRotation().getRadians(),
													targetState.speeds.omegaRadiansPerSecond)
											.setMeasurement(
													currentPose.getRotation().getRadians(),
													currentSpeeds.omegaRadiansPerSecond)
											.getOutput();

							var speeds =
									new ChassisSpeeds(
											vx + vxFF + xAccelFF * ACCELERATION_CONSTANT,
											vy + vyFF + yAccelFF * ACCELERATION_CONSTANT,
											rotation + targetState.speeds.omegaRadiansPerSecond);

							request.withSpeeds(speeds);
						})
				.until(new Trigger(() -> trajectory.isDone()).debounce(DEBOUNCE_TIME.in(Seconds)));
	}

	/** Locks the robot onto a pose. Utilizes feedforwards derived from the current chassis speeds */
	public Command headingLock(DoubleSupplier xSpeed, DoubleSupplier ySpeed, Translation2d pose) {
		JoystickVector stick = new JoystickVector();

		ProfiledPIDVController thetaController =
				new ProfiledPIDVController(
						new ProfiledPIDVConstants(
								new PIDVConstants(10.0, 0.0, 1),
								new TrapezoidProfile.Constraints(Math.PI * 16, Math.PI * 5)));
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		return drive(
						() -> new FieldCentric(),
						(request) -> {
							double rawX = xSpeed.getAsDouble();
							double rawY = ySpeed.getAsDouble();

							stick.x = rawX;
							stick.y = rawY;

							Util.applyRadialDeadband(stick, Constants.OperatorConstants.EPSILON);
							double xFancy = stick.x;
							double yFancy = stick.y;

							var state = getState();
							var delta = pose.minus(getPose().getTranslation());
							var targetDirection = delta.getAngle();

							var normSq = delta.getNorm() * delta.getNorm();
							var fieldSpeeds =
									ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, getPose().getRotation());
							var rotationalRate =
									normSq > 1e-4
											? (-delta.getX() * fieldSpeeds.vyMetersPerSecond
															+ delta.getY() * fieldSpeeds.vxMetersPerSecond)
													/ (normSq)
											: 0.0;

							var rotation =
									thetaController
											.setTarget(targetDirection.getRadians(), rotationalRate)
											.setMeasurement(
													state.Pose.getRotation().getRadians(), state.Speeds.omegaRadiansPerSecond)
											.getOutput();

							SmartDashboard.putNumber(
									"error tracking",
									MathUtil.inputModulus(
											state.Pose.getRotation().minus(targetDirection).getDegrees(), -180, 180));

							request
									.withVelocityX(xFancy * MAX_SPEED)
									.withVelocityY(yFancy * MAX_SPEED)
									.withRotationalRate(rotation);
						})
				.withName("HeadingLock");
	}

	/** Adds a vision update */
	public void addVisionUpdate(Pose2d pose, double timestamp) {
		getCtreDrive().addVisionMeasurement(pose, timestamp);
	}

	/** Adds a vision update with standard deviations */
	public void addVisionUpdate(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
		getCtreDrive().addVisionMeasurement(pose, timestamp, stdDevs);
	}

	/** Resets pose estimator to a pose */
	public void resetPose(Pose2d pose) {
		getCtreDrive().resetPose(pose);
	}

	/** A command that resets the pose */
	public Command resetPoseCommand(Pose2d pose) {
		return Commands.runOnce(() -> resetPose(pose));
	}

	/** Whether the pitch is stable */
	public boolean isPitchStable() {
		return drivetrain
								.getPigeon2()
								.getAngularVelocityYDevice()
								.getValue()
								.abs(Units.DegreesPerSecond)
						< MAX_VELOCITY_STABLE
				&& drivetrain.getPigeon2().getPitch().getValue().abs(BaseUnits.AngleUnit)
						< MAX_PITCH_STABLE;
	}

	/** Whether the roll is stable */
	public boolean isRollStable() {
		return drivetrain
								.getPigeon2()
								.getAngularVelocityXDevice()
								.getValue()
								.abs(Units.DegreesPerSecond)
						< MAX_VELOCITY_STABLE
				&& drivetrain.getPigeon2().getRoll().getValue().abs(BaseUnits.AngleUnit) < MAX_PITCH_STABLE;
	}

	/** Whether the robot is stable */
	public boolean isStable() {
		ChassisSpeeds speeds = getState().Speeds;
		return isPitchStable()
				&& isRollStable()
				&& Units.MetersPerSecond.of(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond))
						.lte(MAX_SPEED_SCORING_TRANSLATION)
				&& Units.RadiansPerSecond.of(speeds.omegaRadiansPerSecond).lte(MAX_ROTATION_SPEED_SCORING);
	}
}
