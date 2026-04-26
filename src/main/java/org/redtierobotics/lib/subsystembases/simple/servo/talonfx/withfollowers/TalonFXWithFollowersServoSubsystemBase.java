package org.redtierobotics.lib.subsystembases.simple.servo.talonfx.withfollowers;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import org.redtierobotics.lib.io.motor.talonfx.FollowerTalonFXMotorIO;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXInputs;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXInputsAutoLogged;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXMotorIO;
import org.redtierobotics.lib.subsystembases.simple.servo.talonfx.TalonFXServoSubsystemBase;
import org.redtierobotics.lib.util.Util;

public class TalonFXWithFollowersServoSubsystemBase extends TalonFXServoSubsystemBase {
	protected List<TalonFXInputs> inputsList = new ArrayList<>();
	protected List<FollowerTalonFXMotorIO> followers = new ArrayList<>();
	protected List<Boolean> inverts = new ArrayList<>();

	/**
	 * Constructs a subsystem with multiple TalonFX motors
	 *
	 * @param io The TalonFX IO object
	 * @param followers IOs for all follower TalonFX's
	 * @param config Configs for the main motor and all followers
	 */
	public TalonFXWithFollowersServoSubsystemBase(
			TalonFXMotorIO io,
			List<FollowerTalonFXMotorIO> followers,
			TalonFXWithFollowersServoSubsystemConfig config) {
		super(io, config);
		this.followers = followers;
		inputsList.add((TalonFXInputs) inputs);
		inverts.add(false);
		for (int i = 0; i < followers.size(); i++) {
			TalonFXInputsAutoLogged inputsObj = new TalonFXInputsAutoLogged();
			registerIO("/Follower/" + i, followers.get(i), inputsObj);
			inputsList.add(inputsObj);
			inverts.add(config.alignments.get(i) != MotorAlignmentValue.Aligned);
		}
	}

	/** Resets the position of all motors */
	public Command resetPosition(Angle position) {
		return runOnce(
				() -> {
					io.resetPosition(position);
					for (FollowerTalonFXMotorIO follower : followers) {
						follower.resetPosition(position);
					}
				});
	}

	/** Whether the average position of all motors is within tolerance */
	public Trigger isPositionInRange(Angle target) {
		return new Trigger(
				() -> {
					MutAngle avg = Rotations.mutable(0.0);
					for (int i = 0; i < inputsList.size(); i++) {
						if (inverts.get(i)) {
							avg.mut_minus(inputsList.get(i).position);
						} else {
							avg.mut_plus(inputsList.get(i).position);
						}
					}
					avg.mut_divide(inputsList.size());
					return Util.epsilonEquals(avg, target, config.positionEpsilon);
				});
	}

	/** Whether the average velocity of all motors is within tolerance */
	public Trigger isVelocityInRange(AngularVelocity target) {
		return new Trigger(
				() -> {
					MutAngularVelocity avg = RotationsPerSecond.mutable(0.0);
					for (int i = 0; i < inputsList.size(); i++) {
						if (inverts.get(i)) {
							avg.mut_minus(inputsList.get(i).velocity);
						} else {
							avg.mut_plus(inputsList.get(i).velocity);
						}
					}
					avg.div(inputsList.size());
					return Util.epsilonEquals(avg, target, config.velocityEpsilon);
				});
	}
}
