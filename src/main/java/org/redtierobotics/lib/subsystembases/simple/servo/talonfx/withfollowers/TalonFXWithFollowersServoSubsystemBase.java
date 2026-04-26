package org.redtierobotics.lib.subsystembases.simple.servo.talonfx.withfollowers;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.List;

import org.redtierobotics.lib.io.motor.talonfx.FollowerTalonFXMotorIO;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXInputs;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXInputsAutoLogged;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXMotorIO;
import org.redtierobotics.lib.subsystembases.simple.servo.talonfx.TalonFXServoSubsystemBase;
import org.redtierobotics.lib.util.Util;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TalonFXWithFollowersServoSubsystemBase extends TalonFXServoSubsystemBase {
	protected List<TalonFXInputs> inputsList = new ArrayList<>();
	protected List<FollowerTalonFXMotorIO> followers = new ArrayList<>();
	
	public TalonFXWithFollowersServoSubsystemBase(TalonFXMotorIO io, List<FollowerTalonFXMotorIO> followers, TalonFXWithFollowersServoSubsystemConstants constants) {
		super(io, constants);
		this.followers = followers;
		inputsList.add((TalonFXInputs) inputs);
		for (int i = 0; i < followers.size(); i++) {
			TalonFXInputsAutoLogged inputsObj = new TalonFXInputsAutoLogged();
			registerIO("/Follower/" + i, followers.get(i), inputsObj);
			inputsList.add(inputsObj);
		}
	}

	public Command resetPosition(Angle position) {
		return runOnce(() -> {
			io.resetPosition(position);
			for (FollowerTalonFXMotorIO follower : followers) {
				follower.resetPosition(position);
			}
		});
	}

	public Trigger isPositionInRange(Angle target) {
		return new Trigger(
			() -> {
				MutAngle avg = Rotations.mutable(0.0);
				for (TalonFXInputs i : inputsList) {
					avg.mut_acc(i.position);
				}
				return Util.epsilonEquals(avg, target, constants.positionEpsilon);
			});
	}

	public Trigger isVelocityInRange(AngularVelocity target) {
		return new Trigger(
			() -> {
				MutAngularVelocity avg = RotationsPerSecond.mutable(0.0);
				for (TalonFXInputs i : inputsList) {
					avg.mut_acc(i.velocity);
				}
				return Util.epsilonEquals(avg, target, constants.velocityEpsilon);
			});
	}
}
