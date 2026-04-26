package org.redtierobotics.robot2027.subsystems.examplearmlevator.exampleelevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static org.redtierobotics.robot2027.subsystems.examplearmlevator.exampleelevator.ExampleElevatorConstants.*;

import java.util.List;

import org.redtierobotics.lib.io.motor.talonfx.FollowerTalonFXMotorIO;
import org.redtierobotics.lib.io.motor.talonfx.SimTalonFXMotorIO;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXMotorIO;
import org.redtierobotics.lib.subsystembases.simple.servo.talonfx.withfollowers.TalonFXWithFollowersServoSubsystemBase;
import org.redtierobotics.lib.subsystembases.simple.servo.talonfx.withfollowers.TalonFXWithFollowersServoSubsystemConstants;
import org.redtierobotics.robot2027.Robot;
import org.redtierobotics.robot2027.subsystems.examplearmlevator.exampleelevator.ExampleElevatorConstants.ElevatorSetpoint;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class ExampleElevator extends TalonFXWithFollowersServoSubsystemBase {
	public ExampleElevator() {
		super(
				Robot.isSimulation()
						? new SimTalonFXMotorIO<>(LEFT_MOTOR, ELEVATOR_CONFIG, SIM_CONFIG, GEAR_RATIO)
						: new TalonFXMotorIO(LEFT_MOTOR, ELEVATOR_CONFIG),
				List.of(new FollowerTalonFXMotorIO(RIGHT_MOTOR, LEFT_MOTOR, MotorAlignmentValue.Opposed)),
				new TalonFXWithFollowersServoSubsystemConstants(
						LEFT_MOTOR, List.of(RIGHT_MOTOR), POSITION_EPSILON, VELOCITY_EPSILON, DEBOUNCE, ELEVATOR_CONFIG, List.of(MotorAlignmentValue.Opposed)));
		io.resetPosition(Rotations.of(STARTING_HEIGHT.in(Meters)));
		for (FollowerTalonFXMotorIO follower : followers) {
			follower.resetPosition(Rotations.of(STARTING_HEIGHT.in(Meters)));
		}
	}

	public Command setpoint(ElevatorSetpoint setpoint) {
		return profiledMoveToPositionBlocking(setpoint.toAngle());
	}

	public Command resetHeight(Distance height) {
		return resetPosition(Rotations.of(height.in(Meters)));
	}
	
	// TODO: distance unit getCurrentHeight, etc
}
