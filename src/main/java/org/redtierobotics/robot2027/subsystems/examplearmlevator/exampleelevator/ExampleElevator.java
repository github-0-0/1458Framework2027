package org.redtierobotics.robot2027.subsystems.examplearmlevator.exampleelevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static org.redtierobotics.robot2027.subsystems.examplearmlevator.exampleelevator.ExampleElevatorConstants.*;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import org.redtierobotics.lib.io.motor.talonfx.FollowerTalonFXMotorIO;
import org.redtierobotics.lib.io.motor.talonfx.SimFollowerTalonFXMotorIO;
import org.redtierobotics.lib.io.motor.talonfx.SimTalonFXMotorIO;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXMotorIO;
import org.redtierobotics.lib.sim.servo.elevator.ElevatorSim;
import org.redtierobotics.lib.subsystembases.simple.servo.talonfx.withfollowers.TalonFXWithFollowersServoSubsystemBase;
import org.redtierobotics.lib.subsystembases.simple.servo.talonfx.withfollowers.TalonFXWithFollowersServoSubsystemConfig;
import org.redtierobotics.robot2027.Robot;
import org.redtierobotics.robot2027.subsystems.examplearmlevator.exampleelevator.ExampleElevatorConstants.ElevatorSetpoint;

public class ExampleElevator extends TalonFXWithFollowersServoSubsystemBase {
	@SuppressWarnings("unchecked")
	public ExampleElevator() {
		super(
				Robot.isSimulation()
						? new SimTalonFXMotorIO<>(LEFT_MOTOR, ELEVATOR_CONFIG, SIM_CONFIG)
						: new TalonFXMotorIO(LEFT_MOTOR, ELEVATOR_CONFIG),
				List.of(
						Robot.isSimulation()
								? new SimFollowerTalonFXMotorIO(
										RIGHT_MOTOR, LEFT_MOTOR, MotorAlignmentValue.Opposed, ELEVATOR_CONFIG)
								: new FollowerTalonFXMotorIO(
										RIGHT_MOTOR, LEFT_MOTOR, MotorAlignmentValue.Opposed, ELEVATOR_CONFIG)),
				new TalonFXWithFollowersServoSubsystemConfig(
						LEFT_MOTOR,
						List.of(RIGHT_MOTOR),
						POSITION_EPSILON,
						VELOCITY_EPSILON,
						DEBOUNCE,
						ELEVATOR_CONFIG,
						List.of(MotorAlignmentValue.Opposed)));
		io.resetPosition(Rotations.of(STARTING_HEIGHT.in(Meters)));
		for (FollowerTalonFXMotorIO follower : followers) {
			follower.resetPosition(Rotations.of(STARTING_HEIGHT.in(Meters)));
			if (Robot.isSimulation()) {
				((SimFollowerTalonFXMotorIO) follower).setLeaderIO((SimTalonFXMotorIO<ElevatorSim>) io);
			}
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
