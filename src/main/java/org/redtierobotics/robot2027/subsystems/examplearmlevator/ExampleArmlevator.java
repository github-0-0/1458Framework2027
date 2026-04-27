package org.redtierobotics.robot2027.subsystems.examplearmlevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static org.redtierobotics.robot2027.subsystems.examplearmlevator.ExampleArmlevator.ArmlevatorState.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Map;
import java.util.Set;
import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachine.State;
import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachine.StateNode;
import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachineSubsystemBase;
import org.redtierobotics.robot2027.subsystems.examplearmlevator.examplearm.ExampleArm;
import org.redtierobotics.robot2027.subsystems.examplearmlevator.examplearm.ExampleArmConstants.ArmSetpoint;
import org.redtierobotics.robot2027.subsystems.examplearmlevator.exampleelevator.ExampleElevator;
import org.redtierobotics.robot2027.subsystems.examplearmlevator.exampleelevator.ExampleElevatorConstants.ElevatorSetpoint;

public class ExampleArmlevator extends UnweightedStateMachineSubsystemBase {
	public static enum ArmlevatorState implements State {
		STOW(ArmSetpoint.STOW, ElevatorSetpoint.ZERO),
		L2_PRESCORE(ArmSetpoint.L23_PRESCORE, ElevatorSetpoint.L2_PRESCORE),
		L2_POSTSCORE(ArmSetpoint.L23_POSTSCORE, ElevatorSetpoint.L2_POSTSCORE),
		L3_PRESCORE(ArmSetpoint.L23_PRESCORE, ElevatorSetpoint.L3_PRESCORE),
		L3_POSTSCORE(ArmSetpoint.L23_POSTSCORE, ElevatorSetpoint.L3_POSTSCORE),
		L4_PRESCORE(ArmSetpoint.L4_PRESCORE, ElevatorSetpoint.L4_PRESCORE),
		L4_POSTSCORE(ArmSetpoint.L4_POSTSCORE, ElevatorSetpoint.L4_POSTSCORE);

		public ArmSetpoint armSetpoint;
		public ElevatorSetpoint elevatorSetpoint;

		private ArmlevatorState(ArmSetpoint armSetpoint, ElevatorSetpoint elevatorSetpoint) {
			this.armSetpoint = armSetpoint;
			this.elevatorSetpoint = elevatorSetpoint;
		}

		@Override
		public StateNode node() {
			return new StateNode(this);
		}

		@Override
		public String toString() {
			return name();
		}
	}

	public Mechanism2d armlevator;
	public ExampleArm arm;
	public MechanismLigament2d armLigament;
	public ExampleElevator elevator;
	public MechanismLigament2d elevatorLigament;

	public ExampleArmlevator(ExampleArm arm, ExampleElevator elevator) {
		super(STOW.node(), arm, elevator);
		this.arm = arm;
		this.elevator = elevator;
		setUpStateMachine();

		armlevator = new Mechanism2d(1, 3);
		armLigament = new MechanismLigament2d("Arm", Units.inchesToMeters(28), 0);
		elevatorLigament = new MechanismLigament2d("Elevator", 0, 90);
		armlevator.getRoot("Root", 0.5, Units.inchesToMeters(5)).append(elevatorLigament);
		elevatorLigament.append(armLigament);

		SmartDashboard.putData("Armlevator", armlevator);
	}

	@Override
	protected void setUpStateMachine() {
		interconnectSingleSided(
				(from, to) ->
						() ->
								Commands.sequence(
										elevator.setpoint(((ArmlevatorState) to).elevatorSetpoint),
										arm.setpoint(((ArmlevatorState) to).armSetpoint)),
				STOW,
				L2_PRESCORE,
				L3_PRESCORE,
				L4_PRESCORE);
		connectAll((from, to) ->
						() ->
								Commands.parallel(
										elevator.setpoint(((ArmlevatorState) to).elevatorSetpoint),
										arm.setpoint(((ArmlevatorState) to).armSetpoint)), 
				Map.of(
					L2_PRESCORE, L2_POSTSCORE, 
					L3_PRESCORE, L3_POSTSCORE, 
					L4_PRESCORE, L4_POSTSCORE));
		interconnectSingleSided(
				(from, to) ->
						() ->
								Commands.parallel(
										elevator.setpoint(((ArmlevatorState) to).elevatorSetpoint),
										arm.setpoint(((ArmlevatorState) to).armSetpoint)),
				Set.of(L2_POSTSCORE, L3_POSTSCORE, L4_POSTSCORE),
				Set.of(STOW));
	}

	@Override
	public void periodic() {
		super.periodic();
		updateDisplay();
	}

	public void updateDisplay() {
		elevatorLigament.setLength(elevator.getCurrentPosition().in(Rotations));
		armLigament.setAngle(arm.getCurrentPosition().in(Degrees) - 90);
	}
}
