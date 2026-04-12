package org.redtierobotics.lib.subsystembases.complex.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.redtierobotics.lib.subsystembases.complex.CompositeSubsystemBase;
import org.redtierobotics.lib.subsystembases.complex.statemachine.StateMachine.State;

public abstract class StateMachineSubsystemBase extends CompositeSubsystemBase
		implements StateMachineSubsystem {
	protected StateMachine stateMachine;
	protected State current;

	public StateMachineSubsystemBase(SubsystemBase... subsystems) {
		super(subsystems);
	}

	protected abstract void setUpStateMachine();

	public Command state(State state) {
		return Commands.parallel(
				(Command[]) stateMachine.findPath(current.node(), state.node()).toArray());
	}
}
