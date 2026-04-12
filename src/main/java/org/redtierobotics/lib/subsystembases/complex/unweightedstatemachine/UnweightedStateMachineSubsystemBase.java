package org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BiFunction;

import org.redtierobotics.lib.subsystembases.complex.CompositeSubsystemBase;
import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachine.State;
import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachine.StateEdge;
import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachine.StateNode;

public abstract class UnweightedStateMachineSubsystemBase extends CompositeSubsystemBase {
	protected UnweightedStateMachine stateMachine;

	public UnweightedStateMachineSubsystemBase(SubsystemBase... subsystems) {
		super(subsystems);
		setUpStateMachine();
	}

	protected abstract void setUpStateMachine();

	public void addStateTransition(State first, State second, Command edge) {
		stateMachine.connectSingleSided(first.node(), second.node(), new StateEdge(edge));
	}

	public void addStateTransitionDoubleSided(State first, State second, Command toSecond, Command toFirst) {
		stateMachine.connectDoubleSided(first.node(), second.node(), new StateEdge(toSecond), new StateEdge(toFirst));
	}

	public void interconnect(BiFunction<State, State, Command> interconnector, State... states) {
		for (int i = 0; i < states.length; i++) {
			for (int j = 0; j < states.length; j++) {
				if (i != j) {
					addStateTransitionDoubleSided(
						states[i],
						states[j], 
						interconnector.apply(states[i], states[j]), 
						interconnector.apply(states[j], states[i]));
				}
			}
		}
	}

	public Command state(State state) {
		return defer(() -> {
			var path = stateMachine.findPath(stateMachine.getState().node(), state.node());
			Command[] commands = new Command[path.size()];
			int i = 0;
			for (Pair<StateNode, StateEdge> step : path) {
				var cmd = step.getSecond().get();
				commands[i++] = cmd.andThen(runOnce(() -> stateMachine.setState(step.getFirst().get())));
			}

			return Commands.sequence(commands);
		});
	}
}
