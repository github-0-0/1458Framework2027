package org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.redtierobotics.lib.graph.unweighted.UnweightedEdge;
import org.redtierobotics.lib.graph.unweighted.UnweightedGraph;
import org.redtierobotics.lib.graph.unweighted.UnweightedNode;
import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachine.StateEdge;
import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachine.StateNode;

public class UnweightedStateMachine extends UnweightedGraph<StateNode, StateEdge> {
	public static interface State {
		/** Creates a StateNode object for this state */
		public StateNode node();

		/** Name of the state */
		public String toString();
	}

	public static class StateNode extends UnweightedNode<State> {
		public StateNode(State state) {
			super(state);
		}
	}

	public static class StateEdge extends UnweightedEdge<Supplier<Command>> {
		public StateEdge(Supplier<Command> state) {
			super(state);
		}

		@Override
		public UnweightedEdge<Supplier<Command>> clone() {
			return new StateEdge(value);
		}
	}

	protected State current;

	public UnweightedStateMachine(StateNode start) {
		super(start);
		current = start.get();
	}

	public State getState() {
		return current;
	}

	public void setState(State state) {
		current = state;
	}
}
