package org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine;

import edu.wpi.first.wpilibj2.command.Command;
import org.redtierobotics.lib.graph.unweighted.UnweightedEdge;
import org.redtierobotics.lib.graph.unweighted.UnweightedGraph;
import org.redtierobotics.lib.graph.unweighted.UnweightedNode;
import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachine.StateEdge;
import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachine.StateNode;

public class UnweightedStateMachine extends UnweightedGraph<StateNode, StateEdge> {
	public static interface State {
		public StateNode node();
		public String toString();
	}

	public static class StateNode extends UnweightedNode<State> {
		public StateNode(State state) {
			super(state);
		}
	}

	public static class StateEdge extends UnweightedEdge<Command> {
		public StateEdge(Command state) {
			super(state);
		}

		@Override
		public UnweightedEdge<Command> clone() {
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
