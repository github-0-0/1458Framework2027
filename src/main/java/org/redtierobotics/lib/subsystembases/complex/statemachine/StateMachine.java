package org.redtierobotics.lib.subsystembases.complex.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import org.redtierobotics.lib.graph.unweighted.UnweightedEdge;
import org.redtierobotics.lib.graph.unweighted.UnweightedGraph;
import org.redtierobotics.lib.graph.unweighted.UnweightedNode;
import org.redtierobotics.lib.subsystembases.complex.statemachine.StateMachine.StateEdge;
import org.redtierobotics.lib.subsystembases.complex.statemachine.StateMachine.StateNode;

public class StateMachine extends UnweightedGraph<StateNode, StateEdge> {
	public static interface State {
		public StateNode node();
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

	public StateMachine(StateNode start) {
		super(start);
	}
}
