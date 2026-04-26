package org.redtierobotics.lib.subsystembases.complex.weightedstatemachine;

import edu.wpi.first.wpilibj2.command.Command;
import org.redtierobotics.lib.graph.weighted.WeightedEdge;
import org.redtierobotics.lib.graph.weighted.WeightedGraph;
import org.redtierobotics.lib.graph.weighted.WeightedNode;
import org.redtierobotics.lib.subsystembases.complex.weightedstatemachine.WeightedStateMachine.StateEdge;
import org.redtierobotics.lib.subsystembases.complex.weightedstatemachine.WeightedStateMachine.StateNode;

public class WeightedStateMachine extends WeightedGraph<StateNode, StateEdge> {
	public static interface State {
		public StateNode node();
	}

	public static class StateNode extends WeightedNode<State> {
		public StateNode(State state) {
			super(state);
		}
	}

	public static class StateEdge extends WeightedEdge<Command> {
		public StateEdge(Command state, double timeEstSec) {
			super(state, timeEstSec);
		}

		@Override
		public WeightedEdge<Command> clone() {
			return new StateEdge(value, weight);
		}
	}

	protected State current;

	public WeightedStateMachine(StateNode start) {
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
