package org.redtierobotics.lib.subsystembases.complex.weightedstatemachine;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BiFunction;
import java.util.function.Supplier;
import org.redtierobotics.lib.subsystembases.complex.CompositeSubsystemBase;
import org.redtierobotics.lib.subsystembases.complex.weightedstatemachine.WeightedStateMachine.State;
import org.redtierobotics.lib.subsystembases.complex.weightedstatemachine.WeightedStateMachine.StateEdge;
import org.redtierobotics.lib.subsystembases.complex.weightedstatemachine.WeightedStateMachine.StateNode;

public abstract class WeightedStateMachineSubsystemBase extends CompositeSubsystemBase {
	protected WeightedStateMachine stateMachine;

	/** Creates an optimized finite state machine subsystem */
	public WeightedStateMachineSubsystemBase(StateNode start, SubsystemBase... subsystems) {
		super(subsystems);
		stateMachine = new WeightedStateMachine(start);
	}

	/**
	 * Sets up the state machine. Use {@link #addStateTransition(State, State, Supplier)}, {@link
	 * #addStateTransitionDoubleSided(State, State, Supplier, Supplier)}, and {@link
	 * #interconnect(BiFunction, State...)}
	 */
	protected abstract void setUpStateMachine();

	/**
	 * Adds a transition between states
	 *
	 * @param first State to transition from
	 * @param second State to transition to
	 * @param toSecond Command to transition between them
	 * @param timeEst Time estimate for the command
	 */
	public void addStateTransition(
			State first, State second, Supplier<Command> edge, double timeEst) {
		stateMachine.connectSingleSided(first.node(), second.node(), new StateEdge(edge, timeEst));
	}

	/**
	 * Adds 2 transitions between states
	 *
	 * @param first State to transition from
	 * @param second State to transition to
	 * @param toSecond Command to transition to the second state from the first
	 * @param toFirst Command to transition to the first state from the second
	 * @param timeToSecond Time estimate for the first command
	 * @param timeToFirst Time estimate for the second command
	 */
	public void addStateTransitionDoubleSided(
			State first,
			State second,
			Supplier<Command> toSecond,
			Supplier<Command> toFirst,
			double timeToSecond,
			double timeToFirst) {
		stateMachine.connectDoubleSided(
				first.node(),
				second.node(),
				new StateEdge(toSecond, timeToSecond),
				new StateEdge(toFirst, timeToFirst));
	}

	/**
	 * Quickly connect all states with a map of commands
	 *
	 * @param interconnector Maps 2 states to its command and time estimate
	 * @param states The states to connect
	 */
	public void interconnect(
			BiFunction<State, State, Pair<Supplier<Command>, Double>> interconnector, State... states) {
		for (int i = 0; i < states.length; i++) {
			for (int j = 0; j < states.length; j++) {
				if (i != j) {
					addStateTransitionDoubleSided(
							states[i],
							states[j],
							interconnector.apply(states[i], states[j]).getFirst(),
							interconnector.apply(states[j], states[i]).getFirst(),
							interconnector.apply(states[i], states[j]).getSecond(),
							interconnector.apply(states[j], states[i]).getSecond());
				}
			}
		}
	}

	/**
	 * Quickly connect all states with a map of commands
	 *
	 * @param interconnector Maps 2 states to its command
	 * @param states The states to connect
	 */
	public void interconnectSingleSided(
			BiFunction<State, State, Pair<Supplier<Command>, Double>> interconnector,
			State from,
			State... states) {
		for (int i = 0; i < states.length; i++) {
			addStateTransition(
					from,
					states[i],
					interconnector.apply(from, states[i]).getFirst(),
					interconnector.apply(from, states[i]).getSecond());
		}
	}

	/** A command that uses the fastest available path to the target state */
	public Command state(State state) {
		return defer(
				() -> {
					var path = stateMachine.findPath(stateMachine.getState().node(), state.node());
					if (path == null) {
						DriverStation.reportWarning(
								"No path found between "
										+ stateMachine.current.toString()
										+ " and "
										+ state.toString(),
								true);
						return Commands.none();
					}

					Command[] commands = new Command[path.size()];
					int i = 0;
					for (Pair<StateNode, StateEdge> step : path) {
						var cmd = step.getSecond().get().get();
						commands[i++] =
								cmd.andThen(runOnce(() -> stateMachine.setState(step.getFirst().get())));
					}

					return Commands.sequence(commands);
				});
	}
}
