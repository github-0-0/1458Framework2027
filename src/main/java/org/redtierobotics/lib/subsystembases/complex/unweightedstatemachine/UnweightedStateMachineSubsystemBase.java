package org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BiFunction;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.redtierobotics.lib.subsystembases.complex.CompositeSubsystemBase;
import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachine.State;
import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachine.StateEdge;
import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachine.StateNode;

public abstract class UnweightedStateMachineSubsystemBase extends CompositeSubsystemBase {
	protected UnweightedStateMachine stateMachine;

	/** Creates a simple finite state machine subsystem */
	public UnweightedStateMachineSubsystemBase(StateNode start, SubsystemBase... subsystems) {
		super(subsystems);
		stateMachine = new UnweightedStateMachine(start);
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
	 */
	public void addStateTransition(State first, State second, Supplier<Command> edge) {
		stateMachine.connectSingleSided(first.node(), second.node(), new StateEdge(edge));
	}

	/**
	 * Adds 2 transitions between states
	 *
	 * @param first State to transition from
	 * @param second State to transition to
	 * @param toSecond Command to transition to the second state from the first
	 * @param toFirst Command to transition to the first state from the second
	 */
	public void addStateTransitionDoubleSided(
			State first, State second, Supplier<Command> toSecond, Supplier<Command> toFirst) {
		stateMachine.connectDoubleSided(
				first.node(), second.node(), new StateEdge(toSecond), new StateEdge(toFirst));
	}

	/**
	 * Quickly connect all states with a map of commands
	 *
	 * @param interconnector Maps 2 states to its command
	 * @param states The states to connect
	 */
	public void interconnect(
			BiFunction<State, State, Supplier<Command>> interconnector, State... states) {
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

	@Override
	public void periodic() {
		super.periodic();
		Logger.recordOutput(name + "/State", stateMachine.getState().toString());
	}

	/** A command that uses an available path to the target state */
	public Command state(State state) {
		return Commands.defer(
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
						},
						subsystems)
				.withName(state.toString() + ": to state");
	}
}
