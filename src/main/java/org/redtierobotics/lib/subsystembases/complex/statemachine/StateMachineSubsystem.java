package org.redtierobotics.lib.subsystembases.complex.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import org.redtierobotics.lib.subsystembases.complex.statemachine.StateMachine.State;

public interface StateMachineSubsystem {

	Command state(State target);
}
