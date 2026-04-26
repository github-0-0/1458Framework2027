package org.redtierobotics.robot2027;

import static org.redtierobotics.robot2027.Robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.redtierobotics.robot2027.subsystems.examplearmlevator.ExampleArmlevator.ArmlevatorState;

public class Operator {
	public final CommandXboxController controller;

	public Operator(CommandXboxController controller) {
		this.controller = controller;
	}

	public void setDefaults() {
		subsystems.drive.setDefaultCommand(
				subsystems.drive.openLoopControl(
						() -> -controller.getLeftY(),
						() -> -controller.getLeftX(),
						() -> -controller.getRightX()));
	}

	public void bindTeleop() {
		controller.a().onTrue(subsystems.armlevator.state(ArmlevatorState.STOW));
		controller.b().onTrue(subsystems.armlevator.state(ArmlevatorState.POST_SCORE));
	}
}
