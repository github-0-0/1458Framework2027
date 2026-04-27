package org.redtierobotics.robot2027;

import static org.redtierobotics.robot2027.Robot.subsystems;
import static org.redtierobotics.robot2027.subsystems.examplearmlevator.ExampleArmlevator.ArmlevatorState.*;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
		controller.a().onTrue(subsystems.armlevator.state(STOW));
		controller.b().onTrue(subsystems.armlevator.state(L2_POSTSCORE));
		controller.x().onTrue(subsystems.armlevator.state(L3_POSTSCORE));
		controller.y().onTrue(subsystems.armlevator.state(L4_POSTSCORE));
	}
}
