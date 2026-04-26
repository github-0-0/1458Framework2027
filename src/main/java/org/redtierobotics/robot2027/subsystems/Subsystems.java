package org.redtierobotics.robot2027.subsystems;

import org.redtierobotics.robot2027.subsystems.drive.Drive;
import org.redtierobotics.robot2027.subsystems.examplearmlevator.ExampleArmlevator;
import org.redtierobotics.robot2027.subsystems.examplearmlevator.examplearm.ExampleArm;
import org.redtierobotics.robot2027.subsystems.examplearmlevator.exampleelevator.ExampleElevator;

public class Subsystems {
	public final Drive drive;
	public final ExampleArm arm;
	public final ExampleElevator elevator;
	public final ExampleArmlevator armlevator;

	public Subsystems() {
		drive = new Drive();
		arm = new ExampleArm();
		elevator = new ExampleElevator();
		armlevator = new ExampleArmlevator(arm, elevator);
	}
}
