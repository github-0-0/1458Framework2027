package org.redtierobotics.robot2027.subsystems;

import org.redtierobotics.robot2027.subsystems.drive.Drive;
import org.redtierobotics.robot2027.subsystems.examplearm.ExampleArm;

public class Subsystems {
	public final Drive drive;
	public final ExampleArm arm;

	public Subsystems() {
		drive = new Drive();
		arm = new ExampleArm();
	}
}
