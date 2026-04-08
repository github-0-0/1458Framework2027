package org.redtierobotics.robot2027.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.redtierobotics.robot2027.auto.AutoSelector.Auto;

public final class AutoRoutines {
	@Auto(name = "Hello World")
	public static Command helloWorldAuto = Commands.print("Hello world");
}
