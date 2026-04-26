package org.redtierobotics.robot2027.subsystems.examplearmlevator.examplearm;

import edu.wpi.first.wpilibj2.command.Command;

import static org.redtierobotics.robot2027.subsystems.examplearmlevator.examplearm.ExampleArmConstants.*;

import org.redtierobotics.lib.io.motor.talonfx.SimTalonFXMotorIO;
import org.redtierobotics.lib.io.motor.talonfx.TalonFXMotorIO;
import org.redtierobotics.lib.subsystembases.simple.servo.talonfx.TalonFXServoSubsystemBase;
import org.redtierobotics.lib.subsystembases.simple.servo.talonfx.TalonFXServoSubsystemConstants;
import org.redtierobotics.robot2027.Robot;
import org.redtierobotics.robot2027.subsystems.examplearmlevator.examplearm.ExampleArmConstants.ArmSetpoint;

public class ExampleArm extends TalonFXServoSubsystemBase {
	public ExampleArm() {
		super(
				Robot.isSimulation()
						? new SimTalonFXMotorIO<>(DEVICE, ARM_CONFIG, SIM_CONFIG, GEAR_RATIO)
						: new TalonFXMotorIO(DEVICE, ARM_CONFIG),
				new TalonFXServoSubsystemConstants(
						DEVICE, POSITION_EPSILON, VELOCITY_EPSILON, DEBOUNCE, ARM_CONFIG));
		io.resetPosition(STARTING_ANGLE);
	}

	public Command setpoint(ArmSetpoint setpoint) {
		return profiledMoveToPositionBlocking(setpoint.armAngle());
	}
}
