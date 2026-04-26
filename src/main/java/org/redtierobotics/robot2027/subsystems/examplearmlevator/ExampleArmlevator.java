package org.redtierobotics.robot2027.subsystems.examplearmlevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static org.redtierobotics.robot2027.subsystems.examplearmlevator.ExampleArmlevator.ArmlevatorState.*;

import org.redtierobotics.robot2027.subsystems.examplearmlevator.examplearm.ExampleArmConstants.ArmSetpoint;
import org.redtierobotics.robot2027.subsystems.examplearmlevator.exampleelevator.ExampleElevatorConstants.ElevatorSetpoint;

import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachine.State;
import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachine.StateNode;
import org.redtierobotics.robot2027.subsystems.examplearmlevator.examplearm.ExampleArm;
import org.redtierobotics.robot2027.subsystems.examplearmlevator.exampleelevator.ExampleElevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

import org.redtierobotics.lib.subsystembases.complex.unweightedstatemachine.UnweightedStateMachineSubsystemBase;

public class ExampleArmlevator extends UnweightedStateMachineSubsystemBase {
    public static enum ArmlevatorState implements State {
        STOW(ArmSetpoint.LOW, ElevatorSetpoint.LOW),
        PRE_SCORE(ArmSetpoint.HIGH, ElevatorSetpoint.HIGH),
        POST_SCORE(ArmSetpoint.MID, ElevatorSetpoint.MID);

        public ArmSetpoint armSetpoint;
        public ElevatorSetpoint elevatorSetpoint;

        private ArmlevatorState(ArmSetpoint armSetpoint, ElevatorSetpoint elevatorSetpoint) {
            this.armSetpoint = armSetpoint;
            this.elevatorSetpoint = elevatorSetpoint;
        }

        @Override
        public StateNode node() {
            return new StateNode(this);
        }

        @Override
        public String toString() {
            return name();
        }
    }

    public Mechanism2d armlevator;
    public ExampleArm arm;
    public MechanismLigament2d armLigament;
    public ExampleElevator elevator;
    public MechanismLigament2d elevatorLigament;

    public ExampleArmlevator(ExampleArm arm, ExampleElevator elevator) {
        super(STOW.node(), arm, elevator);
        this.arm = arm;
        this.elevator = elevator;
		setUpStateMachine();

        armlevator = new Mechanism2d(1, 3);
        armLigament = new MechanismLigament2d("Arm", 0.5, 0);
        elevatorLigament = new MechanismLigament2d("Elevator", 0, 90);
        armlevator.getRoot("Root", 0.5, 0).append(elevatorLigament);
        elevatorLigament.append(armLigament);

        SmartDashboard.putData("Armlevator", armlevator);
    }

    @Override
    protected void setUpStateMachine() {
        addStateTransition(STOW, PRE_SCORE, 
            Commands.parallel(
                arm.setpoint(PRE_SCORE.armSetpoint), 
                elevator.setpoint(PRE_SCORE.elevatorSetpoint)));
        addStateTransition(PRE_SCORE, POST_SCORE, 
            Commands.parallel(
                arm.setpoint(POST_SCORE.armSetpoint), 
                elevator.setpoint(POST_SCORE.elevatorSetpoint)));
        addStateTransition(POST_SCORE, STOW, 
            Commands.sequence(
                arm.setpoint(STOW.armSetpoint), 
                elevator.setpoint(STOW.elevatorSetpoint)));
    }

    @Override
    public void periodic() {
        super.periodic();
        updateDisplay();
    }

    public void updateDisplay() {
        elevatorLigament.setLength(elevator.getCurrentPosition().in(Rotations));
        armLigament.setAngle(arm.getCurrentPosition().in(Degrees));
    }
}
