package org.redtierobotics.robot2027.subsystems.examplearmlevator.exampleelevator;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import org.redtierobotics.lib.sim.servo.elevator.ElevatorSim;
import org.redtierobotics.lib.util.CanDevice;

public class ExampleElevatorConstants {
	public static class ElevatorSetpoint {
		public static final ElevatorSetpoint HIGH = new ElevatorSetpoint(Inches.of(50));
		public static final ElevatorSetpoint MID = new ElevatorSetpoint(Inches.of(30));
		public static final ElevatorSetpoint LOW = new ElevatorSetpoint(Inches.of(0));

		public final Distance elevatorHeight;
		public final Angle rotations;

		public ElevatorSetpoint(Distance elevatorHeight) {
			this.elevatorHeight = elevatorHeight;
			this.rotations = Rotations.of(elevatorHeight.in(Meters));
		}

		public Angle toAngle() {
			return rotations;
		}
	}

	public static final CanDevice LEFT_MOTOR = CanDevice.fromId(30);
	public static final CanDevice RIGHT_MOTOR = CanDevice.fromId(31);

	public static final Distance LINEAR_POSITION_EPSILON = Centimeters.of(2);
	public static final LinearVelocity LINEAR_VELOCITY_EPSILON = MetersPerSecond.of(0.1);

	public static final Angle POSITION_EPSILON = Rotations.of(LINEAR_POSITION_EPSILON.in(Meters));
	public static final AngularVelocity VELOCITY_EPSILON =
			RotationsPerSecond.of(LINEAR_VELOCITY_EPSILON.in(MetersPerSecond));
	public static final Time DEBOUNCE = Milliseconds.of(40);

	public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(5);
	public static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(20);

	public static final Distance MAX_HEIGHT = Inches.of(67);
	public static final Distance MIN_HEIGHT = Inches.of(0);
	public static final Distance STARTING_HEIGHT = Inches.of(0);

	public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
	public static final double GEAR_RATIO = 10.0;

	public static final Mass CARRIAGE_MASS = Pounds.of(10);
	public static final Distance DRUM_RADIUS = Inches.of(1);

	public static final TalonFXConfiguration ELEVATOR_CONFIG =
			new TalonFXConfiguration()
					.withSlot0(
							new Slot0Configs()
									.withKS(0.125)
									.withKV(0.0)
									.withKP(60.0)
									.withKI(0.0)
									.withKD(0.05)
									.withKG(0.375)
									.withGravityType(GravityTypeValue.Elevator_Static))
					.withMotionMagic(
							new MotionMagicConfigs()
									.withMotionMagicCruiseVelocity(
											RotationsPerSecond.of(MAX_SPEED.in(MetersPerSecond)))
									.withMotionMagicAcceleration(
											RotationsPerSecondPerSecond.of(MAX_ACCEL.in(MetersPerSecondPerSecond)))
									.withMotionMagicJerk(1600))
					.withCurrentLimits(
							new CurrentLimitsConfigs()
									.withStatorCurrentLimit(60)
									.withSupplyCurrentLimit(40)
									.withStatorCurrentLimitEnable(true)
									.withSupplyCurrentLimitEnable(true))
					.withVoltage(
							new VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0))
					.withFeedback(
							new FeedbackConfigs()
									.withSensorToMechanismRatio(GEAR_RATIO / (DRUM_RADIUS.in(Meters) * Math.PI * 2)))
					.withMotorOutput(new MotorOutputConfigs().withInverted(INVERTED));

	public static final ElevatorSim SIM_CONFIG =
			new ElevatorSim(
					new edu.wpi.first.wpilibj.simulation.ElevatorSim(
							DCMotor.getKrakenX60(2),
							GEAR_RATIO,
							CARRIAGE_MASS.in(Pounds),
							DRUM_RADIUS.in(Meters),
							MIN_HEIGHT.in(Meters),
							MAX_HEIGHT.in(Meters),
							true,
							STARTING_HEIGHT.in(Meters),
							0,
							0));
}
