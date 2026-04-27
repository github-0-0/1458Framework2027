package org.redtierobotics.robot2027.subsystems.examplearmlevator.examplearm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.redtierobotics.lib.sim.servo.arm.ArmSim;
import org.redtierobotics.lib.util.CanDevice;

public class ExampleArmConstants {
	public static class ArmSetpoint {
		public static final ArmSetpoint L23_PRESCORE = of(Degrees, 55);
		public static final ArmSetpoint L23_POSTSCORE = of(Degrees, 30);
		public static final ArmSetpoint L4_PRESCORE = of(Degrees, 23);
		public static final ArmSetpoint L4_POSTSCORE = of(Degrees, 0);

		public static final ArmSetpoint STOW = of(Degrees, 90);

		public final Angle armAngle;

		public static ArmSetpoint of(AngleUnit unit, double magnitude) {
			return new ArmSetpoint(unit.of(magnitude));
		}

		public ArmSetpoint(Angle armAngle) {
			this.armAngle = armAngle;
		}

		public Angle armAngle() {
			return armAngle;
		}
	}

	public static final CanDevice DEVICE = CanDevice.fromId(20);
	public static final Angle POSITION_EPSILON = Degrees.of(3);
	public static final AngularVelocity VELOCITY_EPSILON = DegreesPerSecond.of(10);
	public static final Time DEBOUNCE = Milliseconds.of(40);

	public static final AngularVelocity MAX_SPEED = DegreesPerSecond.of(300);
	public static final AngularAcceleration MAX_ACCEL = DegreesPerSecondPerSecond.of(800);

	public static final Angle MAX_ANGLE = Degrees.of(270);
	public static final Angle MIN_ANGLE = Degrees.of(-90);
	public static final Angle STARTING_ANGLE = Degrees.of(10);

	public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;
	public static final double GEAR_RATIO = 50.0;

	public static final MomentOfInertia MOI =
			MomentOfInertiaUnit.combine(Pounds.mult(InchesPerSecond).mult(Inches), RadiansPerSecond)
					.of(47);
	public static final Distance COG_LENGTH = Inches.of(5);

	public static final TalonFXConfiguration ARM_CONFIG =
			new TalonFXConfiguration()
					.withSlot0(
							new Slot0Configs()
									.withKS(0.125)
									.withKV(0.0)
									.withKP(100.0)
									.withKI(0.0)
									.withKD(0.05)
									.withKG(0.375)
									.withGravityType(GravityTypeValue.Arm_Cosine)
									.withGravityArmPositionOffset(POSITION_EPSILON))
					.withMotionMagic(
							new MotionMagicConfigs()
									.withMotionMagicAcceleration(MAX_ACCEL)
									.withMotionMagicCruiseVelocity(MAX_SPEED)
									.withMotionMagicJerk(1600))
					.withCurrentLimits(
							new CurrentLimitsConfigs()
									.withStatorCurrentLimit(60)
									.withSupplyCurrentLimit(40)
									.withStatorCurrentLimitEnable(true)
									.withSupplyCurrentLimitEnable(true))
					.withVoltage(
							new VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0))
					.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
					.withMotorOutput(new MotorOutputConfigs().withInverted(INVERTED));

	public static final ArmSim SIM_CONFIG =
			new ArmSim(
					new SingleJointedArmSim(
							DCMotor.getKrakenX60(1),
							GEAR_RATIO,
							MOI.in(KilogramSquareMeters),
							COG_LENGTH.in(Meters),
							MIN_ANGLE.in(Radians),
							MAX_ANGLE.in(Radians),
							true,
							STARTING_ANGLE.in(Radians),
							0,
							0));
}
