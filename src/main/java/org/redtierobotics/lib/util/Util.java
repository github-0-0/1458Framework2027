package org.redtierobotics.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

/** Contains basic functions that are used often. */
public class Util {
	public static class JoystickVector {
		public double x;
		public double y;

		public double getNorm() {
			return Math.sqrt(x * x + y * y);
		}

		public JoystickVector toZero() {
			x = 0.0;
			y = 0.0;
			return this;
		}
	}

	/** Circular deadbanding */
	public static JoystickVector applyRadialDeadband(JoystickVector stick, double deadband) {
		double mag = stick.getNorm();
		if (mag <= deadband) return stick.toZero();
		double scaledMag = (mag - deadband) / (1.0 - deadband);
		double ux = stick.x / mag;
		double uy = stick.y / mag;
		stick.x = ux * scaledMag;
		stick.y = uy * scaledMag;
		return stick;
	}

	/** Checks if 2 ChassisSpeeds is near each other */
	public static boolean chassisSpeedsEpsilonEquals(
			ChassisSpeeds speed1, ChassisSpeeds other, double epsilon) {
		return MathUtil.isNear(speed1.vxMetersPerSecond, other.vxMetersPerSecond, epsilon)
				&& MathUtil.isNear(speed1.vyMetersPerSecond, other.vyMetersPerSecond, epsilon)
				&& MathUtil.isNear(speed1.omegaRadiansPerSecond, other.omegaRadiansPerSecond, epsilon);
	}

	/** Returns the magnitude of a ChassisSpeeds */
	public static double chassisSpeedsMagnitude(ChassisSpeeds other) {
		return Math.sqrt(
				other.vxMetersPerSecond * other.vxMetersPerSecond
						+ other.vyMetersPerSecond * other.vyMetersPerSecond);
	}

	/** A generic epsilonEquals for measures */
	public static <T extends Unit> boolean epsilonEquals(
			Measure<T> actual, Measure<T> target, Measure<T> epsilon) {
		double actualUnits = actual.baseUnitMagnitude();
		double targetUnits = target.baseUnitMagnitude();
		double epsilonUnits = epsilon.baseUnitMagnitude();
		return MathUtil.isNear(actualUnits, targetUnits, epsilonUnits);
	}
}
