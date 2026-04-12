package org.redtierobotics.lib.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** A class that represents the chassis acceleration. */
public class ChassisAccels {
	public final double ax, ay, alpha;

	/** Creates a {@code ChassisAccels} with 0 acceleration. */
	public ChassisAccels() {
		ax = 0.0;
		ay = 0.0;
		alpha = 0.0;
	}

	/**
	 * Creates a {@code ChassisAccels}.
	 *
	 * @param ax The horizontal acceleration.
	 * @param ay The vertical acceleration.
	 * @param alpha The rotational acceleration.
	 */
	public ChassisAccels(double ax, double ay, double alpha) {
		this.ax = ax;
		this.ay = ay;
		this.alpha = alpha;
	}

	/**
	 * Estimates chassis acceleration from 2 {@code ChassisSpeeds}. This usually works better when
	 * {@code dt} is lower.
	 *
	 * @param first The initial speeds.
	 * @param second The final speeds.
	 * @param dt The time difference.
	 * @return The acceleration.
	 */
	public static ChassisAccels estimate(ChassisSpeeds first, ChassisSpeeds second, double dt) {
		return new ChassisAccels(
				(second.vxMetersPerSecond - first.vxMetersPerSecond) / dt,
				(second.vyMetersPerSecond - first.vyMetersPerSecond) / dt,
				(second.omegaRadiansPerSecond - first.omegaRadiansPerSecond) / dt);
	}
}
