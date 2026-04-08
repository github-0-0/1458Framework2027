package org.redtierobotics.lib.control;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** A class containing controller constants. */
public class ControlConstants {
	/** Constants used for a {@link PIDVController} */
	public static class PIDVConstants {
		public double kP;
		public double kI;
		public double kD;
		public double dT = 0.020;

		public PIDVConstants(double p, double i, double d) {
			this.kP = p;
			this.kI = i;
			this.kD = d;
		}
	}

	/** Constants used for a {@link ProfiledPIDVController} */
	public static class ProfiledPIDVConstants {
		public double kP;
		public double kI;
		public double kD;
		public double dT = 0.020;
		public TrapezoidProfile.Constraints constraints;

		public ProfiledPIDVConstants(
				double p, double i, double d, TrapezoidProfile.Constraints constraints) {
			this.kP = p;
			this.kI = i;
			this.kD = d;
			this.constraints = constraints;
		}

		public ProfiledPIDVConstants(
				PIDVConstants constants, TrapezoidProfile.Constraints constraints) {
			this(constants.kP, constants.kI, constants.kD, constraints);
		}

		public PIDVConstants getPIDVConstants() {
			return new PIDVConstants(kP, kI, kD);
		}
	}
}
