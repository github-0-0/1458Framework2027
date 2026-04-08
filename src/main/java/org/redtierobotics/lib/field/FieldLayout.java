package org.redtierobotics.lib.field;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldLayout {
	// TODO: this must be tuned to the specific year's field
	public static Field2d field;
	public static final double FIELD_LENGTH;
	public static final double FIELD_WIDTH;

	public static final double APRITAG_WIDTH = Units.inchesToMeters(6.50);
	public static AprilTagFieldLayout APRILTAG_MAP;

	static {
		try {
			APRILTAG_MAP =
					AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltAndymark.m_resourceFile);
		} catch (Exception e) {
			DriverStation.reportError(e.getMessage(), false);
			APRILTAG_MAP = AprilTagLayoutGenerated.getLayout();
		}

		FIELD_LENGTH = APRILTAG_MAP.getFieldLength();
		FIELD_WIDTH = APRILTAG_MAP.getFieldWidth();
		field = new Field2d();
		SmartDashboard.putData(field);
	}
}
