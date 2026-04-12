package org.redtierobotics.lib.trajectory;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class TrajectoryLoader {
	public final RobotConfig config;

	public TrajectoryLoader(RobotConfig config) {
		this.config = config;
	}

	/**
	 * Loads a trajectory from the deploy directory.
	 *
	 * @param type The type of trajectory to load.
	 * @param fileName The file name (without the extension).
	 * @return The trajectory.
	 */
	public Optional<RedTrajectory> loadAutoTrajectory(
			RedTrajectory.TrajectoryType type, String fileName) {
		switch (type) {
			case CHOREO:
				Optional<? extends Trajectory<?>> load = Choreo.loadTrajectory(fileName);
				if (load.isPresent()) {
					Trajectory<?> trajectory = load.get();
					if (trajectory.samples().get(0) instanceof SwerveSample) {
						@SuppressWarnings("unchecked")
						Trajectory<SwerveSample> casted = (Trajectory<SwerveSample>) trajectory;
						return Optional.of(new RedTrajectory(casted, true));
					} else {
						DriverStation.reportWarning(fileName + " is not a swerve trajectory!", false);
						return Optional.empty();
					}
				} else {
					DriverStation.reportWarning(fileName + " is not a valid trajectory!", false);
					return Optional.empty();
				}
			case PATHPLANNER:
				try {
					PathPlannerPath path = PathPlannerPath.fromPathFile(fileName);
					Optional<PathPlannerTrajectory> traj = path.getIdealTrajectory(config);
					if (traj.isPresent()) {
						return Optional.of(new RedTrajectory(traj.get(), true, path.name));
					} else {
						DriverStation.reportWarning(fileName + " is not a valid trajectory!", false);
						return Optional.empty();
					}
				} catch (Exception e) {
					DriverStation.reportWarning(
							fileName + " is not a valid trajectory! " + e.getClass().getSimpleName(), false);
					return Optional.empty();
				}
			default:
				DriverStation.reportWarning(type.name() + " is not a trajectory type!", false);
				return Optional.empty();
		}
	}
}
