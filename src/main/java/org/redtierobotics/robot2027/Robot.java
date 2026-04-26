package org.redtierobotics.robot2027;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.redtierobotics.robot2027.auto.AutoSelector;
import org.redtierobotics.robot2027.subsystems.Subsystems;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@SuppressWarnings("unused")
public class Robot extends LoggedRobot {
	private static final CommandScheduler commandScheduler = CommandScheduler.getInstance();
	private AutoSelector autoChooser;
	private Command autoCommand;
	public static final Subsystems subsystems = new Subsystems();
	public static final Operator operator = new Operator(new CommandXboxController(0));

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	public Robot() {
		boolean replay = Logger.hasReplaySource();
		boolean usbPresent = new java.io.File("/u").exists();
		Logger.recordMetadata("UsbPresent", Boolean.toString(usbPresent));

		String dir = usbPresent ? "/u/logs" : "/home/lvuser/logs";

		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

		switch (BuildConstants.DIRTY) {
			case 0:
				Logger.recordMetadata("GitDirty", "All changes committed");
				break;
			case 1:
				Logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;
			default:
				Logger.recordMetadata("GitDirty", "Unknown");
				break;
		}

		if (RobotBase.isReal()) {
			if (!DriverStation.isFMSAttached()) {
				Logger.addDataReceiver(new NT4Publisher());
			}
			Logger.addDataReceiver(new WPILOGWriter(dir));
		} else if (replay) {
			setUseTiming(false);
			String logPath = LogFileUtil.findReplayLog();
			Logger.setReplaySource(new WPILOGReader(logPath));
			Logger.addDataReceiver(new WPILOGWriter());
		} else if (RobotBase.isSimulation()) {
			Logger.addDataReceiver(new NT4Publisher());
			Logger.addDataReceiver(new WPILOGWriter());
		}

		Logger.start();
		if (!Logger.hasReplaySource()) {
			RobotController.setTimeSource(RobotController::getFPGATime);
		}

		autoChooser = new AutoSelector();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		commandScheduler.run();
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {}

	/** This function is called periodically during disabled. */
	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	/** This autonomous runs the autonomous command selected. */
	@Override
	public void autonomousInit() {
		autoCommand = autoChooser.getAuto();
		if (autoCommand != null) {
			commandScheduler.schedule(autoCommand);
		} else {
			DriverStation.reportWarning("Tried to schedule a null auto", false);
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {}

	/** This function is called when autonomous mode ends. */
	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running.
		if (autoCommand != null) {
			autoCommand.cancel();
		}

		operator.bindTeleop();
		operator.setDefaults();
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}
}
