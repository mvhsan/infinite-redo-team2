package org.team3128.redobot.main;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.kauailabs.navx.frc.AHRS;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
import org.team3128.compbot.subsystems.FalconDrive;
import org.team3128.common.utility.Log;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.team3128.common.generics.ThreadScheduler;

public class MainDrivetrain extends NarwhalRobot {

    private DriveCommandRunning dcr;
    static FalconDrive drive = FalconDrive.getInstance();

    ExecutorService executor = Executors.newFixedThreadPool(6);
    ThreadScheduler scheduler = new ThreadScheduler();
    Thread auto;

    public Joystick joystickRight, joystickLeft;
    public ListenerManager listenerLeft, listenerRight;
    public AHRS ahrs;
    public static PowerDistributionPanel pdp;

    @Override
    protected void constructHardware() {
        // TODO Auto-generated method stub
        scheduler.schedule(drive, executor);

        dcr = new DriveCommandRunning();

        ahrs = drive.ahrs;

        joystickRight = new Joystick(1);
        listenerRight = new ListenerManager(joystickRight);
        addListenerManager(listenerRight);

        joystickLeft = new Joystick(0);
        listenerLeft = new ListenerManager(joystickLeft);
        addListenerManager(listenerLeft);

    }

    @Override
    protected void setupListeners() {
        // TODO Auto-generated method stub

        listenerRight.nameControl(ControllerExtreme3D.TWIST, "MoveTurn");
        listenerRight.nameControl(ControllerExtreme3D.JOYY, "MoveForwards");
        listenerRight.nameControl(ControllerExtreme3D.THROTTLE, "Throttle");

        listenerRight.addMultiListener(() -> {
            if (dcr.isRunning) {
                double horiz = -0.5 * listenerRight.getAxis("MoveTurn");
                double vert = -1.0 * listenerRight.getAxis("MoveForwards");
                double throttle = -1.0 * listenerRight.getAxis("Throttle");

                drive.arcadeDrive(horiz, vert, throttle, true);
            }
        }, "MoveTurn", "MoveForwards", "Throttle");

    }

    @Override
    protected void teleopPeriodic() {
        scheduler.resume();
    }

    double maxLeftSpeed = 0;
    double maxRightSpeed = 0;
    double maxSpeed = 0;
    double minLeftSpeed = 0;
    double minRightSpeed = 0;
    double minSpeed = 0;

    double currentLeftSpeed;
    double currentRightSpeed;
    double currentSpeed;
    double currentDistance;

    @Override
    protected void updateDashboard() {
        NarwhalDashboard.put("time", DriverStation.getInstance().getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());

        currentLeftSpeed = drive.getLeftSpeed();
        currentRightSpeed = drive.getRightSpeed();

        currentSpeed = drive.getSpeed();
    

        SmartDashboard.putNumber("Left Velocity", currentLeftSpeed);
        SmartDashboard.putNumber("Right Velocity", currentRightSpeed);

    }

    @Override
    protected void teleopInit() {
        scheduler.resume();
        Log.info("MainDrivetrain", "TeleopInit has started");
        dcr.isRunning = true;
    }

    @Override
    protected void autonomousInit() {
        scheduler.resume();
        drive.resetGyro();

    }

    public static void main(String... args) {
        RobotBase.startRobot(MainDrivetrain::new);
    }

    @Override
    public void endCompetition() {
        // TODO Auto-generated method stub

    }
}