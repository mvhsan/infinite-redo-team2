package org.team3128.redobot.subsystems;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.kauailabs.navx.frc.AHRS;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.control.trajectory.Trajectory;
import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
import org.team3128.redobot.subsystems.FalconDrive;
import org.team3128.common.utility.Log;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerExtreme3D;
import org.team3128.common.listener.controltypes.Button;
import org.team3128.common.listener.controltypes.POV;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.hardware.motor.LazyCANSparkMax;
import org.team3128.common.utility.units.Length;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

import org.team3128.common.generics.ThreadScheduler;
import org.team3128.common.generics.Threaded;

public class Claw extends Threaded {
    public static final Claw instance = new Claw();

    Piston pushPiston;
    boolean isPushing = false;
    boolean isDone = false;
    double passedTime;

    private Claw() {
        configPistons();
    }

    public static Claw getInstance() {
        return instance;
    }

    private void configPistons() {
        pushPiston = new Piston(1, 2);
        pushPiston.setPistonOff();
    }

    public void Push() {
        isPushing = true;
        isDone = false;
    }

    public void Retract() {
        isPushing = false;
        isDone = false;
    }

    public void update() {
        if(isPushing && !isDone) {
            pushPiston.setPistonOn();
            isDone = true;
        } else if(!isPushing && !isDone) {
            pushPiston.setPistonOff();
            isDone = true;
        }
    }
}