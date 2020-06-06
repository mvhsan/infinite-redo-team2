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

public class Forklift extends Threaded {

    public static enum ForkliftState {
        ZERO(0 * Length.in), // forklift is all the way down
        LOW(20 * Length.in), // lol idk
        MID(40 * Length.in), // something mediocre
        HIGH(60 * Length.in); // top of the mountain I have no idea what I'm saying

        public double height;

        private ForkliftState(double target) {
            this.height = target;
        }
    }
    
    public static final Forklift instance = new Forklift();

    public LazyCANSparkMax FORKLIFT_MOTOR_LEADER, FORKLIFT_MOTOR_FOLLOWER;
    public DigitalInput LIMIT_SWITCH;
    public double setpoint;
    public ForkliftState FORKLIFT_STATE;
    public CANEncoder LEADER_ENCODER;

    public double current, accumulator, error, prevError, output;
    public int plateauCount;

    public static Forklift getInstance() {
        return instance;
    }

    private Forklift() {
        configMotors();
        configSensors();
        configEncoders();
        setState(ForkliftState.ZERO);
    }

    private void configMotors() {
        FORKLIFT_MOTOR_LEADER = new LazyCANSparkMax(Constants.ForkliftConstants.FORKLIFT_MOTOR_LEADER_ID, MotorType.kBrushless);
        FORKLIFT_MOTOR_FOLLOWER = new LazyCANSparkMax(Constants.ForkliftConstants.FORKLIFT_MOTOR_FOLLOWER_ID, MotorType.kBrushless);
        FORKLIFT_MOTOR_FOLLOWER.follow(FORKLIFT_MOTOR_LEADER, false);
    }

    private void configSensors() {
        LIMIT_SWITCH = new DigitalInput(Constants.ForkliftConstants.FORKLIFT_LIMIT_SWITCH_ID);
    }

    private void configEncoders() {
        LEADER_ENCODER = FORKLIFT_MOTOR_LEADER.getEncoder();
    }

    private void setSetpoint(double desiredPos) {
        setpoint = desiredPos;
    }

    public void setState(ForkliftState targetState) {
        FORKLIFT_STATE = targetState;
        setSetpoint(targetState.height);
    }

    public double getEncoderPos() {
        return LEADER_ENCODER.getPosition();
    }

    public boolean getLimitStatus() {
        return !LIMIT_SWITCH.get();
    }

    public double getHeight() {
        return (((getEncoderPos() / Constants.MechanismConstants.ENCODER_RESOLUTION_PER_ROTATION)
                / Constants.ForkliftConstants.RATIO) * 360);
    }

    public void update() {
        if (setpoint > Constants.ForkliftConstants.MAX_HEIGHT) {
            setpoint = Constants.ForkliftConstants.MAX_HEIGHT;
        }

        if (setpoint < 0) {
            setpoint = 0;
        }


        if (getLimitStatus()) {
            LEADER_ENCODER.setPosition(0);
        }
        
        // if (FORKLIFT_STATE.height != setpoint) {
        //     Log.info("FORKLIFT", "Setpoint override (setpoint has been set without using ForkliftState)");
        // }
        
        current = getHeight();
        error = setpoint - current;
        accumulator += error * Constants.MechanismConstants.DT;
        if (accumulator > Constants.ForkliftConstants.SATURATION_LIMIT) {
            accumulator = Constants.ForkliftConstants.SATURATION_LIMIT;
        } else if (accumulator < -Constants.ForkliftConstants.SATURATION_LIMIT) {
            accumulator = -Constants.ForkliftConstants.SATURATION_LIMIT;
        }
        double kP_term = Constants.ForkliftConstants.FORKLIFT_PID.kP * error;
        double kI_term = Constants.ForkliftConstants.FORKLIFT_PID.kI * accumulator;
        double kD_term = Constants.ForkliftConstants.FORKLIFT_PID.kD * (error - prevError) / Constants.MechanismConstants.DT;

        double voltage_output = Constants.ForkliftConstants.FEEDFORWARD_POWER + kI_term + kD_term;
        double voltage = RobotController.getBatteryVoltage();

        output = voltage_output / voltage;
        if (output > 1) {
            output = 1;
        } else if (output < -1) {
            output = -1;
        }

        if (Math.abs(error) < Constants.ForkliftConstants.THRESHOLD) {
            plateauCount++;
        } else {
            plateauCount = 0;
        }

        if((setpoint == 0) && !getLimitStatus()) {
            output = Constants.ForkliftConstants.ZEROING_POWER;
        } else if((setpoint == 0) && getLimitStatus()) {
            output = 0;
        }

        FORKLIFT_MOTOR_LEADER.set(output);

        prevError = error;
    }
}