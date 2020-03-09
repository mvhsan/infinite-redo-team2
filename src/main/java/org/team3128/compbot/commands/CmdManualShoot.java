package org.team3128.compbot.commands;

import org.team3128.common.drive.DriveCommandRunning;
import org.team3128.common.drive.DriveSignal;
import org.team3128.common.hardware.limelight.LEDMode;
import org.team3128.common.hardware.limelight.Pipeline;
import org.team3128.common.hardware.limelight.Limelight;
import org.team3128.common.hardware.limelight.LimelightData;
import org.team3128.common.hardware.limelight.LimelightKey;
import org.team3128.common.hardware.limelight.StreamMode;
import org.team3128.common.hardware.gyroscope.Gyro;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.utility.Log;
import org.team3128.common.utility.RobotMath;
import org.team3128.common.utility.datatypes.PIDConstants;
import org.team3128.common.utility.units.Angle;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.*;
import org.team3128.compbot.subsystems.Hopper.ActionState;

import com.kauailabs.navx.frc.AHRS;


import org.team3128.compbot.commands.*;

public class CmdManualShoot extends Command {
    Shooter shooter;
    Hopper hopper;
    Arm arm;

    int ballCount;

    private StateTracker stateTracker = StateTracker.getInstance();

    private Command hopperShoot, organize;

    public CmdManualShoot(Shooter shooter, Arm arm, Hopper hopper) {
        this.shooter = shooter;
        this.arm = arm;
        this.hopper = hopper;
    }

    @Override
    protected void initialize() {
        arm.setState(stateTracker.getState().targetArmState);
        shooter.setState(stateTracker.getState().targetShooterState);
        hopper.setAction(Hopper.ActionState.SHOOTING);
        shooter.driverReady = false;
        ballCount = hopper.getBallCount();
    }

    @Override
    protected void execute() {
        if (hopper.getBallCount() < ballCount) {
            ballCount = hopper.getBallCount();
            shooter.driverReady = false;
        }
        if (shooter.isReady() && shooter.driverReady) {
            // Log.info("CmdAlignShoot", "Trying to shoot ball");
            hopper.shoot();
        } else {
            hopper.unShoot();
        }
    }

    @Override
    protected boolean isFinished() {
        return hopper.isEmpty();
    }

    @Override
    protected void end() {
        shooter.setSetpoint(0);

        Log.info("CmdAlignShoot", "Command Finished.");
        hopper.setAction(Hopper.ActionState.ORGANIZING);
    }

    @Override
    protected void interrupted() {
        end();
    }
}