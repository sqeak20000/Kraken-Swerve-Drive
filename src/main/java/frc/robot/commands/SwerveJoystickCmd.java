package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Swerve Joystick contoslled!");
    }

    @Override
    public void execute() {
        double slow = 1;

        // 1. Get real-time joystick inputs
        double xSpeed = .1;
        double ySpeed = 0;
        double turningSpeed = turningSpdFunction.get();

        System.out.print("Joystick Input: (" + xSpeed + ", " + ySpeed + ")");

        // 2. Apply deadband & square output for more precise movement at low speed
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? Math.copySign(Math.pow(xSpeed * slow, 2), xSpeed) : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? Math.copySign(Math.pow(ySpeed * slow, 2), ySpeed) : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? Math.copySign(Math.pow(turningSpeed * slow, 2), turningSpeed) : 0.0;
        swerveSubsystem.excuteJoystickRunFromField(xSpeed, ySpeed, turningSpeed);
    }

    @Override
    public void end(boolean interrupted) {

        // swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
