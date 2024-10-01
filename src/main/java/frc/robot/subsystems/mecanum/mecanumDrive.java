package frc.robot.subsystems.mecanum;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class mecanumDrive {
    private final mecanumDriveIO io;
    private final mecanumDriveIOInputsAutoLogged inputs = new mecanumDriveIOInputsAutoLogged();

    public mecanumDrive(mecanumDriveIO io){
        this.io = io;
    }

    public void driveVolts(double leftFrontVolts, double leftBackVolts, double rightFrontVolts, double rightBackVolts){
        io.setVoltage(leftFrontVolts,leftBackVolts,rightFrontVolts,rightBackVolts);
    }

    public void drive(double xSpeed, double ySpeed, double zRotation){
        var speeds = MecanumDrive.driveCartesianIK(xSpeed, ySpeed, zRotation);
        io.setVoltage(speeds.frontLeft,speeds.rearLeft,speeds.frontRight,speeds.rearRight);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Drive",inputs);
    }
}
