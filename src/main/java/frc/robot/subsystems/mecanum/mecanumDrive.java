package frc.robot.subsystems.mecanum;

public class mecanumDrive {
    private final mecanumDriveIO io;
    private final mecanumDriveIOInputsAutoLogged inputs = new mecanumDriveIOInputsAutoLogged();

    public mecanumDrive(mecanumDriveIO io){
        this.io = io;
    }

    public void driveVolts(double leftFrontVolts, double leftBackVolts, double rightFrontVolts, double rightBackVolts){
        io.setVoltage(leftFrontVolts,leftBackVolts,rightFrontVolts,rightBackVolts);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
    }
}
