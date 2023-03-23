package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConsts;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveModule frontLeft;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    private SwerveModule frontRight;

    private AHRS navx;

    public SwerveSubsystem() {
        frontLeft = new SwerveModule(SwerveConsts.FL_TURN_PORT, SwerveConsts.FL_DRIVE_PORT, 
            SwerveConsts.FL_OFFSET, true);

        backLeft = new SwerveModule(SwerveConsts.BL_TURN_PORT, SwerveConsts.BL_DRIVE_PORT,
            SwerveConsts.BL_OFFSET, false);

        backRight = new SwerveModule(SwerveConsts.BR_TURN_PORT, SwerveConsts.BR_DRIVE_PORT,
            SwerveConsts.BR_OFFSET, true);

        frontRight = new SwerveModule(SwerveConsts.FR_TURN_PORT, SwerveConsts.FR_DRIVE_PORT,
            SwerveConsts.FR_OFFSET, true);

        navx = new AHRS(SPI.Port.kMXP);

    }

    public void resetEnc() {
        frontLeft.resetEncoders();
        backLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
    }

    public double feetToEncCounts(double x){
        return 18*x; //FIXME
    }

    public void resetNavx() {
        navx.zeroYaw();
    }


    public double getYawAngle() {
        return ( /* navx.getYaw() */ Math.abs(navx.getAngle()) % 360 /* 360-navx.getYaw() */ );
    }

    public double getYaw(){
        return navx.getYaw();
    }

    public double getRoll(){
        return navx.getRoll();
    }

    public double getPitch() {
        return navx.getPitch();
    }

    public double getDriveEnc() {
        return frontLeft.getDrivePosition();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(360.0 - navx.getYaw());
    }

    public void stopModules() {
        frontLeft.stop();
        backLeft.stop();
        backRight.stop();
        frontRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConsts.MAX_SPEED);
        frontLeft.setDesiredState(desiredStates[0]);
        backLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        frontRight.setDesiredState(desiredStates[3]);
    }

    public void lock() {
        SwerveModuleState fl = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(45)));
        SwerveModuleState bl = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(-45)));
        SwerveModuleState br = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(45)));
        SwerveModuleState fr = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(-45)));

        frontLeft.setAngle(fl);
        backLeft.setAngle(bl);
        backRight.setAngle(br);
        frontRight.setAngle(fr);
    }

    public void straightenWheels(){
        SwerveModuleState fl = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(0)));
        SwerveModuleState bl = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(0)));
        SwerveModuleState br = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(0)));
        SwerveModuleState fr = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(0)));

        frontLeft.setAngle(fl);
        backLeft.setAngle(bl);
        backRight.setAngle(br);
        frontRight.setAngle(fr);
    }

    public void driveForward(double speed) {
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS
                .toSwerveModuleStates(new ChassisSpeeds(-speed, 0, 0));
        setModuleStates(moduleStates);
    }

    public void driveBackward(double speed) {
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS
                .toSwerveModuleStates(new ChassisSpeeds(speed, 0, 0));
        setModuleStates(moduleStates);
    }

    public void strafeLeft(double speed) {
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS
                .toSwerveModuleStates(new ChassisSpeeds(0, -speed, 0));
        setModuleStates(moduleStates);
    }

    public void strafeRight(double speed) {
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS
                .toSwerveModuleStates(new ChassisSpeeds(0, speed, 0));
        setModuleStates(moduleStates);
    }

    public void rotateLeft(double speed) {
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS
                .toSwerveModuleStates(new ChassisSpeeds(0, 0, speed));
        setModuleStates(moduleStates);
    }

    public void rotateRight(double speed) {
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS
                .toSwerveModuleStates(new ChassisSpeeds(0, 0, -speed));
        setModuleStates(moduleStates);
    }

    // PERIODIC - runs repeatedly (like periodic from timed robot)
    @Override
    public void periodic() {
        SmartDashboard.putNumber("[S] Yaw", getYaw());
        SmartDashboard.putNumber("[S] Pitch", getPitch());
        SmartDashboard.putNumber("[S] Timer Class", Timer.getMatchTime());
    }
}
