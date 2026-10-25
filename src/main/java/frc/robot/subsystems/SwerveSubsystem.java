package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveConfiguration;
import frc.robot.subsystems.drive.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final SwerveDriveKinematics m_kinematics;

    private final Field2d m_field = new Field2d();
    private Pose2d m_simPose = new Pose2d(2.0, 2.0, new Rotation2d());
    private ChassisSpeeds m_currentSpeeds = new ChassisSpeeds();

    public SwerveSubsystem() {
        frontLeft = new SwerveModule(SwerveConfiguration.FRONT_LEFT);
        frontRight = new SwerveModule(SwerveConfiguration.FRONT_RIGHT);
        backLeft = new SwerveModule(SwerveConfiguration.BACK_LEFT);
        backRight = new SwerveModule(SwerveConfiguration.BACK_RIGHT);

        m_kinematics = new SwerveDriveKinematics(
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        );

        SmartDashboard.putData("Field", m_field);
    }

    public SwerveModule[] getModules() {
        return new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
    }

    public void robotInit() {
        frontLeft.init();
        frontRight.init();
        backLeft.init();
        backRight.init();
    }

    public void apply(ChassisSpeeds speeds) {
        m_currentSpeeds = speeds;
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
        frontLeft.apply(states[0]);
        frontRight.apply(states[1]);
        backLeft.apply(states[2]);
        backRight.apply(states[3]);
    }

    @Override
    public void periodic() {
        // Integrate speeds in sim over standard 20ms robot loop
        double dt = 0.02;
        Rotation2d newHeading = m_simPose.getRotation().plus(new Rotation2d(m_currentSpeeds.omegaRadiansPerSecond * dt));
        double newX = m_simPose.getX() + (m_currentSpeeds.vxMetersPerSecond * dt);
        double newY = m_simPose.getY() + (m_currentSpeeds.vyMetersPerSecond * dt);
        m_simPose = new Pose2d(newX, newY, newHeading);

        m_field.setRobotPose(m_simPose);
    }
}
