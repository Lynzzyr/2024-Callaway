package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;

// 5409: The Chargers
// http://github.com/FRC5409

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.kCANID;
import frc.robot.Constants.kDrivetrain;

public class Drivetrain extends SubsystemBase {

    private static Drivetrain instance = null;

    // Motors
    private final CANSparkFlex motorFL;
    private final CANSparkFlex motorFR;
    private final CANSparkFlex motorBL;
    private final CANSparkFlex motorBR;

    // Encoders
    private final RelativeEncoder encoderFL;
    private final RelativeEncoder encoderFR;
    private final RelativeEncoder encoderBL;
    private final RelativeEncoder encoderBR;

    // Other stuff
    private final Pigeon2 pigeon;
    private final DifferentialDrive ddrive;

    // SysID
    MutableMeasure<Voltage> sysIdAppliedVoltage;
    MutableMeasure<Distance> sysIdDistance;
    MutableMeasure<Velocity<Distance>> sysIdVelocity;

    private final SysIdRoutine sysIdRoutine;

    private Drivetrain() {
        // Motors
        motorFL = new CANSparkFlex(kCANID.idMotorFL, MotorType.kBrushless);
        motorFR = new CANSparkFlex(kCANID.idMotorFR, MotorType.kBrushless);
        motorBL = new CANSparkFlex(kCANID.idMotorBL, MotorType.kBrushless);
        motorBR = new CANSparkFlex(kCANID.idMotorBR, MotorType.kBrushless);

        configMotor(motorFL, false);
        configMotor(motorFR, true);
        configMotor(motorBL, false);
        configMotor(motorBR, true);

        motorBL.follow(motorFL);
        motorBR.follow(motorFR);

        // Encoders
        encoderFL = motorFL.getEncoder();
        encoderFR = motorFR.getEncoder();
        encoderBL = motorBL.getEncoder();
        encoderBR = motorBR.getEncoder();

        configEncoder(encoderFL);
        configEncoder(encoderFR);
        configEncoder(encoderBL);
        configEncoder(encoderBR);
        
        // Other stuff
        pigeon = new Pigeon2(kCANID.idPigeon2);
        ddrive = new DifferentialDrive(motorFL, motorFR);

        // SysID
        sysIdAppliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
        sysIdDistance = MutableMeasure.mutable(Units.Meters.of(0));
        sysIdVelocity = MutableMeasure.mutable(Units.MetersPerSecond.of(0));

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {
                    setMeasureVoltage(motorFL, volts);
                    setMeasureVoltage(motorFR, volts);
                    setMeasureVoltage(motorBL, volts);
                    setMeasureVoltage(motorBR, volts);
                },
                log -> {
                    log.motor("FL")
                        .voltage(sysIdAppliedVoltage.mut_replace(motorFL.getBusVoltage(), Units.Volts))
                        .linearPosition(sysIdDistance.mut_replace(encoderFL.getPosition(), Units.Meters))
                        .linearVelocity(sysIdVelocity.mut_replace(encoderFL.getVelocity(), Units.MetersPerSecond));
                    log.motor("FR")
                        .voltage(sysIdAppliedVoltage.mut_replace(motorFL.getBusVoltage(), Units.Volts))
                        .linearPosition(sysIdDistance.mut_replace(encoderFL.getPosition(), Units.Meters))
                        .linearVelocity(sysIdVelocity.mut_replace(encoderFL.getVelocity(), Units.MetersPerSecond));
                    log.motor("BL")
                        .voltage(sysIdAppliedVoltage.mut_replace(motorFL.getBusVoltage(), Units.Volts))
                        .linearPosition(sysIdDistance.mut_replace(encoderFL.getPosition(), Units.Meters))
                        .linearVelocity(sysIdVelocity.mut_replace(encoderFL.getVelocity(), Units.MetersPerSecond));
                    log.motor("BR")
                        .voltage(sysIdAppliedVoltage.mut_replace(motorFL.getBusVoltage(), Units.Volts))
                        .linearPosition(sysIdDistance.mut_replace(encoderFL.getPosition(), Units.Meters))
                        .linearVelocity(sysIdVelocity.mut_replace(encoderFL.getVelocity(), Units.MetersPerSecond));
                },
                this
            )
        );
    }

    // Get subsystem
    public static Drivetrain getInstance() {
        if (instance == null) instance = new Drivetrain();

        return instance;
    }

    private void configMotor(CANSparkFlex motor, boolean isInverted) {
        motor.setInverted(isInverted);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setClosedLoopRampRate(kDrivetrain.kCLRampRate);
        motor.setSmartCurrentLimit(kDrivetrain.kCurrentLimit);
        motor.burnFlash();
    }

    private void configEncoder(RelativeEncoder encoder) {
        encoderFL.setPositionConversionFactor(kDrivetrain.kWheelCircumference);
        encoderFL.setVelocityConversionFactor(kDrivetrain.kWheelCircumference / 60);
        encoderFL.setPosition(0);
    }

    public void arcadeDrive(double accel, double rot) {
        ddrive.arcadeDrive(accel, rot);
    }

    public void setMeasureVoltage(CANSparkFlex motor, Measure<Voltage> volts) {
        motor.setVoltage(volts.in(Units.Volts));
    }

    public void stopAllMotors() {
        motorFL.stopMotor();
        motorFR.stopMotor();
        motorBL.stopMotor();
        motorBR.stopMotor();
    }

    public void zeroHeading() {
        pigeon.reset();
    }

    public double getHeading() {
        double heading = pigeon.getRotation2d().getDegrees() % 360;
        if (heading < 0) heading += 360;
        return heading;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        
    }

}