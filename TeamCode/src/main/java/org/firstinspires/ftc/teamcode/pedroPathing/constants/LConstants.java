package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.002;
        TwoWheelConstants.strafeTicksToInches = 0.002;
        TwoWheelConstants.forwardY = 1;
        TwoWheelConstants.strafeX = -1.5;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "rightRear";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "leftRear";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    }
}




