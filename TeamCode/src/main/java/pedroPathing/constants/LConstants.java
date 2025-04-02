package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.0019715;
        ThreeWheelConstants.strafeTicksToInches = 0.00211;
        ThreeWheelConstants.turnTicksToInches = 0.001932;
        ThreeWheelConstants.leftY = 10.7/2.54;
        ThreeWheelConstants.rightY = -10.7/2.54;
        ThreeWheelConstants.strafeX = 8.5/2.54;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFront";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightBack";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}