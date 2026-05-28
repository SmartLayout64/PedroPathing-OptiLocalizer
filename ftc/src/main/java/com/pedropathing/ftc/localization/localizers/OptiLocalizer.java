package com.pedropathing.ftc.localization.localizers;

import android.content.Context;
import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.HashMap;

/**
 * This is the OptiLocalizer class. This class extends the Localizer superclass and is a
 * localizer that uses a wired USB mouse with IMU set up.
 * @author Ashrit Mandava - 25611 Robotechs
 * @version 1.0, 5/9/2026
 */
public class OptiLocalizer implements Localizer {

    // USB HID Constants
    private static final int HID_SUBCLASS_BOOT = 0x01;
    private static final int HID_PROTOCOL_MOUSE = 0x02;
    private static final int TRANSFER_TIMEOUT_MS = 50;

    // Movement State
    private Pose currentPose = new Pose();
    private Pose velocity = new Pose();
    private double totalHeading = 0;
    private double lastHeading = 0;
    private final ElapsedTime timer = new ElapsedTime();

    // Accumulated Movement Storage
    private int accumulatedDeltaX = 0;
    private int accumulatedDeltaY = 0;

    // Configuration
    private double sensorDPI;
    private double forwardMultiplier = 1.0;
    private double lateralMultiplier = 1.0;

    // USB Management
    private final UsbManager usbManager;
    private UsbDeviceConnection connection;
    private UsbEndpoint inEndpoint;
    private UsbInterface usbInterface;

    private Thread pollThread;
    private volatile boolean running = false;
    // IMU
    private final IMU imu;

    public OptiLocalizer(HardwareMap hardwareMap, double sensorDPI) {
        this.sensorDPI = sensorDPI;
        this.usbManager = (UsbManager) hardwareMap.appContext.getSystemService(Context.USB_SERVICE);

        if (initUsb()) {
            startPolling();
        }
        timer.reset();

        this.imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
    }

    private boolean initUsb() {
        HashMap<String, UsbDevice> deviceList = this.usbManager.getDeviceList();

        for (UsbDevice device : deviceList.values()) {
            for (int i = 0; i < device.getInterfaceCount(); i++) {
                UsbInterface iface = device.getInterface(i);

                if (iface.getInterfaceClass() == UsbConstants.USB_CLASS_HID &&
                        iface.getInterfaceSubclass() == HID_SUBCLASS_BOOT &&
                        iface.getInterfaceProtocol() == HID_PROTOCOL_MOUSE) {

                    if (!this.usbManager.hasPermission(device)) return false;

                    this.connection = this.usbManager.openDevice(device);
                    if (this.connection == null) return false;

                    if (this.connection.claimInterface(iface, true)) {
                        usbInterface = iface;
                        for (int j = 0; j < iface.getEndpointCount(); j++) {
                            UsbEndpoint ep = iface.getEndpoint(j);
                            if (ep.getType() == UsbConstants.USB_ENDPOINT_XFER_INT &&
                                    ep.getDirection() == UsbConstants.USB_DIR_IN) {
                                this.inEndpoint = ep;
                                return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    }

    private void startPolling() {
        this.running = true;

        this.pollThread = new Thread(() -> {
            byte[] buf = new byte[8];

            while (this.running && !Thread.interrupted()) {
                int len = connection.bulkTransfer(this.inEndpoint, buf, buf.length, TRANSFER_TIMEOUT_MS);

                if (len >= 3) {
                    synchronized (this) {
                        this.accumulatedDeltaX += buf[1];
                        this.accumulatedDeltaY += buf[2];
                    }
                }
            }
        });

        this.pollThread.setDaemon(true);
        this.pollThread.start();
    }

    /**
     * This returns the current pose estimate from the Localizer.
     *
     * @return returns the pose as a Pose object.
     */
    @Override
    public Pose getPose() {
        return this.currentPose;
    }

    /**
     * This returns the current velocity estimate from the Localizer.
     *
     * @return returns the velocity as a Pose object.
     */
    @Override
    public Pose getVelocity() {
        return this.velocity;
    }

    /**
     * This returns the current velocity estimate from the Localizer as a Vector.
     *
     * @return returns the velocity as a Vector.
     */
    @Override
    public Vector getVelocityVector() {
        return this.velocity.getAsVector();
    }

    /**
     * This sets the start pose of the Localizer. Changing the start pose should move the robot as if
     * all its previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        this.currentPose = setStart;
        this.lastHeading = setStart.getHeading();
    }

    /**
     * This sets the current pose estimate of the Localizer. Changing this should just change the
     * robot's current pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        this.currentPose = setPose;
    }

    /**
     * This calls an update to the Localizer, updating the current pose estimate and current velocity
     * estimate.
     */
    @Override
    public void update() {
        double deltaTime = timer.seconds();
        this.timer.reset();

        int localX, localY;

        synchronized (this) {
            localX = this.accumulatedDeltaX;
            localY = this.accumulatedDeltaY;
            this.accumulatedDeltaX = 0;
            this.accumulatedDeltaY = 0;
        }

        double dX = (localX / sensorDPI) * forwardMultiplier;
        double dY = (localY / sensorDPI) * lateralMultiplier;

        double heading = currentPose.getHeading();
        double deltaHeading = heading - lastHeading;
        this.lastHeading = heading;
        this.totalHeading += deltaHeading;

        double globalDeltaX = dX * Math.cos(heading) - dY * Math.sin(heading);
        double globalDeltaY = dX * Math.sin(heading) + dY * Math.cos(heading);

        currentPose = new Pose(
                this.currentPose.getX() + globalDeltaX,
                this.currentPose.getY() + globalDeltaY,
                heading
        );

        if (deltaTime > 0) {
            this.velocity = new Pose(globalDeltaX / deltaTime, globalDeltaY / deltaTime, deltaHeading / deltaTime);
        }
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    @Override
    public double getTotalHeading() {
        return this.totalHeading;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    @Override
    public double getForwardMultiplier() {
        return this.forwardMultiplier;
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    @Override
    public double getLateralMultiplier() {
        return this.lateralMultiplier;
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    @Override
    public double getTurningMultiplier() {
        return 1;
    }

    /**
     * This resets the IMU of the localizer, if applicable.
     */
    @Override
    public void resetIMU() {
        this.imu.resetYaw();
    }

    /**
     * This is overridden to return the IMU's heading estimate, if there is one.
     *
     * @return returns the IMU's heading estimate if it exists
     */
    @Override
    public double getIMUHeading() {
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orientation.firstAngle;
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns if any component of the robot's position is NaN
     */
    @Override
    public boolean isNAN() {
        return Double.isNaN(currentPose.getX()) || Double.isNaN(currentPose.getY()) || Double.isNaN(currentPose.getHeading());
    }

    public void stop() {
        running = false;
        if (connection != null) {
            connection.releaseInterface(usbInterface);
            connection.close();
        }
    }
}
