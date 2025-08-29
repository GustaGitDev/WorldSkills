package frc.robot.subsystems;

import com.orbbec.obsensor.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.cscore.CvSource;
import org.opencv.core.Mat;
import org.opencv.core.CvType;
import org.opencv.imgcodecs.Imgcodecs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DepthCamera extends SubsystemBase {

    private CvSource outputStream;
    private Mat frameMat;

    private OBContext obContext;
    private final Object pipeLock = new Object();
    private Pipeline pipeline;

    private volatile double distanceToWall = -1.0; // Z direto
    private volatile boolean initialized = false;

    // X e Y em standby
    private volatile double pX = 0;
    private volatile double pY = 0;

    public DepthCamera() {
        // Inicializa CameraServer
        outputStream = CameraServer.getInstance().putVideo("Orbbec Color", 640, 480);
        frameMat = new Mat(480, 640, CvType.CV_8UC3);

        initCamera();
    }

    private void initCamera() {
        obContext = new OBContext(new DeviceChangedCallback() {
            @Override
            public void onDeviceAttach(DeviceList deviceList) {
                initPipeline(deviceList);
                startStreams();
                deviceList.close();
            }

            @Override
            public void onDeviceDetach(DeviceList deviceList) {
                stopStreams();
                deInitPipeline();
                deviceList.close();
            }
        });

        DeviceList deviceList = obContext.queryDevices();
        if (deviceList != null && deviceList.getDeviceCount() > 0) {
            initPipeline(deviceList);
            startStreams();
        }
        if (deviceList != null) deviceList.close();
    }

    private void initPipeline(DeviceList deviceList) {
        synchronized (pipeLock) {
            if (pipeline != null) pipeline.close();
            try (Device device = deviceList.getDevice(0)) {
                pipeline = new Pipeline(device);
            } catch (Exception e) { e.printStackTrace(); }
        }
    }

    private void deInitPipeline() {
        synchronized (pipeLock) {
            try {
                if (pipeline != null) { pipeline.close(); pipeline = null; }
            } catch (Exception e) { e.printStackTrace(); }
        }
    }

    public void startStreams() {
        synchronized (pipeLock) {
            if (pipeline == null) return;

            Config config = new Config();
            try (StreamProfileList colorProfileList = pipeline.getStreamProfileList(SensorType.COLOR)) {
                VideoStreamProfile colorProfile = colorProfileList.getStreamProfile(0).as(StreamType.VIDEO);
                if (colorProfile != null) config.enableStream(colorProfile);
                colorProfile.close();
            } catch (Exception e) { e.printStackTrace(); }

            try (StreamProfileList depthProfileList = pipeline.getStreamProfileList(SensorType.DEPTH)) {
                VideoStreamProfile depthProfile = depthProfileList.getStreamProfile(0).as(StreamType.VIDEO);
                if (depthProfile != null) config.enableStream(depthProfile);
                depthProfile.close();
            } catch (Exception e) { e.printStackTrace(); }

            try {
                pipeline.start(config, new FrameSetCallback() {
                    @Override
                    public void onFrameSet(FrameSet frameSet) {
                        processFrameSet(frameSet);
                        frameSet.close();
                    }
                });
                config.close();
            } catch (Exception e) { e.printStackTrace(); }
        }
    }

    public void stopStreams() {
        synchronized (pipeLock) {
            try { if (pipeline != null) pipeline.stop(); } catch (Exception e) { e.printStackTrace(); }
        }
    }

    private void processFrameSet(FrameSet frameSet) {
        // Vídeo
        try (ColorFrame colorFrame = frameSet.getFrame(FrameType.COLOR)) {
            if (colorFrame != null) sendFrameToCameraServer(colorFrame);
        } catch (Exception e) { e.printStackTrace(); }

        // DepthFrame → só Z
        try (DepthFrame depthFrame = frameSet.getFrame(FrameType.DEPTH)) {
            if (depthFrame != null) processDepthFrame(depthFrame);
        } catch (Exception e) { e.printStackTrace(); }

        // Atualiza SmartDashboard
        updateSmartDashboard();
    }

    private void sendFrameToCameraServer(ColorFrame colorFrame) {
        try {
            byte[] data = new byte[colorFrame.getDataSize()];
            colorFrame.getData(data);

            if (data.length == frameMat.total() * frameMat.channels()) {
                frameMat.put(0, 0, data);
            } else {
                Mat encoded = new Mat(1, data.length, CvType.CV_8UC1);
                encoded.put(0, 0, data);
                frameMat = Imgcodecs.imdecode(encoded, Imgcodecs.IMREAD_COLOR);
            }

            outputStream.putFrame(frameMat);
        } catch (Exception e) { e.printStackTrace(); }
    }

    private void processDepthFrame(DepthFrame frame) {
        int height = frame.getWidth();
        int width = frame.getHeight();
        int x = width / 2;
        int y = height / 2;

        byte[] frameData = new byte[frame.getDataSize()];
        frame.getData(frameData);

        int depthD1 = frameData[y * width + x];
        int depthD2 = frameData[y * width + x + 1];
        double z = ((depthD2 << 8) & 0xFF00) + (depthD1 & 0xFF);

        // Só mantém Z
        this.distanceToWall = z / 1000.0; // metros

        if (!initialized) initialized = true;
    }

    private void updateSmartDashboard() {
        SmartDashboard.putBoolean("Camera Initialized", initialized);
        
        // Exibe a distância em metros
        SmartDashboard.putNumber("Distance to Wall (m)", distanceToWall);
        
        // Exibe a distância em centímetros
        SmartDashboard.putNumber("Distance to Wall (cm)", distanceToWall * 100.0);
    }

    // --- Getters ---
    public boolean isInitialized() { return initialized; }
    public double getDistanceToWall() { return distanceToWall; }

    public void destroyCamera() {
        synchronized(pipeLock) {
            try {
                if (pipeline != null) { pipeline.stop(); pipeline.close(); pipeline = null; }
                if (obContext != null) { obContext.close(); obContext = null; }
                initialized = false;
            } catch (Exception e) { e.printStackTrace(); }
        }
    }
}
