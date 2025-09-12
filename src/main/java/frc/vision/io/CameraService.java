package frc.vision.io;


import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import frc.vision.config.VisionConfig;


public class CameraService implements AutoCloseable {
private final VideoCapture cap;


public CameraService() {
cap = new VideoCapture(VisionConfig.CAMERA_INDEX);
if (!cap.isOpened()) throw new RuntimeException("Camera não abriu");
cap.set(3, VisionConfig.FRAME_WIDTH); // CAP_PROP_FRAME_WIDTH
cap.set(4, VisionConfig.FRAME_HEIGHT); // CAP_PROP_FRAME_HEIGHT
cap.set(5, VisionConfig.FRAME_FPS); // CAP_PROP_FPS
}


public boolean read(Mat out) { return cap.read(out); }


@Override public void close() { if (cap != null) cap.release(); }
}