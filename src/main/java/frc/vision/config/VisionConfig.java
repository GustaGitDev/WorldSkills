package frc.vision.config;


public final class VisionConfig {
// Camera
public static final int CAMERA_INDEX = 0; // USB camera na Pi
public static final int FRAME_WIDTH = 640;
public static final int FRAME_HEIGHT = 480;
public static final int FRAME_FPS = 20; // ajuste conforme o desempenho


// YOLO
public static final String YOLO_CFG = "yolov3-tiny.cfg"; // arquivo no diretório do jar
public static final String YOLO_WEIGHTS = "yolov3-tiny.weights";
public static final double CONF_THRESHOLD = 0.40; // confiança mínima
public static final double NMS_THRESHOLD = 0.45; // Non-Max Suppression
public static final int INPUT_SIZE = 416; // 320 ou 416


// NetworkTables (NT3 em 2020)
public static final String NT_IDENTITY = "vision-rpi";
public static final String NT_TABLE = "vision";
public static final String NT_SERVER = "10.12.34.2"; // IP do roboRIO (ajuste)
public static final int DEBUG_PORT = 5005; // JDWP
}
