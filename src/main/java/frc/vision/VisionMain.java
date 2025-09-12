package frc.vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import frc.vision.config.VisionConfig;
import frc.vision.io.CameraService;
import frc.vision.nt.VisionPublisher;
import frc.vision.yolo.*;

import java.nio.file.*;
import java.nio.charset.StandardCharsets;
import java.io.IOException;
import java.util.*;
import java.util.stream.Collectors;

// >>> CameraServer (Opção B)
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSource;

public class VisionMain {

  // (opcional) Filtrar por uma classe específica do COCO. Ex.: "person", "car", "bottle".
  // Se não quiser filtrar, deixe como null.
  private static final String TARGET_CLASS = null; // "person";

  // Carrega coco.names uma vez
  private static final List<String> LABELS = loadLabels("coco.names");

  private static List<String> loadLabels(String path) {
    try {
      return Files.readAllLines(Paths.get(path), StandardCharsets.UTF_8);
    } catch (IOException e) {
      return Collections.emptyList();
    }
  }

  public static void main(String[] args) {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

    YoloDetector detector = new YoloDetector(
        VisionConfig.YOLO_CFG, VisionConfig.YOLO_WEIGHTS);

    try (CameraService cam = new CameraService();
         VisionPublisher pub = new VisionPublisher()) {

      // === CameraServer: cria um stream de saída para o Dashboard ===
      final int w = VisionConfig.FRAME_WIDTH;
      final int h = VisionConfig.FRAME_HEIGHT;
      final CvSource out = CameraServer.getInstance().putVideo("Vision-Processed", w, h);

      Mat frame = new Mat();

      // parâmetros de texto (evita depender de constantes da API)
      final int fontFace = 0;       // 0 == FONT_HERSHEY_SIMPLEX
      final double fontScale = 0.5;
      final int thickness = 1;

      while (true) {
        if (!cam.read(frame)) continue;

        List<Detection> detections = detector.detect(frame);
        if (TARGET_CLASS != null && !LABELS.isEmpty()) {
          detections = detections.stream()
              .filter(d -> {
                int id = d.classId;
                String name = (id >= 0 && id < LABELS.size())
                    ? LABELS.get(id) : ("class_" + id);
                return TARGET_CLASS.equals(name);
              })
              .collect(Collectors.toList());
        }

        // Prepara um frame para desenhar (sempre publica algo no stream)
        Mat display;
        if (frame.channels() == 1) {
          display = new Mat();
          Imgproc.cvtColor(frame, display, Imgproc.COLOR_GRAY2BGR);
        } else {
          display = frame.clone();
        }

        if (detections.isEmpty()) {
          pub.publishNoTarget();
          out.putFrame(display);
          continue;
        }

        // Melhor detecção (maior confiança)
        Detection best = Collections.max(
            detections, Comparator.comparingDouble(d -> d.confidence));

        String label = (best.classId >= 0 && best.classId < LABELS.size())
            ? LABELS.get(best.classId)
            : ("class_" + best.classId);

        // Desenha overlay (retângulo + centro + rótulo)
        Point tl = new Point(best.box.x, best.box.y);
        Point br = new Point(best.box.x + best.box.width, best.box.y + best.box.height);

        // Usa assinatura completa (compatível com todas as 4.x):
        Imgproc.rectangle(display, tl, br, new Scalar(0, 255, 0), 2, Imgproc.LINE_8, 0);

        Point c = new Point(best.box.x + best.box.width / 2.0,
                            best.box.y + best.box.height / 2.0);
        Imgproc.circle(display, c, 6, new Scalar(0, 255, 0), 2);

        Imgproc.putText(
            display,
            String.format("%s (%.2f)", label, best.confidence),
            new Point(best.box.x, Math.max(0, best.box.y - 5)),
            fontFace, fontScale, new Scalar(0, 255, 0), thickness
        );

        // Publica posição normalizada + confiança no NT
        double nx = c.x / frame.cols();
        double ny = c.y / frame.rows();
        pub.publish(nx, ny, best.confidence);

        // Envia o frame anotado para o CameraServer
        out.putFrame(display);
      }
    }
  }
}
