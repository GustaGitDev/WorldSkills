package frc.vision;

import org.opencv.core.*;
import frc.vision.config.VisionConfig;
import frc.vision.io.CameraService;
import frc.vision.nt.VisionPublisher;
import frc.vision.yolo.*;
import java.nio.file.*;
import java.nio.charset.StandardCharsets;
import java.io.IOException;
import java.util.*;
import java.util.stream.Collectors;

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
      // Se não achar o arquivo, segue com lista vazia (evita travar a visão)
      return Collections.emptyList();
    }
  }

  public static void main(String[] args) {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

    YoloDetector detector = new YoloDetector(
        VisionConfig.YOLO_CFG, VisionConfig.YOLO_WEIGHTS);

    try (CameraService cam = new CameraService();
         VisionPublisher pub = new VisionPublisher()) {

      Mat frame = new Mat();

      while (true) {
        if (!cam.read(frame)) continue;

        List<Detection> detections = detector.detect(frame);
        if (detections.isEmpty()) {
          pub.publishNoTarget();
          continue;
        }

        // (opcional) Filtrar pela classe desejada ANTES de escolher a melhor
        if (TARGET_CLASS != null && !LABELS.isEmpty()) {
          detections = detections.stream()
              .filter(d -> {
                int id = d.classId;
                String name = (id >= 0 && id < LABELS.size())
                    ? LABELS.get(id) : ("class_" + id);
                return TARGET_CLASS.equals(name);
              })
              .collect(Collectors.toList());

          if (detections.isEmpty()) {
            pub.publishNoTarget();
            continue;
          }
        }

        // Melhor detecção (maior confiança)
        Detection best = Collections.max(
            detections, Comparator.comparingDouble(d -> d.confidence));

        // Nome da classe (para logs/filtros)
        String label = (best.classId >= 0 && best.classId < LABELS.size())
            ? LABELS.get(best.classId)
            : ("class_" + best.classId);

        // Debug opcional
        // System.out.printf("Detect: %s conf=%.2f box=%s%n", label, best.confidence, best.box.toString());

        double centerX = best.box.x + best.box.width / 2.0;
        double centerY = best.box.y + best.box.height / 2.0;
        double nx = centerX / frame.cols();
        double ny = centerY / frame.rows();

        pub.publish(nx, ny, best.confidence);

        // (Opcional) desenhar/salvar:
        // Imgproc.rectangle(frame, best.box, new Scalar(0,255,0), 2);
        // Imgcodecs.imwrite("debug.jpg", frame);
      }
    }
  }
}
