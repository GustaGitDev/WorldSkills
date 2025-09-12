package frc.vision.yolo;

import java.util.*;
import org.opencv.core.*;
import org.opencv.dnn.*;
import frc.vision.config.VisionConfig;

// +++ IMPORTS NECESSÁRIOS PARA NMSBoxes EM 4.8.x +++
import org.opencv.core.Rect2d;
import org.opencv.core.MatOfRect2d;

public class YoloDetector {
  private final Net net;
  private final Size inputSize;

  public YoloDetector(String cfgPath, String weightsPath) {
    net = Dnn.readNetFromDarknet(cfgPath, weightsPath);
    net.setPreferableBackend(Dnn.DNN_BACKEND_OPENCV);
    net.setPreferableTarget(Dnn.DNN_TARGET_CPU);
    inputSize = new Size(VisionConfig.INPUT_SIZE, VisionConfig.INPUT_SIZE);
  }

  public List<Detection> detect(Mat frame) {
    // Pré-processamento
    Mat blob = Dnn.blobFromImage(frame, 1/255.0, inputSize, new Scalar(0,0,0), true, false);
    net.setInput(blob);

    // Forward nas camadas de saída
    List<String> outNames = net.getUnconnectedOutLayersNames();
    List<Mat> outs = new ArrayList<>();
    net.forward(outs, outNames);

    // >>> Troca para Rect2d <<<
    List<Rect2d> boxes = new ArrayList<>();
    List<Float> confs = new ArrayList<>();
    List<Integer> classIds = new ArrayList<>();

    int width = frame.cols();
    int height = frame.rows();

    for (Mat out : outs) {
      for (int i = 0; i < out.rows(); i++) {
        // YOLO row: [cx, cy, w, h, obj, class1, class2, ...]
        double[] data = out.get(i, 0);
        if (data == null) continue;

        float cx = (float) data[0] * width;
        float cy = (float) data[1] * height;
        float w  = (float) data[2] * width;
        float h  = (float) data[3] * height;
        float obj = (float) data[4];

        // varre classes para maior score
        int bestClass = -1; float bestScore = 0f;
        for (int c = 5; c < out.cols(); c++) {
          float score = (float) data[c];
          if (score > bestScore) { bestScore = score; bestClass = c - 5; }
        }
        float conf = obj * bestScore;
        if (conf < (float) VisionConfig.CONF_THRESHOLD) continue;

        int x  = Math.max(0, (int) (cx - w/2));
        int y  = Math.max(0, (int) (cy - h/2));
        int bw = Math.min((int) w, width  - x);
        int bh = Math.min((int) h, height - y);

        // >>> Rect2d aqui <<<
        boxes.add(new Rect2d(x, y, bw, bh));
        confs.add(conf);
        classIds.add(bestClass);
      }
    }

    // NMS (Non-Maximum Suppression) com MatOfRect2d
    MatOfRect2d matBoxes = new MatOfRect2d();
    matBoxes.fromList(boxes);
    MatOfFloat matScores = new MatOfFloat();
    matScores.fromList(confs);
    MatOfInt indices = new MatOfInt();

    Dnn.NMSBoxes(
        matBoxes,
        matScores,
        (float) VisionConfig.CONF_THRESHOLD,
        (float) VisionConfig.NMS_THRESHOLD,
        indices
    );

    // Converte de Rect2d -> Rect se sua classe Detection usa Rect (int)
    List<Detection> results = new ArrayList<>();
    for (int idx : indices.toArray()) {
      Rect2d r2 = boxes.get(idx);
      Rect r = new Rect((int) r2.x, (int) r2.y, (int) r2.width, (int) r2.height);
      results.add(new Detection(classIds.get(idx), confs.get(idx), r));
    }
    return results;
  }
}
