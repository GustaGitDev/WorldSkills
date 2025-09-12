package frc.vision.yolo;


import org.opencv.core.Rect;


public class Detection {
public final int classId;
public final float confidence;
public final Rect box;


public Detection(int classId, float confidence, Rect box) {
this.classId = classId;
this.confidence = confidence;
this.box = box;
}
}