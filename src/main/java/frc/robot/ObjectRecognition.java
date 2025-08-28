package frc.robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import org.opencv.objdetect.CascadeClassifier;
import org.opencv.imgproc.Imgproc;

public class ObjectRecognition {
    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Carregar o classificador Haar Cascade para rosto
        CascadeClassifier faceCascade = new CascadeClassifier("C:/opencv/sources/data/haarcascades/haarcascade_frontalface_default.xml");

        if (faceCascade.empty()) {
            System.out.println("Erro ao carregar o classificador Haar.");
            return;
        }

        VideoCapture camera = new VideoCapture(0);
        if (!camera.isOpened()) {
            System.out.println("Erro ao abrir a câmera.");
            return;
        }

        Mat frame = new Mat();
        while (true) {
            camera.read(frame);  // Captura um frame da câmera

            if (frame.empty()) {
                System.out.println("Erro ao capturar o frame");
                break;
            }

            // Convertendo para escala de cinza
            Mat grayFrame = new Mat();
            Imgproc.cvtColor(frame, grayFrame, Imgproc.COLOR_BGR2GRAY);

            // Detectar rostos
            MatOfRect faces = new MatOfRect();
            faceCascade.detectMultiScale(grayFrame, faces);

            // Desenhar retângulos ao redor dos rostos detectados
            for (Rect face : faces.toArray()) {
                Imgproc.rectangle(frame, face.tl(), face.br(), new Scalar(0, 255, 0), 2);
            }

            // Exibir a imagem com os rostos detectados
            HighGui.imshow("Rostos Detectados", frame);

            if (HighGui.waitKey(1) == 27) {
                break;  // Sai ao pressionar ESC
            }
        }

        camera.release();
        HighGui.destroyAllWindows();
    }
}
