## **Optional: Training a Custom YOLO Model**

Follow these steps to train a new YOLOv8 detection model:

1. Run the go1_robot/go1_detection/training/generate_images.py script to create images and labels for the new dataset:
```bash
 python generate_images.py
``` 

2. Copy the generated data to the appropriate dataset folders:
   - Training dataset : Copy to go1_robot/go1_detection/training/Detection/dataset/train.
   - Validation dataset : Run generate_images.py again and copy a portion of the data to go1_robot/go1_detection/training/Detection/dataset/val.

3. For training:
   - Upload the go1_robot/go1_detection/training/Detection folder to Google Drive.
   - Open the folder, right-click on detection.ipynb, and select Open with Google Colab.
   - Change runtime type to T4 GPU and run the cells.
   - Test the model by supplying example images to /Detection/test_images. Prediction results are saved to /Detection/predictions. 

4. Once training is complete:
   - Download the best model which is runs/detect/train/weights/best.pt . Replace this file (best.pt) with the model in the go1_robot/go1_detection/models folder.
