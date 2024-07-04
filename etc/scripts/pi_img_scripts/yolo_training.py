from ultralytics import YOLO

def main():
    model = YOLO('yolov8n.pt')
    print("model loaded, ultra")
    output = model.train(data='data.yaml', epochs=20, batch = 32)
    print("model trained, ultra")
    new_model = YOLO('best.pt',task='detect')
    new_model.export(format="ncnn")


if __name__ == "__main__":
    main()