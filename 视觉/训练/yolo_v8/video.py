from ultralytics import YOLO
import cv2

model = YOLO('best2.engine', task='detect')

results_gen = model.predict(
    source=0,
    stream=True,
    show=False,
    conf=0.8,
    imgsz=640,
    verbose=False
)

# 获取第一帧来确定实际尺寸
first_result = next(results_gen)
annotated = first_result.plot()
h, w = annotated.shape[:2]

# 使用 XVID 编码写入 AVI 文件（Jetson 原生支持）
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output_detected.avi', fourcc, 30.0, (w, h))
out.write(annotated)

print(f"录制开始，分辨率 {w}x{h}，按 Ctrl+C 停止...")
try:
    for r in results_gen:
        annotated = r.plot()
        out.write(annotated)
except KeyboardInterrupt:
    print("录制结束")

out.release()
cv2.destroyAllWindows()
