import cv2
from ultralytics import YOLO

model = YOLO('best2.engine', task='detect')

# 利用 ultralytics 的流式预测，开启 show=True（它会自动使用 GStreamer 低延迟管道）
results = model.predict(
    source=0,
    stream=True,   # 流模式
    show=True,     # 自动显示
    imgsz=640,
    conf=0.8,
    verbose=False
)

# 遍历生成器，维持显示循环
for r in results:
    # 内部已经调用 cv2.imshow 和 cv2.waitKey(1)
    # 按 'q' 键退出（这是 ultralytics 内置的行为）
    pass
