import cv2
import json
import os
import numpy as np
from deepface import DeepFace
import time
from numpy import dot
from numpy.linalg import norm

# =========================
# 配置
# =========================
address = "udp://localhost:5000"
db_json_path = "./photo_database.json"
MODEL_NAME = "VGG-Face"
SIM_THRESHOLD = 0   # 相似度阈值，可自己调

# =========================
# 工具函数
# =========================
def cosine_similarity(a, b):
    return np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))


# =========================
# Step1: 读取 JSON，构建数据库 embedding
# =========================
print("Loading face database from JSON...")

with open(db_json_path, "r", encoding="utf-8") as f:
    db = json.load(f)

# 结构：
# {
#   "科比·布莱恩特": [emb1, emb2, ...],
#   "某某": [...]
# }
print(f'Database:\n {db}')

db_embeddings = {}

for person_name, img_paths in db.items():
    db_embeddings[person_name] = []
    for img_path in img_paths:
        try:
            emb = DeepFace.represent(
                img_path=img_path,
                model_name=MODEL_NAME,
                enforce_detection=False
            )[0]["embedding"]
            db_embeddings[person_name].append(emb)
        except Exception as e:
            print(f"[Skip] {img_path}: {e}")

print(f"Database loaded: {len(db_embeddings)} persons")

# =========================
# Step2: 命令循环（C / Q）
# =========================
print("\nCommand:")
print("  C - Capture & recognize")
print("  Q - Quit")

while True:
    cmd = input("\nEnter command (C/Q): ").strip().upper()

    if cmd == "Q":
        print("Bye 👋")
        break

    if cmd != "C":
        print("Invalid command.")
        continue

    # =========================
    # Step3: 打开摄像头，采集一帧
    # =========================
    # cap = cv2.VideoCapture(address)
    # if not cap.isOpened():
    #     print("Error: Cannot open camera")
    #     continue

    # frame = None
    # ret = False

    # # 给网络流一点缓冲时间
    # time.sleep(0.5)

    # for _ in range(20):  # 最多尝试 20 次
    #     ret, frame = cap.read()
    #     if ret and frame is not None:
    #         break
    #     time.sleep(0.05)

    # cap.release()

    # if not ret or frame is None:
    #     print("Failed to capture frame (no data from stream)")
    #     continue


    temp_img_path = "temp_frame.jpg"
    # cv2.imwrite(temp_img_path, frame)

    # =========================
    # Step4: 提取当前人脸 embedding
    # =========================
    try:
        face_emb = DeepFace.represent(
            img_path=temp_img_path,
            model_name=MODEL_NAME,
            enforce_detection=False
        )[0]["embedding"]
    except Exception:
        print("No face detected")
        continue

    # =========================
    # Step5: 与数据库比对
    # =========================
    best_person = None
    best_score = -1
    
    for person_name, emb_list in db_embeddings.items():
        for emb in emb_list:
            sim = dot(face_emb, emb) / (norm(face_emb) * norm(emb))
            if sim > best_score:
                best_score = sim
                best_person = person_name

    # =========================
    # Step6: 输出结果
    # =========================
    if best_person is not None and best_score > SIM_THRESHOLD:
        print(f"你好！{best_person}")
        print(f"confidence: {best_score:.4f}")
    else:
        print("未识别到已知身份")

    # 可选：显示采集画面
    # cv2.imshow("Captured Frame", frame)
    # cv2.waitKey(1000)
    # cv2.destroyAllWindows()
