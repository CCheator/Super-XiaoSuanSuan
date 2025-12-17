import cv2
import json
import os
import numpy as np
from deepface import DeepFace
import time
from numpy import dot
from numpy.linalg import norm
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# =========================
# 配置
# =========================
address = "udp://localhost:5000"
db_json_path = "./photo_database.json"
MODEL_NAME = "VGG-Face"
SIM_THRESHOLD = 0.5   # 提高阈值，避免误唤醒
DETECTION_INTERVAL = 1.0  # 检测间隔（秒）

# =========================
# 工具函数
# =========================
def cosine_similarity(a, b):
    return np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))


class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')
        self.face_pub = self.create_publisher(String, '/face_recognition_result', 10)
        self.timer = self.create_timer(DETECTION_INTERVAL, self.detect_face)
        
        # 加载人脸数据库
        self.db_embeddings = self.load_database()
        self.get_logger().info(f"Face database loaded: {len(self.db_embeddings)} persons")

    def load_database(self):
        """加载人脸数据库"""
        self.get_logger().info("Loading face database from JSON...")
        try:
            with open(db_json_path, "r", encoding="utf-8") as f:
                db = json.load(f)
            
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
                        self.get_logger().warn(f"[Skip] {img_path}: {e}")
            return db_embeddings
        except Exception as e:
            self.get_logger().error(f"Failed to load database: {e}")
            return {}

    def detect_face(self):
        """检测人脸并发布结果"""
        # 打开摄像头采集帧
        cap = cv2.VideoCapture(address)
        if not cap.isOpened():
            self.get_logger().error("Error: Cannot open camera")
            return

        frame = None
        ret = False

        # 给网络流一点缓冲时间
        time.sleep(0.5)

        # 尝试获取有效帧
        for _ in range(20):
            ret, frame = cap.read()
            if ret and frame is not None:
                break
            time.sleep(0.05)

        cap.release()

        if not ret or frame is None:
            self.get_logger().warn("Failed to capture frame")
            return

        # 保存临时帧用于识别
        temp_img_path = "temp_frame.jpg"
        cv2.imwrite(temp_img_path, frame)

        # 提取当前人脸特征
        try:
            face_emb = DeepFace.represent(
                img_path=temp_img_path,
                model_name=MODEL_NAME,
                enforce_detection=False
            )[0]["embedding"]
        except Exception:
            self.get_logger().info("No face detected")
            return

        # 与数据库比对
        best_person = None
        best_score = -1
        
        for person_name, emb_list in self.db_embeddings.items():
            for emb in emb_list:
                sim = dot(face_emb, emb) / (norm(face_emb) * norm(emb))
                if sim > best_score:
                    best_score = sim
                    best_person = person_name

        # 发布识别结果（仅当匹配度超过阈值）
        if best_person is not None and best_score > SIM_THRESHOLD:
            self.get_logger().info(f"Recognized: {best_person} (confidence: {best_score:.4f})")
            msg = String()
            msg.data = best_person  # 发布识别到的人名
            self.face_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down face recognition node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()