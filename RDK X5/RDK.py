import os
import sys
import cv2
import time
import serial
import numpy as np
import threading
import subprocess
import re
import socket
from collections import deque
from flask import Flask, Response
from flask_cors import CORS
from hobot_dnn import pyeasy_dnn as dnn

# 全局核心配置模块 (Global Configuration)
MODEL_PATH = "/root/qianrushi/detect.bin"  # 地平线 BPU 编译量化后的特征识别模型 (.bin)
CONF_THRESHOLD = 0.65                       # 目标置信度阈值 (边缘侧初筛门槛)
IOU_THRESHOLD = 0.50                        # 非极大值抑制 (NMS) 交并比重叠度阈值
CLASS_NAMES = ["person"]                    # 模型支持的检测目标类别列表

# 车载通信外设配置 (AGV Chassis Hardware Interconnect)
SERIAL_PORT = "/dev/ttyS1"                  # 控制底盘硬件的物理串口
SERIAL_BAUDRATE = 115200                    # 串口通信波特率
SERIAL_COOLDOWN = 1                         # 串口下发停止指令的最小冷却是时（秒），防止总线拥堵

# 异步播音子系统配置 (Audio Alert Subsystem)
VOICE_FILE_DIR = "/root/qianrushi/"          # 音频资源文件存放根目录
VOICE_FILES = {"person": "Detect.wav"}  # 触发警报时播放的指定 WAV 文件
VOICE_COOLDOWN = 500                        # 播音全局防抖冷却周期

# 边缘网关推流配置 (Web Video Streaming Service)
STREAM_PORT = 5000                          # Flask 实时局域网图传服务器监听端口


def sigmoid(x):
    """数值稳定的 Sigmoid 激活函数，用于解析检测头分类置信度"""
    return 1.0 / (1.0 + np.exp(-np.clip(x, -50, 50)))
# 自动外设检索与环境发现辅助函数 (Hardware Discovery Helpers)
def list_video_devices():
    """扫描 Linux V4L2 系统底层的全部视频设备节点"""
    devs = [os.path.join('/dev', dev) for dev in os.listdir('/dev') if dev.startswith('video')]
    return devs

def is_usb_camera(device):
    """通过尝试拉流，验证指定的 /dev/videoX 节点是否为可用且未被占用的 USB 摄像头"""
    try:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return False
        ret, _ = cap.read()
        cap.release()
        return ret
    except:
        return False

def find_first_usb_camera():
    """自动枚举并锁定系统首个合法的硬件 UVC/USB 摄像头通道"""
    devs = list_video_devices()
    if not devs:
        return None
    for dev in devs:
        if is_usb_camera(dev):
            return dev
    return None

def get_usb_audio_card_index():
    try:
        result = subprocess.getoutput("aplay -l")
        for line in result.split('\n'):
            if "USB Audio Device" in line or "USB Audio" in line:
                match = re.search(r"card (\d+):", line)
                if match:
                    return int(match.group(1))
    except:
        pass
    return 0  # 默认未检测到则返回系统默认声卡 0

def get_local_ip():
    """创建 UDP 伪连接以获取边缘网关在当前局域网内的真实活动 IP 地址，方便远程调试登录"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "127.0.0.1"

# 地平线 BPU 专用 YOLO 推理内核 (Horizon Inference Engine)
class YOLODetectorV2:
    """针对地平线 BPU (BPU Acceleration) 深度优化的实时 YOLO 目标检测器"""
    
    def __init__(self, model_path):
        # 载入地平线 BPU 后端量化模型
        models = dnn.load(model_path)
        self.model = models[0]
        self.reg_max = 16  # DFL (Distribution Focal Loss) 的回归最大通道范围
        
        # 动态解析模型的输入排列格式 (自动适配 NCHW / NHWC 布局)
        in_shape = list(self.model.inputs[0].properties.shape)
        if in_shape[1] == 3:
            self.input_h, self.input_w = int(in_shape[2]), int(in_shape[3])
        else:
            self.input_h, self.input_w = int(in_shape[1]), int(in_shape[2])
            
        self.heads = self._parse_heads()
        
        # 预计算特征图网格点映射 (Grid Anchors Pre-computation)，节约运行时 CPU 开销
        for h in self.heads:
            H, W = h["hw"]
            gy, gx = np.meshgrid(np.arange(H), np.arange(W), indexing="ij")
            h["grid"] = np.stack([gx, gy], axis=-1).reshape(-1, 2).astype(np.float32)

    def _parse_heads(self):
        """解析模型输出张量，自动完成 bbox(边界框) 与 cls(分类) 特征头的多尺度配对"""
        bbox_map, cls_map = {}, {}
        for idx, out in enumerate(self.model.outputs):
            shp = list(out.properties.shape)
            if len(shp) != 4:
                continue
            # 通道数等于 64 (16*4) 的判定为边界框回归分支
            if shp[1] == 64:
                bbox_map[(shp[2], shp[3])] = (idx, "NCHW")
            elif shp[-1] == 64:
                bbox_map[(shp[1], shp[2])] = (idx, "NHWC")
            # 通道数匹配类别数的判定为分类分支
            elif shp[1] in (1, len(CLASS_NAMES)) and shp[2] > 1:
                cls_map[(shp[2], shp[3])] = (idx, "NCHW")
            elif shp[-1] in (1, len(CLASS_NAMES)):
                cls_map[(shp[1], shp[2])] = (idx, "NHWC")
                
        heads = []
        for hw in sorted(bbox_map.keys(), key=lambda x: -x[0]):
            if hw not in cls_map:
                continue
            b_idx, b_fmt = bbox_map[hw]
            c_idx, c_fmt = cls_map[hw]
            stride = self.input_h // hw[0]  # 计算该特征图的下采样步长 (Stride)
            heads.append({
                "bbox_idx": b_idx, "cls_idx": c_idx, "hw": hw, "stride": stride,
                "bbox_fmt": b_fmt, "cls_fmt": c_fmt,
            })
        return heads

    def bgr_to_nv12(self, img):
        """将高分辨率 BGR 图像转换为 NV12 (YUV420SP) 格式，同时保持等比例缩放"""
        h, w = img.shape[:2]
        scale = min(self.input_h / h, self.input_w / w)
        new_h, new_w = int(h * scale), int(w * scale)
        resized = cv2.resize(img, (new_w, new_h))
        canvas = np.full((self.input_h, self.input_w, 3), 114, dtype=np.uint8)  # 使用 Gray 色度填充边界
        
        pad_top = (self.input_h - new_h) // 2
        pad_left = (self.input_w - new_w) // 2
        canvas[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = resized
        
        # 核心算法：快速完成 BGR 到 NV12 内存视窗的格式转换
        yuv = cv2.cvtColor(canvas, cv2.COLOR_BGR2YUV_I420)
        y = yuv[:self.input_h, :]
        u = yuv[self.input_h:self.input_h + self.input_h // 4, :].reshape(self.input_h // 2, self.input_w // 2)
        v = yuv[self.input_h + self.input_h // 4:, :].reshape(self.input_h // 2, self.input_w // 2)
        uv = np.stack([u, v], axis=-1).reshape(self.input_h // 2, self.input_w)
        nv12 = np.concatenate([y, uv], axis=0)
        return nv12, scale, pad_left, pad_top

    def dfl_decode(self, bbox_raw):
        """多尺度 DFL 积分解码：将 64 个回归通道还原为目标的 LTRB(左上右下) 边缘相对偏移量"""
        bbox = bbox_raw.reshape(-1, 4, self.reg_max)
        bbox_exp = np.exp(bbox - np.max(bbox, axis=-1, keepdims=True))
        bbox_sm = bbox_exp / np.sum(bbox_exp, axis=-1, keepdims=True)
        weights = np.arange(self.reg_max, dtype=np.float32).reshape(1, 1, -1)
        return np.sum(bbox_sm * weights, axis=-1)

    def _extract_feat(self, out, idx, fmt, H, W, C):
        """高效抽取 BPU 节点底层物理 Buffer 中的一维特征指针，并拉伸重塑为多维矩阵"""
        buf = np.array(out[idx].buffer, copy=False).astype(np.float32)
        return buf.reshape(1, C, H, W).transpose(0, 2, 3, 1).reshape(-1, C) if fmt == "NCHW" else buf.reshape(-1, C)

    def detect(self, frame):
        """输入原始 BGR 帧，执行全套 BPU 前向推理、多尺度解码及非极大值抑制 (NMS)，输出最终预测框"""
        try:
            orig_h, orig_w = frame.shape[:2]
            nv12, scale, pad_left, pad_top = self.bgr_to_nv12(frame)
            outs = self.model.forward(nv12)  # 送入地平线硬件加速核心进行推理
            all_boxes, all_scores = [], []
            
            for h in self.heads:
                H, W = h["hw"]
                stride = h["stride"]
                grid = h["grid"]
                bbox_feat = self._extract_feat(outs, h["bbox_idx"], h["bbox_fmt"], H, W, 64)
                cls_feat = self._extract_feat(outs, h["cls_idx"], h["cls_fmt"], H, W, len(CLASS_NAMES))
                
                scores = sigmoid(cls_feat)
                max_scores = np.max(scores, axis=1)
                keep = max_scores >= CONF_THRESHOLD
                if not np.any(keep):
                    continue
                    
                bbox_keep = bbox_feat[keep]
                score_keep = max_scores[keep]
                grid_keep = grid[keep]
                
                # 解码边界回归
                ltrb = self.dfl_decode(bbox_keep)
                xc = (grid_keep[:, 0] + 0.5) * stride
                yc = (grid_keep[:, 1] + 0.5) * stride
                
                # 将预测框映射回原图坐标系
                x1 = np.clip((xc - ltrb[:, 0] * stride - pad_left) / scale, 0, orig_w)
                y1 = np.clip((yc - ltrb[:, 1] * stride - pad_top) / scale, 0, orig_h)
                x2 = np.clip((xc + ltrb[:, 2] * stride - pad_left) / scale, 0, orig_w)
                y2 = np.clip((yc + ltrb[:, 3] * stride - pad_top) / scale, 0, orig_h)
                
                areas = (x2 - x1) * (y2 - y1)
                valid = areas > 100  # 排除小于 10x10 的微小图像噪点
                if not np.any(valid):
                    continue
                    
                boxes = np.stack([x1[valid], y1[valid], x2[valid], y2[valid]], axis=1)
                all_boxes.append(boxes)
                all_scores.extend(score_keep[valid].tolist())
                
            if not all_boxes:
                return []
                
            boxes = np.concatenate(all_boxes).tolist()
            # 在全图范围内通过 CPU 执行 NMS，抑制重叠重叠框
            indices = cv2.dnn.NMSBoxes(boxes, all_scores, CONF_THRESHOLD, IOU_THRESHOLD)
            if len(indices) == 0:
                return []
            return [{"box": boxes[i], "score": all_scores[i]} for i in indices.flatten()]
        except Exception as e:
            print(f"⚠️ [推理保护] 视频帧解码错误: {e}")
            return []
# 前端轻量化图传流服务器 (Flask Gateway Stream Server)
app = Flask(__name__)
CORS(app)  # 开启全源跨域支持，支持与 Web Web 仪表盘快速连通
frame_buffer = None
frame_lock = threading.Lock()  # 线程锁，防止底层写入时网络端读取导致画面撕裂

@app.route('/')
def index():
    """主控大屏嵌入式监控主页"""
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8">
        <title>智能时空防抖边缘网关</title>
        <style>
            body { margin: 0; background: #000; display: flex; justify-content: center; align-items: center; min-height: 100vh; }
            img { max-width: 100vw; max-height: 100vh; border: 2px solid #333; }
        </style>
    </head>
    <body><img src="/video_feed" alt="AI检测流"></body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    """利用 MJPEG 传输流协议，将画好的图片以极低延迟推向局域网浏览器"""
    def generate():
        while True:
            local_frame = None
            with frame_lock:
                if frame_buffer: local_frame = frame_buffer
            if local_frame:
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + local_frame + b'\r\n')
            time.sleep(0.03)  # 控制推流心跳帧率在 30FPS 左右
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

# 核心流式数据处理与防抖状态机 (Data Loop & Temporal-Spatial State Machine)

def process_frame():
    """多线程后台核心：全面负责视频采集、两级自适应过滤、防抖状态判定与外设控制"""
    global frame_buffer
    detector = YOLODetectorV2(MODEL_PATH)

    # 动态确认视频传感器输入源
    if len(sys.argv) > 1:
        video_device = sys.argv[1]
    else:
        video_device = find_first_usb_camera()

    cap = None
    if video_device is not None:
        cap = cv2.VideoCapture(video_device, cv2.CAP_V4L2)
        if cap.isOpened():
            codec = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
            cap.set(cv2.CAP_PROP_FOURCC, codec)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

    # 挂载底盘硬连接
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1, write_timeout=1)
        time.sleep(1)
        print(f"成功挂载硬件串口: {SERIAL_PORT}")
    except:
        ser = None
        print(f"WARNING: 串口 {SERIAL_PORT} 打开失败")

    WINDOW_SIZE = 15          # 时间滑窗大小
    TRIGGER_THRESHOLD = 11    # 15帧内必须包含 11 帧捕获才可触发
    RESET_THRESHOLD = 2       # 消失门槛限制
    
    detection_window = deque(maxlen=WINDOW_SIZE)  # 时间维度的捕获队列
    box_x_window = deque(maxlen=WINDOW_SIZE)      # 空间维度的X坐标移动轨迹监测队列
    
    # 队列全零填充初始化
    for _ in range(WINDOW_SIZE):
        detection_window.append(False)
        box_x_window.append(0.0)
        
    is_target_active = False   # 判定 AGV 是否处于紧急制动状态下的核心全局状态指针
    last_voice = 0
    last_serial_send = 0

    while True:
        # 防护：若摄像头物理掉线，自动渲染错误占位图推送至网页大屏
        if cap is None or not cap.isOpened():
            placeholder = np.zeros((640, 640, 3), dtype=np.uint8)
            cv2.putText(placeholder, "No Camera Connection", (120, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            _, jpeg = cv2.imencode('.jpg', placeholder, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            with frame_lock: frame_buffer = jpeg.tobytes()
            time.sleep(0.1)
            continue

        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.01)
            continue

        img_h, img_w = frame.shape[:2]
        raw_detections = detector.detect(frame)

        detections = []
        filtered_boxes_for_debug = []  # 缓存本帧被剥离拦截的碎砖头/道路噪声目标
        
        for d in raw_detections:
            box = d["box"]
            bw = box[2] - box[0]
            bh = box[3] - box[1]
            box_area = bw * bh
            ratio = bw / (bh + 1e-6)  # 宽高比精度保护
            
            # 远端超微小尺度噪点快速拦截机制
            if box_area < (img_w * img_h * 0.002):
                filtered_boxes_for_debug.append(box)
                continue
                
            # 路面红砖(扁平体)与横躺受困人体的完美多级形态判别
            # 若宽高比呈极度扁平状 (ratio > 3.0) 且面积偏小，则极大概率为路面红砖
            # 若宽高比极度扁平，但全图面积占比极大 (> 1.5%)，则属于近端严重倒地横躺目标，必须予以放行。
            if ratio > 3.0:
                if box_area < (img_w * img_h * 0.015):
                    filtered_boxes_for_debug.append(box)
                    continue  
                    
            # 极端畸形纵向框过滤（排除反光镜边缘畸变、雨刮器局部噪点干扰）
            if ratio < 0.12:
                filtered_boxes_for_debug.append(box)
                continue
                
            detections.append(d)
        # 高刚性时空防抖与静态僵死目标状态机处理 (Temporal State Machine)
        has_detection = len(detections) > 0
        detection_window.append(has_detection)
        
        if has_detection:
            box_x_window.append(float(detections[0]["box"][0]))
        else:
            box_x_window.append(-1.0)
            
        active_frames_in_window = sum(detection_window)
        
        # 算法亮点：针对相机镜头前附着树叶、昆虫、污渍等“僵死背景干扰”进行运动一阶微分轨迹检查
        is_stagnant_ghost = False
        if active_frames_in_window >= TRIGGER_THRESHOLD:
            valid_x = [x for x in box_x_window if x >= 0]
            if len(valid_x) >= 6:
                diffs = np.abs(np.diff(valid_x))
                max_diff = np.max(diffs)
                # 若连续十数帧在 X 轴上的绝对移动跨度近乎为 0 且小车在运动，断定该目标为镜头静态污渍污影
                if max_diff < 1e-4:
                    is_stagnant_ghost = True

        # 时序信号量的状态转移控制 (State Transition Control)
        if not is_target_active:
            # 只有当达到触发帧密度，且被判定为非静态僵死污渍时，才确立真实报警状态
            if active_frames_in_window >= TRIGGER_THRESHOLD and not is_stagnant_ghost:
                is_target_active = True
                print(f"🚀 [人体目标确认] 激活机器人停止信号！")
            elif is_stagnant_ghost:
                print(f"🛑 [防御成功] 拦截固定背景干扰")
        else:
            # 消失判定：当前窗口内捕获密度极低时，立刻解冻底盘，恢复自主行驶能力
            if active_frames_in_window <= RESET_THRESHOLD:
                is_target_active = False
                print(f"❄️ [复位解开] 目标消失，机器人恢复行驶")

        # 🎯 外设多态同步触发管理 (Peripherals Synchronous Actuation)
        if is_target_active:
            current_time = time.time()
            # 触发有冷却保护的高可靠性底盘硬件串口下发 (串口单向防拥堵通信)
            if ser and (current_time - last_serial_send > SERIAL_COOLDOWN):
                try:
                    ser.write(bytes([0x05]))  # 向机器人底盘控制主板抛送紧急停止控制原语: 0x05
                    last_serial_send = current_time
                    print(f"⚡ [串口] 发送 0x05 (停止指令)")
                except Exception as e:
                    print(f"串口发送失败: {e}")

            # 触发非阻塞低时延外置独立 USB 声卡系统广播播放
            if time.time() - last_voice > VOICE_COOLDOWN:
                voice_p = os.path.join(VOICE_FILE_DIR, VOICE_FILES["person"])
                if os.path.exists(voice_p):
                    current_card = get_usb_audio_card_index()
                    # 采用 Linux 原生 aplay 并配合 nohup 及后台多进程符 (&)，做到完全不阻塞视觉主循环的高实时性
                    os.system(f"nohup aplay -D plughw:{current_card},0 {voice_p} > /dev/null 2>&1 &")
                    last_voice = time.time()

        # 实时动态图传画面渲染与多状态语义标签打印 (OSD Overlay)
        # 绘制被系统成功卡死、过滤的路面红砖及杂物干扰 (橙色线条柔和显示)
        for box in filtered_boxes_for_debug:
            box = [int(x) for x in box]
            cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 165, 255), 1)
            cv2.putText(frame, "BRICK/MISC FILTERED", (box[0], max(15, box[1]-5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)

        # 绘制进入追踪器核心的生效目标框
        if detections:
            for d in detections:
                box = [int(x) for x in d["box"]]
                box[0], box[2] = np.clip([box[0], box[2]], 0, img_w - 1)
                box[1], box[3] = np.clip([box[1], box[3]], 0, img_h - 1)
                score = d["score"]
                
                # 依据当前时空状态机的判定结果，赋予目标不同的视觉外观
                if is_target_active:
                    box_color = (0, 255, 0)      # 真实威胁：高亮绿色
                    status_lbl = "[HUMAN TARGET]"
                elif is_stagnant_ghost:
                    box_color = (0, 0, 255)      # 僵死污点：鲜艳红色警告提示
                    status_lbl = "[STATIC GHOST]"
                else:
                    box_color = (0, 255, 255)    # 处于时序观测期：黄色
                    status_lbl = "[Checking]"
                    
                cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), box_color, 2)
                label_text = f"{CLASS_NAMES[0]}: {int(score * 100)}% {status_lbl}"
                
                # 精准像素级绘制 OSD 文本背景覆带，大幅改善在复杂光照环境下的可读性
                font_scale = 0.6
                font_thickness = 1
                (text_w, text_h), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
                lbl_top_left = (box[0], max(0, box[1] - text_h - 4))
                lbl_bottom_right = (min(img_w - 1, box[0] + text_w + 6), min(img_h - 1, max(text_h + 4, box[1])))
                cv2.rectangle(frame, lbl_top_left, lbl_bottom_right, box_color, -1)
                cv2.putText(frame, label_text, (box[0] + 3, lbl_bottom_right[1] - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)

        # 规范推流分辨率，对大图进行二次下采样防止打爆边缘网关网卡的上传带宽
        show_frame = cv2.resize(frame, (640, 640))
        _, jpeg = cv2.imencode('.jpg', show_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        with frame_lock: frame_buffer = jpeg.tobytes()
        time.sleep(0.01)



# 系统引导入口 (System Bootstrapper)
if __name__ == "__main__":
    # 初始化网关开机占位图片缓存
    init_placeholder = np.zeros((640, 640, 3), dtype=np.uint8)
    cv2.putText(init_placeholder, "Camera Initializing...", (140, 320), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    _, init_jpeg = cv2.imencode('.jpg', init_placeholder, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
    frame_buffer = init_jpeg.tobytes()

    # 将高负荷的核心视觉推理与时空流处理环路放入独立的 Daemon 守护线程，防止阻塞网络控制台
    threading.Thread(target=process_frame, daemon=True).start()
    
    local_ip = get_local_ip()
    print("=" * 70)
    print(" 🎉 开源智能边缘防抖网关核心部署就绪！")
    print(f" 📺 局域网实时监控 Web 仪表盘地址: http://{local_ip}:{STREAM_PORT}")
    print("=" * 70)
    
    # 启动多线程 Flask 核心 Web 引擎
    app.run(host='0.0.0.0', port=STREAM_PORT, threaded=True)
