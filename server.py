from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
import time
import json
import os
from pathlib import Path
from typing import Optional, List
import uvicorn
import asyncio
import base64
from PIL import Image
import io

app = FastAPI(title="机械臂数字孪生后端")

# 允许所有跨域请求（局域网用）
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 局域网临时用*
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 挂载前端静态文件目录，提供 /ui 访问前端页面（使用 HTTP 避免 GitHub Pages 的 HTTPS 导致 ws:// 被阻止）
try:
    # 使用脚本文件所在目录构造绝对路径，避免工作目录不同导致的 Not Found
    SCRIPT_DIR = Path(__file__).resolve().parent
    STATIC_DIR = SCRIPT_DIR / "Github"
    if STATIC_DIR.exists():
        app.mount("/ui", StaticFiles(directory=str(STATIC_DIR), html=True), name="ui")
        print(f"📁 前端静态文件已挂载: /ui -> {STATIC_DIR}")
    else:
        print(f"⚠️ 静态文件目录不存在: {STATIC_DIR}，请确认路径是否正确")
except Exception as e:
    print(f"⚠️ 无法挂载静态文件目录: {e}")

# WebSocket 连接管理
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        print(f"✅ 新的 WebSocket 连接，当前连接数: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
        print(f"❌ WebSocket 断开，当前连接数: {len(self.active_connections)}")

    async def broadcast(self, message: dict):
        """广播消息到所有连接的客户端"""
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception as e:
                print(f"发送失败: {e}")
                disconnected.append(connection)
        
        # 清理断开的连接
        for conn in disconnected:
            if conn in self.active_connections:
                self.active_connections.remove(conn)

manager = ConnectionManager()

# 数据存储
LATEST_DATA = {
    "timestamp": time.time(),
    "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "reward": 0.0,
    "mode": "idle",
    "episode": 0,
    "step": 0,
    "position": [0.0, 0.0, 0.0],
    "velocity": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "target": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "image_base64": None
}

# 存储接收到的命令
LAST_COMMAND = None

# 图像路径
IMAGE_PATH = Path("latest.jpg")

# 奖励历史记录（用于绘制曲线）
REWARD_HISTORY = []
MAX_HISTORY = 100


class StateUpdate(BaseModel):
    joints: list[float]
    reward: Optional[float] = 0.0
    mode: Optional[str] = "idle"
    episode: Optional[int] = 0
    step: Optional[int] = 0
    position: Optional[list[float]] = None
    velocity: Optional[list[float]] = None
    target: Optional[list[float]] = None


class ImageUpload(BaseModel):
    image_base64: str
    timestamp: Optional[float] = None


class Command(BaseModel):
    action: str  # "start", "stop", "move", "reset"
    joint_values: Optional[list[float]] = None
    speed: Optional[float] = 1.0


# WebSocket 端点（网页连接）
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        # 发送初始数据
        await websocket.send_json(LATEST_DATA)
        
        # 保持连接，接收客户端消息
        while True:
            data = await websocket.receive_text()
            print(f"收到 WebSocket 消息: {data}")
            # 可以处理客户端发来的命令
            
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        print(f"WebSocket 错误: {e}")
        manager.disconnect(websocket)


# 根路径
@app.get("/")
def root():
    return {
        "service": "Robotic Arm Digital Twin API",
        "version": "2.0",
        "websocket": "ws://localhost:8000/ws",
        "connections": len(manager.active_connections),
        "endpoints": {
            "WebSocket /ws": "实时数据推送",
            "GET /state": "获取当前状态",
            "POST /update": "更新状态 (MATLAB使用)",
            "GET /image": "获取最新图像",
            "POST /upload_image_base64": "上传图像 (MATLAB使用)",
            "POST /command": "发送控制命令",
            "GET /command": "获取最新命令",
            "GET /reward_history": "获取奖励历史"
        }
    }


# 健康检查
@app.get("/health")
def health_check():
    return {
        "status": "ok",
        "connections": len(manager.active_connections),
        "timestamp": time.time()
    }


# 获取状态
@app.get("/state")
def get_state():
    return LATEST_DATA


# 获取奖励历史
@app.get("/reward_history")
def get_reward_history():
    return {
        "history": REWARD_HISTORY,
        "length": len(REWARD_HISTORY)
    }


# 更新状态（MATLAB调用）
@app.post("/update")
async def update_state(state: StateUpdate):
    global REWARD_HISTORY
    
    # 更新数据
    LATEST_DATA.update({
        "timestamp": time.time(),
        "joints": state.joints,
        "reward": state.reward,
        "mode": state.mode,
        "episode": state.episode,
        "step": state.step
    })
    
    if state.position:
        LATEST_DATA["position"] = state.position
    if state.velocity:
        LATEST_DATA["velocity"] = state.velocity
    if state.target:
        LATEST_DATA["target"] = state.target
    
    # 记录奖励历史
    REWARD_HISTORY.append({
        "step": state.step,
        "episode": state.episode,
        "reward": state.reward,
        "timestamp": time.time()
    })
    
    # 限制历史记录长度
    if len(REWARD_HISTORY) > MAX_HISTORY:
        REWARD_HISTORY = REWARD_HISTORY[-MAX_HISTORY:]
    
    print(f"📊 状态更新: Episode {state.episode}, Step {state.step}, 模式: {state.mode}, 奖励: {state.reward:.3f}")
    
    # 广播到所有 WebSocket 客户端
    await manager.broadcast(LATEST_DATA)
    
    return {"status": "success", "message": "State updated and broadcasted"}


# 获取图像
@app.get("/image")
def get_image():
    if IMAGE_PATH.exists():
        return FileResponse(IMAGE_PATH)
    else:
        # 返回空响应
        raise HTTPException(status_code=404, detail="No image available")


# 上传图像（MATLAB调用）- Base64格式
@app.post("/upload_image_base64")
async def upload_image_base64(data: dict):
    """接收base64编码的图像"""
    try:
        image_b64 = None
        
        # 处理不同的数据格式
        if 'image_base64' in data:
            image_b64 = data['image_base64']
        elif 'image' in data:
            image_b64 = data['image']
        
        if not image_b64:
            return {"status": "error", "message": "No image data provided"}
        
        # 移除 data:image/jpeg;base64, 前缀（如果有）
        if ',' in image_b64:
            image_b64 = image_b64.split(',')[1]
        
        # 解码base64
        image_data = base64.b64decode(image_b64)
        img = Image.open(io.BytesIO(image_data))
        
        # 保存为文件
        img.save(IMAGE_PATH, "JPEG")
        
        # 更新全局数据中的图像
        LATEST_DATA["image_base64"] = image_b64
        
        print(f"📷 图像已更新，大小: {len(image_b64)} bytes")
        
        # 广播到所有 WebSocket 客户端
        await manager.broadcast(LATEST_DATA)
        
        return {"status": "success", "message": "Image saved and broadcasted"}
    except Exception as e:
        print(f"图像上传错误: {e}")
        return {"status": "error", "message": str(e)}


# 发送控制命令（网页调用）
@app.post("/command")
async def send_command(cmd: Command):
    global LAST_COMMAND
    LAST_COMMAND = {
        "timestamp": time.time(),
        "action": cmd.action,
        "joint_values": cmd.joint_values,
        "speed": cmd.speed,
        "processed": False
    }
    print(f"🎮 收到命令: {cmd.action}, 关节值: {cmd.joint_values}")
    
    # 广播命令到所有客户端
    await manager.broadcast({
        "type": "command",
        "command": LAST_COMMAND
    })
    
    return {"status": "success", "command": LAST_COMMAND}


# 获取最新命令（MATLAB轮询）
@app.get("/command/latest")
def get_latest_command():
    if LAST_COMMAND and not LAST_COMMAND.get("processed", False):
        return {
            "has_command": True,
            "command": LAST_COMMAND
        }
    return {
        "has_command": False,
        "command": None
    }


# 标记命令已处理
@app.post("/command/acknowledge")
def acknowledge_command():
    global LAST_COMMAND
    if LAST_COMMAND:
        LAST_COMMAND["processed"] = True
    return {"status": "success"}


if __name__ == "__main__":
    print("=" * 60)
    print("🚀 启动机械臂数字孪生后端服务器")
    print("=" * 60)
    print(f"📡 HTTP API: http://0.0.0.0:8000")
    print(f"🔌 WebSocket: ws://0.0.0.0:8000/ws")
    print(f"📖 API文档: http://localhost:8000/docs")
    print("=" * 60)
    print("⚠️  请确保在网页中配置正确的局域网IP地址")
    print("=" * 60)
    
    uvicorn.run(app, host="0.0.0.0", port=8000)
