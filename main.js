// ===============================
// 1️⃣ 连接 WebSocket（AP 电脑）
// ===============================
const ws = new WebSocket("ws://192.168.1.105:9002");

// ===============================
// 2️⃣ 连接状态
// ===============================
ws.onopen = () => {
    console.log("✅ WebSocket 已连接到 192.168.1.105:9002");
};

ws.onclose = () => {
    console.log("❌ WebSocket 连接已断开");
};

ws.onerror = (err) => {
    console.error("⚠️ WebSocket 错误：", err);
};

// ===============================
// 3️⃣ 接收机械臂数据
// ===============================
ws.onmessage = (event) => {
    const data = JSON.parse(event.data);

    /*
      data 示例：
      {
        joints: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        image: "base64字符串",
        timestamp: 1700000000
      }
    */

    updateJointInfo(data.joints);
    updateImage(data.image);
};

// ===============================
// 4️⃣ 更新关节数据显示
// ===============================
function updateJointInfo(joints) {
    document.getElementById("joints").innerText =
        joints.map((j, i) => `J${i + 1}: ${j.toFixed(3)}`).join(" | ");
}

// ===============================
// 5️⃣ 更新摄像头/仿真图像
// ===============================
function updateImage(base64Img) {
    const img = document.getElementById("camera");
    img.src = "data:image/jpeg;base64," + base64Img;
}
