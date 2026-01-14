const ws = new WebSocket("ws://AP电脑IP:9002");

ws.onopen = () => {
  document.getElementById("status").innerText = "系统状态：已连接";
};

ws.onmessage = (event) => {
  document.getElementById("data").innerText = event.data;
};

function sendCmd() {
  ws.send(JSON.stringify({
    cmd: "move",
    joint: 1,
    value: 0.5
  }));
}
