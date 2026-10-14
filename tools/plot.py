#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
DRV5053 实时电压/位置曲线监视器 (零第三方依赖, 仅需 pyserial)

用法:
    python tools/plot.py [COM口] [波特率]
    默认自动查找第一个可用串口, 115200

运行后浏览器自动打开 (或手动访问) http://localhost:8080

网页功能:
    滚轮          缩放时间轴 (1~60s)
    启动/停止监视 控制串口连接 (停止后释放 COM 口供烧录用)
    标定输入框    两点标定电压->位置映射, 自动保存

注意: 运行前请关闭 VS Code 的串口监视器, 避免 COM 口被占用
"""
import collections
import json
import socket
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial
from serial.tools import list_ports

HTTP_PORT = 8080
BUF_MAX = 40000            # 缓存点数 (500Hz x 80s, 供放大后回看)

points = collections.deque(maxlen=BUF_MAX)   # (t_ms, v_raw, v_filt)
buf_lock = threading.Lock()
mon = {"want": True, "running": False}       # want=用户意图 running=串口实际状态

HTML = r"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>DRV5053 实时电压/位置曲线</title>
<style>
  body { margin:0; background:#14181d; color:#cfd8e3;
         font:14px/1.4 "Segoe UI", "Microsoft YaHei", sans-serif; }
  header { background:#1c2229; border-bottom:1px solid #2b333d; }
  .row { display:flex; gap:18px; align-items:center; padding:7px 16px;
         flex-wrap:wrap; }
  .row + .row { border-top:1px solid #242b33; }
  header h1 { font-size:15px; margin:0; }
  .val b { font-size:18px; font-family:Consolas, monospace; }
  #vraw b { color:#8fa3b8; } #vfilt b { color:#4ade80; } #pos b { color:#fb923c; }
  #status { margin-left:auto; color:#9fb0c0; font-size:13px; }
  .cal { color:#8b98a5; font-size:12px; display:flex; gap:6px; align-items:center; }
  .cal input { width:58px; background:#0e1216; color:#cfd8e3; border:1px solid #39424d;
               border-radius:4px; padding:2px 5px; font:12px Consolas, monospace; }
  button { background:#2b6cb0; color:#fff; border:none; border-radius:4px;
           padding:4px 16px; font-size:13px; cursor:pointer; }
  button:disabled { background:#39424d; color:#7a8794; cursor:default; }
  canvas { display:block; width:100vw; height:calc(100vh - 86px); }
</style>
</head>
<body>
<header>
  <div class="row">
    <h1>DRV5053 曲线</h1>
    <div class="val" id="vraw">原始 <b>--</b> V</div>
    <div class="val" id="vfilt">滤波 <b>--</b> V</div>
    <div class="val" id="pos">位置 <b>--</b> mm</div>
    <div class="val">窗口 <b id="win">15s</b> <span style="color:#7a8794;font-size:11px">(滚轮缩放 1~60s)</span></div>
    <div class="val">接收 <b id="rate">--</b><span style="color:#7a8794;font-size:11px">/s</span></div>
    <div id="status">连接中...</div>
  </div>
  <div class="row">
    <div class="cal">标定 远点 <input id="cvf" type="number" step="0.001" value="0.997"> V = <input id="cmf" type="number" step="0.1" value="0"> mm</div>
    <div class="cal">近点 <input id="cvn" type="number" step="0.001" value="0.015"> V = <input id="cmn" type="number" step="0.1" value="10"> mm</div>
    <label class="cal"><input type="checkbox" id="showpos" checked> 位置曲线</label>
    <div style="flex:1"></div>
    <button id="btnStart">启动监视</button>
    <button id="btnStop">停止监视</button>
  </div>
</header>
<canvas id="cv"></canvas>
<script>
const cv = document.getElementById('cv'), ctx = cv.getContext('2d');
const data = [];          // [t, raw, filt]
let lastIdx = 0;

// ---------- 标定: 两点线性映射 电压->位置 ----------
const calIds = ['cvf', 'cmf', 'cvn', 'cmn'];
const cal = {};
for (const id of calIds) {
  const el = document.getElementById(id);
  cal[id] = parseFloat(localStorage.getItem(id)) || parseFloat(el.value);
  el.value = cal[id];
  el.onchange = () => { cal[id] = parseFloat(el.value) || 0;
                        localStorage.setItem(id, el.value); };
}
function voltToPos(v) {
  const dv = cal.cvn - cal.cvf;
  if (!dv) return null;
  return cal.cmf + (v - cal.cvf) * (cal.cmn - cal.cmf) / dv;
}
const showPos = document.getElementById('showpos');
showPos.checked = localStorage.getItem('showpos') !== '0';
showPos.onchange = () => localStorage.setItem('showpos', showPos.checked ? '1' : '0');

// ---------- 时间轴缩放 ----------
let windowMs = 15000;
const WIN_MIN = 1000, WIN_MAX = 60000, HISTORY_MS = 72000;
cv.addEventListener('wheel', (e) => {
  e.preventDefault();
  const f = e.deltaY > 0 ? 1.25 : 0.8;
  windowMs = Math.min(WIN_MAX, Math.max(WIN_MIN, windowMs * f));
  document.getElementById('win').textContent =
    (windowMs / 1000).toFixed(windowMs < 10000 ? 1 : 0) + 's';
}, { passive: false });

function resize() {
  const dpr = window.devicePixelRatio || 1;
  cv.width = cv.clientWidth * dpr; cv.height = cv.clientHeight * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener('resize', resize); resize();

// ---------- 数据流 ----------
let recvCount = 0, lastMsgAt = Date.now();
let wantMon = true, serverAlive = false, src = null;

// 心跳: 显示接收速率; 检测到断流(Chrome冻结标签页会杀死SSE且不重连)
// 主动重建连接, 服务器缓存可恢复最多80秒历史
setInterval(() => {
  document.getElementById('rate').textContent = recvCount;
  recvCount = 0;
  if (wantMon && serverAlive && Date.now() - lastMsgAt > 6000) {
    document.getElementById('status').textContent = '连接中断, 重连中...';
    connectSSE();
  }
}, 2000);

function connectSSE() {
  if (src) src.close();
  src = new EventSource('/events');
  src.onopen = () => { lastMsgAt = Date.now(); };
  src.onerror = () => { src.close(); lastMsgAt -= 7000; };  // 促使心跳尽快重建
  src.onmessage = (e) => {
    lastMsgAt = Date.now();
    const msg = JSON.parse(e.data);
    recvCount += msg.data.length;
    if (msg.start < lastIdx) data.length = 0;
    lastIdx = msg.start + msg.data.length;
    // 防线: 设备复位(millis归零)导致时间戳倒退 -> 清空画面重绘
    if (data.length && msg.data.length &&
        msg.data[0][0] < data[data.length-1][0] - 2000) {
      data.length = 0;
      lastIdx = msg.start + msg.data.length;
    }
    for (const p of msg.data) data.push(p);
    const cutoff = data.length ? data[data.length-1][0] - HISTORY_MS : 0;
    while (data.length && data[0][0] < cutoff) data.shift();
    if (msg.data.length) {
      const last = msg.data[msg.data.length-1];
      document.querySelector('#vraw b').textContent = last[1].toFixed(3);
      document.querySelector('#vfilt b').textContent = last[2].toFixed(3);
      const p = voltToPos(last[2]);
      document.querySelector('#pos b').textContent = p === null ? '--' : p.toFixed(2);
    }
  };
}
connectSSE();

// ---------- 启动/停止监视 ----------
async function pollStatus() {
  try {
    const j = await (await fetch('/status')).json();
    serverAlive = true;
    wantMon = j.want;
    const st = document.getElementById('status');
    if (j.running)      { st.textContent = '监视中'; }
    else if (j.want)    { st.textContent = '连接中...'; }
    else                { st.textContent = '已停止'; }
    document.getElementById('btnStart').disabled = j.want;
    document.getElementById('btnStop').disabled = !j.want;
  } catch {
    serverAlive = false;
    document.getElementById('status').textContent = '服务无响应';
  }
}
document.getElementById('btnStart').onclick = () =>
  fetch('/control?action=start').then(pollStatus);
document.getElementById('btnStop').onclick = () =>
  fetch('/control?action=stop').then(pollStatus);
setInterval(pollStatus, 1000);
pollStatus();

// ---------- 绘图 ----------
const Y_MAX = 2.0;      // 左轴: 电压 0~2V (5053 满量程 0.2~1.8V)
function draw() {
  const W = cv.clientWidth, H = cv.clientHeight;
  ctx.clearRect(0, 0, W, H);
  const padL = 46, padR = 56, padT = 12, padB = 26;
  const pw = W - padL - padR, ph = H - padT - padB;
  const y = v => padT + ph * (1 - v / Y_MAX);

  // 网格 + 左轴电压刻度
  ctx.font = '11px Consolas, monospace'; ctx.textAlign = 'right';
  for (let v = 0; v <= Y_MAX + 0.001; v += 0.2) {
    const yy = y(v);
    ctx.strokeStyle = v === 0 ? '#39424d' : '#242b33';
    ctx.beginPath(); ctx.moveTo(padL, yy); ctx.lineTo(W - padR, yy); ctx.stroke();
    ctx.fillStyle = '#66727f'; ctx.fillText(v.toFixed(1), padL - 6, yy + 3);
  }
  // 参考线: QVO 1.0V (红虚线), 线性区边界 0.2/1.8V (黄点线)
  for (const [v, c, dash] of [[1.0, '#ef5350', [6,4]], [0.2, '#c9a227', [2,4]], [1.8, '#c9a227', [2,4]]]) {
    ctx.strokeStyle = c; ctx.setLineDash(dash);
    ctx.beginPath(); ctx.moveTo(padL, y(v)); ctx.lineTo(W - padR, y(v)); ctx.stroke();
    ctx.setLineDash([]);
  }
  ctx.fillStyle = '#ef5350'; ctx.textAlign = 'left';
  ctx.fillText('QVO 1.0V', padL + 4, y(1.0) - 4);
  // 图例
  ctx.font = '12px "Segoe UI", "Microsoft YaHei", sans-serif';
  const legend = [['rgba(143,163,184,0.6)', '原始'], ['#4ade80', '滤波'],
                  ...(showPos.checked && cal.cvn !== cal.cvf ? [['#fb923c', '位置']] : [])];
  let lx = padL + 90;
  for (const [c, txt] of legend) {
    ctx.strokeStyle = c; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(lx, y(1.0) - 6); ctx.lineTo(lx + 18, y(1.0) - 6); ctx.stroke();
    ctx.lineWidth = 1; ctx.fillStyle = '#8b98a5'; ctx.fillText(txt, lx + 22, y(1.0) - 2);
    lx += 70;
  }

  // 右轴: 位置刻度
  const mmLo = Math.min(cal.cmf, cal.cmn), mmHi = Math.max(cal.cmf, cal.cmn);
  ctx.textAlign = 'left'; ctx.fillStyle = '#b0763a';
  for (let i = 0; i <= 4; i++) {
    const mm = mmLo + (mmHi - mmLo) * i / 4;
    const yy = padT + ph * (1 - i / 4);
    ctx.fillText(mm.toFixed(1), W - padR + 6, yy + 3);
  }
  ctx.save(); ctx.translate(W - 10, padT + ph / 2); ctx.rotate(Math.PI / 2);
  ctx.textAlign = 'center'; ctx.fillText('mm', 0, 0); ctx.restore();

  if (data.length < 2) return;

  const t1 = data[data.length-1][0], t0 = t1 - windowMs;
  const x = t => padL + pw * (t - t0) / windowMs;

  // 时间轴刻度
  const steps = [100, 200, 500, 1000, 2000, 5000, 10000, 15000, 30000];
  let step = steps[0];
  for (const s of steps) { step = s; if (windowMs / s <= 12) break; }
  ctx.textAlign = 'center'; ctx.fillStyle = '#66727f';
  for (let tt = Math.ceil(t0 / step) * step; tt <= t1; tt += step) {
    const ago = (t1 - tt) / 1000;
    ctx.fillText((windowMs < 10000 ? ago.toFixed(1) : ago.toFixed(0)) + 's前',
                 x(tt), H - 8);
  }

  // 电压曲线: 原始(灰, 半透明作噪声参考) / 滤波(绿)
  for (const [idx, c, lw] of [[1, 'rgba(143,163,184,0.35)', 1], [2, '#4ade80', 2]]) {
    ctx.strokeStyle = c; ctx.lineWidth = lw; ctx.beginPath();
    let started = false;
    for (const p of data) {
      if (p[0] < t0) continue;
      const px = x(p[0]), py = y(Math.min(p[idx], Y_MAX));
      if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
    }
    ctx.stroke();
  }
  ctx.lineWidth = 1;

  // 位置曲线 (右轴, 橙色虚线)
  if (showPos.checked && cal.cvn !== cal.cvf) {
    ctx.strokeStyle = '#fb923c'; ctx.setLineDash([4, 3]); ctx.beginPath();
    let started = false;
    for (const p of data) {
      if (p[0] < t0) continue;
      const mm = voltToPos(p[2]);
      if (mm === null) continue;
      const py = padT + ph * (1 - (mm - mmLo) / (mmHi - mmLo || 1));
      const px = x(p[0]);
      if (!started) { ctx.moveTo(px, py); started = true; } else ctx.lineTo(px, py);
    }
    ctx.stroke(); ctx.setLineDash([]);
  }
}
// 绘图调度: 前台用rAF(流畅60fps); 后台标签页Chrome会暂停rAF,
// 用定时器兜底慢速刷新(~1fps), 保证监视画面持续更新
// try/catch兜底: 单次绘制异常不得杀死渲染循环
let lastDraw = 0;
function safeDraw() { try { draw(); } catch (e) {} lastDraw = Date.now(); }
function rafLoop() { safeDraw(); requestAnimationFrame(rafLoop); }
requestAnimationFrame(rafLoop);
setInterval(() => { if (Date.now() - lastDraw > 400) safeDraw(); }, 200);
</script>
</body>
</html>"""


def open_port(port_name, baud):
    """打开串口并复位板子到正常运行状态。

    pyserial 打开端口时会短暂拉高 DTR/RTS, ESP32-S3 原生 USB 可能因此
    进入下载模式; 打开后立刻释放 DTR/RTS, 等待板子重启进应用再开始读。
    """
    ser = serial.Serial(port_name, baud, timeout=1)
    ser.setDTR(False)
    ser.setRTS(False)
    time.sleep(1.5)                 # 等板子复位并跑起来
    ser.reset_input_buffer()
    return ser


def serial_reader(port_name, baud):
    ser = None
    while True:
        if not mon["want"]:
            # 停止状态: 释放串口, 等待用户启动
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                mon["running"] = False
                print("[信息] 监视已停止, 串口已释放", flush=True)
            time.sleep(0.3)
            continue
        if ser is None:
            try:
                print(f"[信息] 打开 {port_name} @ {baud} ...", flush=True)
                ser = open_port(port_name, baud)
                mon["running"] = True
                # 重连意味着板子已复位, millis() 归零: 清空旧时间轴, 防止新旧数据混绘
                with buf_lock:
                    points.clear()
                print("[信息] 串口就绪, 等待数据...", flush=True)
            except (serial.SerialException, OSError) as e:
                print(f"[警告] 无法打开串口: {e}, 2秒后重试", flush=True)
                print("提示: 检查 VS Code 串口监视器是否已关闭", flush=True)
                time.sleep(2)
            continue
        try:
            line = ser.readline().decode("ascii", "ignore").strip()
        except (serial.SerialException, OSError) as e:
            print(f"[警告] 串口异常: {e}, 1秒后重连", flush=True)
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            mon["running"] = False
            time.sleep(1)
            continue
        if not line:
            continue                    # 1s 无数据 (板子复位/下载模式), 继续等
        parts = line.split(",")
        if len(parts) != 3:
            continue
        try:
            t, vr, vf = int(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            continue                    # 跳过文本行 (启动横幅等)
        with buf_lock:
            points.append((t, vr, vf))


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _json(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/events":
            self.serve_events()
        elif self.path == "/status":
            self._json({"running": mon["running"], "want": mon["want"]})
        elif self.path.startswith("/control"):
            action = ""
            if "?" in self.path:
                for kv in self.path.split("?", 1)[1].split("&"):
                    if kv.startswith("action="):
                        action = kv.split("=", 1)[1]
            if action == "start":
                mon["want"] = True
                print("[信息] 用户启动监视", flush=True)
            elif action == "stop":
                mon["want"] = False
                print("[信息] 用户停止监视", flush=True)
            self._json({"ok": True, "want": mon["want"]})
        else:
            self.send_response(404)
            self.end_headers()

    def serve_events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        sent = 0
        try:
            while True:
                with buf_lock:
                    total = len(points)
                    if total < sent:
                        sent = 0         # 缓存已被清空(重启新会话), 本连接从头推送
                    batch = [list(p) for p in list(points)[sent:]]
                if batch:
                    payload = json.dumps({"start": sent, "data": batch},
                                         separators=(",", ":"))
                    self.wfile.write(f"data: {payload}\n\n".encode())
                    self.wfile.flush()
                    sent = total
                time.sleep(0.04)        # 25Hz 推送
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


def pick_port():
    ports = list(list_ports.comports())
    if not ports:
        return None
    # 优先选 ESP32-S3 原生 USB (VID 0x303A), 否则第一个串口
    for p in ports:
        if p.vid == 0x303A:
            return p.device
    return ports[0].device


def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else pick_port()
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    if not port_name:
        print("[错误] 找不到可用串口", flush=True)
        sys.exit(1)

    # 端口占用预检: Windows 的 SO_REUSEADDR 会让双实例"静默共存"导致混乱,
    # 这里用不带 REUSE 的裸绑定强制独占
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind(("127.0.0.1", HTTP_PORT))
        sock.close()
    except OSError:
        print(f"[错误] 端口 {HTTP_PORT} 已被占用, 请先关闭另一个 plot.bat 实例", flush=True)
        sys.exit(1)

    threading.Thread(target=serial_reader, args=(port_name, baud),
                     daemon=True).start()

    server = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    print(f"[信息] 曲线页面: http://localhost:{HTTP_PORT}", flush=True)
    webbrowser.open(f"http://localhost:{HTTP_PORT}")
    server.serve_forever()


if __name__ == "__main__":
    main()
