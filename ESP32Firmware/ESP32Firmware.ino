#include <SPI.h>
#include <TFT_eSPI.h>
#include <driver/spi_slave.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
unsigned long last_data_time = 0;
unsigned long last_debug_time = 0;
int valid_packet_count = 0;
// ==========================================
// 1. CẤU HÌNH PHẦN CỨNG & STRUCT (DÙNG CHUNG)
// ==========================================
#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS   15
#define SAMPLES_PER_CH 1024

typedef struct __attribute__((packed)) {
    uint8_t header[2];           // 0xAA, 0xBB
    uint16_t ch1[SAMPLES_PER_CH]; // 2048 bytes
    uint16_t ch2[SAMPLES_PER_CH]; // 2048 bytes
    uint8_t padding[2];          // 2 bytes padding
    
    // Các thông số đo
    float vpp1, vpp2;            // 8 bytes
    float freq1, freq2;          // 8 bytes
    float vavg1, vavg2;          // 8 bytes
    float vrms1, vrms2;          // 8 bytes
    float vamp1, vamp2;          // 8 bytes
    
    uint16_t trigger_idx1;       // Trigger CH1
    uint16_t trigger_idx2;       // Trigger CH2
    
    // Khối điều khiển
    uint8_t hold_flag;           // Hold flag
    uint8_t ch_mode;             // Channel mode
    float y_scale1;              // Voltage scale CH1
    float y_scale2;              // Voltage scale CH2
    float x_scale1;              // Time scale CH1
    float x_scale2;              // Time scale CH2
    int16_t y_offset1;           // Vertical offset CH1
    int16_t y_offset2;           // Vertical offset CH2
    
    // Khối offset X/Y
    uint8_t offset_axis;         // 0: Y-axis, 1: X-axis
    int16_t x_offset1;           // Horizontal offset CH1
    int16_t x_offset2;           // Horizontal offset CH2
    uint8_t reset_flag;          // Reset flag
    
    uint8_t footer[2];           // 0xCC, 0xDD
} Packet_t;

// Cấu trúc lệnh gửi từ ESP32 -> STM32
typedef struct __attribute__((packed)) {
    uint8_t cmd;      
    int8_t val;       
    uint8_t unused[30];
} Command_t;
Packet_t stm_data; 
Command_t cmd_to_send;
volatile bool has_new_cmd = false;

portMUX_TYPE dataMutex = portMUX_INITIALIZER_UNLOCKED;

#define PACKET_SIZE sizeof(Packet_t)
WORD_ALIGNED_ATTR uint8_t rx_buf[PACKET_SIZE + 20];
WORD_ALIGNED_ATTR uint8_t tx_buf[PACKET_SIZE + 20];
bool data_valid = false;

// ==========================================
// 2. KHAI BÁO BIẾN CHO LCD (Từ LCD.txt)
// ==========================================
TFT_eSPI tft = TFT_eSPI();
uint16_t prev_y1[385]; 
uint16_t prev_y2[385];

// ==========================================
// 3. KHAI BÁO BIẾN & HTML CHO WEB (Từ WEB.txt)
// ==========================================
const char* ssid = "STM32-Scope-FETEL";
const char* password = "12345678";
WebServer server(80);

String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>HCMUS - ESP32 Oscilloscope</title>
<style>
body{margin:0;background:#0a0a0a;color:white;font-family:Segoe UI;display:flex;justify-content:center;padding:10px}
.frame{width:100%;max-width:900px;background:#1e1e1e;border-radius:10px;padding:15px}
.logo-header {
    text-align: center;
    margin-bottom: 20px;
    padding: 15px;
    background: linear-gradient(135deg, #1e2a3a 0%, #0a1a2a 100%);
    border-radius: 10px;
    border-bottom: 3px solid #00d7ff;
}
.logo-header .title {
    font-size: 24px;
    font-weight: bold;
    margin-left: 15px;
    color: #00d7ff;
    vertical-align: middle;
}
.screen{background:black;height:350px;border:2px solid #444;border-radius:8px;position:relative}
canvas{width:100%;height:100%;cursor:crosshair}
.topbar{position:absolute;top:10px;left:10px;font-size:12px;color:#00ff00}
.measure{position:absolute;bottom:10px;right:10px;font-size:12px;color:#00ff00;background:rgba(0,0,0,0.7);padding:5px;border-radius:5px}
.panel{margin-top:10px;display:grid;grid-template-columns:repeat(4,1fr);gap:6px}
button{padding:10px;background:#333;color:white;border:none;border-radius:5px;font-size:12px;cursor:pointer}
button:active{background:#00d7ff;color:black}
button.active{background:#ffcc00;color:black}
.footer{margin-top:10px;display:flex;justify-content:space-between;font-size:12px}
.cursorLabel{position:absolute;font-size:11px;background:rgba(0,0,0,0.6);padding:2px 4px;border-radius:3px}
</style>
</head>

<body>
<div class="logo-header">
    <span class="title">FETEL HCMUS Oscilloscope</span>
    <div class="subtitle">STM32F303 - ESP32 Dual Core Scope</div>
</div>

<div class="frame">
<div class="screen">
<div class="topbar" id="topbar">STM32 SCOPE | READY</div>
<canvas id="scope"></canvas>

<div id="labelA" class="cursorLabel" style="color:#ffff00"></div>
<div id="labelB" class="cursorLabel" style="color:#ffff00"></div>
<div id="labelV1" class="cursorLabel" style="color:#00ff00"></div>
<div id="labelV2" class="cursorLabel" style="color:#00ff00"></div>

<div class="measure">
<div style="color:#ffff00">CH1 Freq: <span id="freq1">0</span> Hz | Vpp: <span id="vpp1">0</span> V | Vavg: <span id="vavg1">0</span> V | Vrms: <span id="vrms1">0</span> V</div>
<div style="color:#00ffff">CH2 Freq: <span id="freq2">0</span> Hz | Vpp: <span id="vpp2">0</span> V | Vavg: <span id="vavg2">0</span> V | Vrms: <span id="vrms2">0</span> V</div>
<div style="color:#ffff00">Δt: <span id="dt">0</span> us | ΔV: <span id="dv">0</span> V</div>
</div>
</div>

<div class="panel">
<button onclick="sendCmd(1, 1)">TIME +</button>
<button onclick="sendCmd(1, -1)">TIME -</button>
<button onclick="sendCmd(2, 1)">VOLT CH1 +</button>
<button onclick="sendCmd(2, -1)">VOLT CH1 -</button>
<button onclick="sendCmd(8, 1)">VOLT CH2 +</button>
<button onclick="sendCmd(8, -1)">VOLT CH2 -</button>
<button onclick="sendCmd(6, 1)">OFFSET +</button>
<button onclick="sendCmd(6, -1)">OFFSET -</button>
<button id="cursorBtn" onclick="toggleCursor()">CURSOR</button>
<button id="holdBtn" onclick="sendCmd(3, 0)">HOLD</button>
<button id="ch1btn" class="active" onclick="toggleCh(1)">CH1</button>
<button id="ch2btn" class="active" onclick="toggleCh(2)">CH2</button>
</div>

<div class="footer">
<div style="color:#ffff00">CH1: <span id="voltLabel">1.0V/div</span> | Offset: <span id="offset1">0</span>px</div>
<div style="color:#00ffff">CH2: <span id="voltLabel2">1.0V/div</span> | Offset: <span id="offset2">0</span>px</div>
<div id="timeLabel" style="color:#ffffff">Time:100us/div</div>
<div id="runstate" style="color:#ffffff">RUN</div>
</div>
</div>

<script>
const canvas = document.getElementById("scope");
const ctx = canvas.getContext("2d");

const ADC_MAX = 4095;
const VREF = 3.3;

// Dữ liệu thô từ Server
let waveData = [];
let waveData2 = [];

// --- LOGIC ĐIỀU KHIỂN ĐỘC LẬP CHO WEB ---
let vDiv1 = 1.0;   // Volt/div CH1
let vDiv2 = 1.0;   // Volt/div CH2
let tDiv = 100;    // us/div (mặc định)
let ch1Offset = 0; // pixel
let ch2Offset = 0; // pixel
let webTimeZoom = 1.0; // Hệ số zoom trục X cục bộ trên web

let showCh1 = true;
let showCh2 = true;
let hold = false;

// --- CURSOR LOGIC ---
let cursorMode = 0; // 0:Off, 1:Time, 2:Volt
let t1 = 0.3, t2 = 0.7; // Tọa độ % trục X (0.0 -> 1.0)
let y1 = 0.3, y2 = 0.7; // Tọa độ % trục Y (0.0 -> 1.0)
let dragging = null;

// Mảng các nấc giá trị chuẩn cho Oscilloscope
const vSteps = [0.1, 0.2, 0.5, 1.0, 2.0, 5.0];
const tSteps = [10, 20, 50, 100, 200, 500, 1000];

// Thay thế sendCmd cũ thành điều khiển nội bộ Web
function sendCmd(type, val) {
    switch(type) {
        case 1: // TIME Scale
            let tIdx = tSteps.indexOf(tDiv);
            if(val > 0 && tIdx < tSteps.length - 1) tDiv = tSteps[tIdx + 1];
            if(val < 0 && tIdx > 0) tDiv = tSteps[tIdx - 1];
            // Tính toán lại hệ số zoom hiển thị dựa trên tDiv
            webTimeZoom = tDiv / 100.0; 
            break;
        case 2: // VOLT CH1
            let v1Idx = vSteps.indexOf(vDiv1);
            if(val > 0 && v1Idx < vSteps.length - 1) vDiv1 = vSteps[v1Idx + 1];
            if(val < 0 && v1Idx > 0) vDiv1 = vSteps[v1Idx - 1];
            break;
        case 8: // VOLT CH2
            let v2Idx = vSteps.indexOf(vDiv2);
            if(val > 0 && v2Idx < vSteps.length - 1) vDiv2 = vSteps[v2Idx + 1];
            if(val < 0 && v2Idx > 0) vDiv2 = vSteps[v2Idx - 1];
            break;
        case 6: // OFFSET (Dịch chuyển sóng trên web)
            // val > 0 thì dịch lên, val < 0 dịch xuống
            if(showCh1) ch1Offset += (val * 10);
            if(showCh2) ch2Offset += (val * 10);
            break;
        case 3: // HOLD
            hold = !hold;
            document.getElementById("holdBtn").classList.toggle("active");
            document.getElementById("runstate").innerText = hold ? "STOP" : "RUN";
            break;
    }
    updateLabels();
}

function updateLabels() {
    document.getElementById("voltLabel").innerText = vDiv1.toFixed(1) + "V/div";
    document.getElementById("voltLabel2").innerText = vDiv2.toFixed(1) + "V/div";
    document.getElementById("timeLabel").innerText = "Time: " + tDiv + "us/div";
    document.getElementById("offset1").innerText = ch1Offset;
    document.getElementById("offset2").innerText = ch2Offset;
}

function toggleCh(ch) {
    if(ch == 1) { showCh1 = !showCh1; document.getElementById("ch1btn").classList.toggle("active"); }
    if(ch == 2) { showCh2 = !showCh2; document.getElementById("ch2btn").classList.toggle("active"); }
}

function toggleCursor() {
    cursorMode = (cursorMode + 1) % 3;
    document.getElementById("cursorBtn").classList.toggle("active", cursorMode > 0);
}

// --- XỬ LÝ KÉO THẢ CURSOR BẰNG CHUỘT ---
canvas.onmousedown = (e) => {
    if(cursorMode == 0) return;
    const rect = canvas.getBoundingClientRect();
    const mx = (e.clientX - rect.left) / canvas.width;
    const my = (e.clientY - rect.top) / canvas.height;

    if(cursorMode == 1) { // Time
        if(Math.abs(mx - t1) < 0.05) dragging = "t1";
        else if(Math.abs(mx - t2) < 0.05) dragging = "t2";
    } else if(cursorMode == 2) { // Volt
        if(Math.abs(my - y1) < 0.05) dragging = "y1";
        else if(Math.abs(my - y2) < 0.05) dragging = "y2";
    }
};
window.onmouseup = () => { dragging = null; };
window.onmousemove = (e) => {
    if(!dragging) return;
    const rect = canvas.getBoundingClientRect();
    if(dragging == "t1") t1 = (e.clientX - rect.left) / canvas.width;
    if(dragging == "t2") t2 = (e.clientX - rect.left) / canvas.width;
    if(dragging == "y1") y1 = (e.clientY - rect.top) / canvas.height;
    if(dragging == "y2") y2 = (e.clientY - rect.top) / canvas.height;
};

function drawGrid() {
    ctx.strokeStyle = "#222"; 
    ctx.lineWidth = 1;
    for(let i = 0; i <= 10; i++) { 
        let x = i * canvas.width / 10; 
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke(); 
    }
    for(let i = 0; i <= 8; i++) { 
        let y = i * canvas.height / 8; 
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke(); 
    }
    // Trục giữa
    ctx.strokeStyle = "#444";
    ctx.beginPath(); ctx.moveTo(0, canvas.height/2); ctx.lineTo(canvas.width, canvas.height/2); ctx.stroke();
}

function drawVoltageAxis() {
    ctx.font = "11px Consolas";
    ctx.fillStyle = "#ffff00"; 
    for(let i = 0; i <= 8; i++) { 
        let y = i * canvas.height / 8; 
        let v = (4 - i) * vDiv1;
        ctx.fillText(v.toFixed(1) + "V", 5, y < 15 ? 15 : y - 2); 
    }
}

function drawWave(data, color, scale, offset) {
    if(!data || data.length == 0) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();

    // pixelsPerVolt: 1 ô (div) trên web tương đương canvas.height / 8
    let pixelsPerVolt = (canvas.height / 8) / scale;

    for(let i = 0; i < data.length; i++) {
        let x = (i / (data.length - 1)) * canvas.width;
        let vRaw = (data[i] / 4095.0) * 3.3; // Đưa về Volt thực tế
        
        // Vẽ sóng quanh trục giữa (height/2)
        let y = (canvas.height / 2) - (vRaw * pixelsPerVolt) + offset;

        if(i == 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }
    ctx.stroke();
}

function drawCursor() {
    if(cursorMode == 0) return;
    ctx.lineWidth = 1;
    ctx.setLineDash([5, 5]);

    if(cursorMode == 1) { // TIME MEASURE
        ctx.strokeStyle = "#ffffff";
        [t1, t2].forEach(t => {
            ctx.beginPath(); ctx.moveTo(t * canvas.width, 0); ctx.lineTo(t * canvas.width, canvas.height); ctx.stroke();
        });
        // Tính Delta T: Một khung hình ngang mặc định đại diện cho 10 ô (tDiv * 10)
        let deltaT = Math.abs(t1 - t2) * tDiv * 10;
        document.getElementById("dt").innerText = deltaT.toFixed(1);
    } 
    else if(cursorMode == 2) { // VOLT MEASURE
        ctx.strokeStyle = "#00ff00";
        [y1, y2].forEach(y => {
            ctx.beginPath(); ctx.moveTo(0, y * canvas.height); ctx.lineTo(canvas.width, y * canvas.height); ctx.stroke();
        });
        // Tính Delta V: Dựa trên khoảng cách pixel so với tỉ lệ Volt/div
        let deltaV = Math.abs(y1 - y2) * 8 * vDiv1; 
        document.getElementById("dv").innerText = deltaV.toFixed(3);
    }
    ctx.setLineDash([]);
}

async function fetchData() {
    if(hold) return;
    try {
        let res = await fetch('/wave');
        let data = await res.json();
        waveData = data.ch1;
        waveData2 = data.ch2;
        
        // Cập nhật text đo lường từ STM32 gửi sang
        document.getElementById("freq1").innerText = data.freq1.toFixed(1);
        document.getElementById("freq2").innerText = data.freq2.toFixed(1);
        document.getElementById("vpp1").innerText = data.vpp1.toFixed(2);
        document.getElementById("vpp2").innerText = data.vpp2.toFixed(2);
        document.getElementById("vavg1").innerText = data.vavg1.toFixed(2);
        document.getElementById("vavg2").innerText = data.vavg2.toFixed(2);
        document.getElementById("vrms1").innerText = data.vrms1.toFixed(2);
        document.getElementById("vrms2").innerText = data.vrms2.toFixed(2);
        
        // Hiển thị trạng thái kết nối
        document.getElementById("topbar").innerText = "CONNECTED | WEB SCALE ACTIVE";
    } catch(e) { 
        document.getElementById("topbar").innerText = "DISCONNECTED";
    }
}

function draw() {
    ctx.fillStyle = "black"; 
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    drawGrid(); 
    drawVoltageAxis(); 
    if(showCh1) drawWave(waveData, "#ffff00", vDiv1, ch1Offset);
    if(showCh2) drawWave(waveData2, "#00ffff", vDiv2, ch2Offset);
    drawCursor();
    requestAnimationFrame(draw);
}

function initCanvas() {
    canvas.width = canvas.clientWidth; 
    canvas.height = canvas.clientHeight;
}
window.onresize = initCanvas;
initCanvas();
setInterval(fetchData, 100);
draw();
</script>
</body>
</html>
)rawliteral";

// Hàm xử lý TIME scale (gửi lệnh xuống STM32)
void handleTimeScale(int delta) {
    cmd_to_send.cmd = 1;
    cmd_to_send.val = delta;
    has_new_cmd = true;
    Serial.printf("📤 TIME scale: %s\n", delta > 0 ? "+" : "-");
}

// Hàm xử lý VOLT CH1 scale (gửi xuống STM32)
void handleVoltageScaleCH1(int delta) {
    cmd_to_send.cmd = 2;
    cmd_to_send.val = delta;
    has_new_cmd = true;
    Serial.printf("📤 CH1 VOLT scale: %s\n", delta > 0 ? "+" : "-");
}

// Hàm xử lý VOLT CH2 scale (gửi xuống STM32)
void handleVoltageScaleCH2(int delta) {
    cmd_to_send.cmd = 8;
    cmd_to_send.val = delta;
    has_new_cmd = true;
    Serial.printf("📤 CH2 VOLT scale: %s\n", delta > 0 ? "+" : "-");
}

// Hàm xử lý OFFSET (gửi xuống STM32)
void handleOffset(int delta) {
    cmd_to_send.cmd = 6;
    cmd_to_send.val = delta;
    has_new_cmd = true;
    Serial.printf("📤 OFFSET: %s\n", delta > 0 ? "+" : "-");
}

// Hàm xử lý HOLD (gửi xuống STM32)
void handleHold() {
    cmd_to_send.cmd = 3;
    cmd_to_send.val = 0;
    has_new_cmd = true;
    Serial.println("📤 HOLD: Toggle");
}

// Hàm xử lý CHANNEL MODE (gửi xuống STM32)
void handleChannelMode(uint8_t ch) {
    cmd_to_send.cmd = 7;
    cmd_to_send.val = ch;
    has_new_cmd = true;
    Serial.printf("📤 CHANNEL: CH%d toggle\n", ch);
}

// ==========================================
// CẬP NHẬT handleControl
// ==========================================
void handleControl() {
    if(server.hasArg("cmd") && server.hasArg("val")) {
        uint8_t cmd = server.arg("cmd").toInt();
        int8_t val = server.arg("val").toInt();
        
        switch(cmd) {
            case 1:  // TIME scale
                handleTimeScale(val);
                break;
            case 2:  // VOLT CH1 scale
                handleVoltageScaleCH1(val);
                break;
            case 8:  // VOLT CH2 scale
                handleVoltageScaleCH2(val);
                break;
            case 6:  // OFFSET
                handleOffset(val);
                break;
            case 3:  // HOLD
                handleHold();
                break;
            case 7:  // CHANNEL MODE
                handleChannelMode(val);
                break;
            default:
                cmd_to_send.cmd = cmd;
                cmd_to_send.val = val;
                has_new_cmd = true;
                Serial.printf("📤 Command: cmd=%d, val=%d\n", cmd, val);
                break;
        }
        
        server.send(200, "text/plain", "OK");
    } else {
        server.send(400, "text/plain", "Missing parameters");
    }
}

void handleWave() {
    if (!data_valid) {
        server.send(503, "text/plain", "No data from STM32");
        return;
    }

    Packet_t local_data;
    portENTER_CRITICAL(&dataMutex);
    memcpy(&local_data, &stm_data, PACKET_SIZE);
    portEXIT_CRITICAL(&dataMutex);
    
    DynamicJsonDocument doc(16384);
    
    JsonArray ch1 = doc.createNestedArray("ch1");
    JsonArray ch2 = doc.createNestedArray("ch2");

    // Lấy gốc Trigger và hệ số trục X y hệt như LCD
    int base_trig1 = stm_data.trigger_idx1 + stm_data.x_offset1;
    int base_trig2 = stm_data.trigger_idx2 + stm_data.x_offset2;
    
    float xs1 = stm_data.x_scale1 <= 0.1f ? 1.0f : stm_data.x_scale1;
    float xs2 = stm_data.x_scale2 <= 0.1f ? 1.0f : stm_data.x_scale2;

    // Lấy đúng 500 điểm ảnh (tương đương với chiều rộng Canvas trên Web)
    for(int i = 0; i < 500; i++) {
        // Áp dụng công thức đồng bộ Trigger và Zoom X
        int idx1 = base_trig1 + (int)(i * xs1);
        int idx2 = base_trig2 + (int)(i * xs2);

        // Chống lố mảng (hết dữ liệu thì ghim ở mẫu cuối cùng)
        idx1 = constrain(idx1, 0, SAMPLES_PER_CH - 1);
        idx2 = constrain(idx2, 0, SAMPLES_PER_CH - 1);

        ch1.add(stm_data.ch1[idx1]);
        ch2.add(stm_data.ch2[idx2]);
    }
    
    // Gán thông số đo lường (Hiển thị text)
    doc["vpp1"]  = stm_data.vpp1;
    doc["vpp2"]  = stm_data.vpp2;
    doc["freq1"] = stm_data.freq1;
    doc["freq2"] = stm_data.freq2;
    doc["vavg1"] = stm_data.vavg1;
    doc["vavg2"] = stm_data.vavg2;
    doc["vrms1"] = stm_data.vrms1;
    doc["vrms2"] = stm_data.vrms2;

    // Gán thông số Đồ họa (BẮT BUỘC PHẢI CÓ ĐỂ VẼ SÓNG)
    doc["ch_mode"]   = stm_data.ch_mode;
    doc["hold_flag"] = stm_data.hold_flag;
    
    doc["y_scale1"]  = (stm_data.y_scale1 <= 0.1f) ? 1.0f : stm_data.y_scale1;
    doc["y_offset1"] = stm_data.y_offset1;
    
    doc["y_scale2"]  = (stm_data.y_scale2 <= 0.1f) ? 1.0f : stm_data.y_scale2;
    doc["y_offset2"] = stm_data.y_offset2;

    doc["x_scale1"]  = (stm_data.x_scale1 <= 0.1f) ? 1.0f : stm_data.x_scale1;
    doc["x_offset1"] = stm_data.x_offset1;    

    // Các thông số khác
    doc["offset_axis"]  = stm_data.offset_axis;
    doc["trigger_idx1"] = stm_data.trigger_idx1;
    doc["trigger_idx2"] = stm_data.trigger_idx2;
    doc["reset_flag"]   = stm_data.reset_flag;
    
    // Gửi scale cho web hiển thị (dùng scale từ STM32)
    doc["time_scale"]  = stm_data.x_scale1 * 100;  // Convert sang us/div
    doc["volt_scale1"] = stm_data.y_scale1;
    doc["volt_scale2"] = stm_data.y_scale2;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}
// ==========================================
// 4. HÀM CẬP NHẬT MÀN HÌNH LCD
// ==========================================
void drawStaticUI() {
    tft.fillScreen(TFT_BLACK);
    
    // --- TOP BAR (x:0->480, y:0->39) ---
    // Sẽ in thông số ở Top Bar
    
    // --- KHUNG ĐỒ THỊ (x:0->380, y:40->280) ---
    tft.drawRect(0, 39, 381, 242, TFT_WHITE); // Viền trắng
    // Lưới dọc
    for (int x = 40; x <= 380; x += 40) tft.drawFastVLine(x, 40, 240, 0x2104);
    // Lưới ngang
    for (int y = 40; y <= 280; y += 40) tft.drawFastHLine(0, y, 380, 0x2104);
    tft.drawFastHLine(0, 160, 380, TFT_DARKGREY); // Trục trung tâm (Y=160)

    // --- RIGHT PANEL (x:381->480, y:40->280) ---
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextFont(2);
    tft.setCursor(385, 45); tft.print(" DualCore ");
    tft.setCursor(385, 65); tft.print("  Scope   ");
    
    // --- BOTTOM BANNER (x:0->480, y:281->320) ---
    tft.drawFastHLine(0, 281, 480, TFT_DARKGREY);
}
void updateLCD() {
    static uint8_t last_reset_flag = 0;
    
    // Nếu có lệnh reset màn hình từ STM32
    if (stm_data.reset_flag == 1 && last_reset_flag == 0) {
        drawStaticUI(); 
        for (int x = 0; x < 385; x++) {
            prev_y1[x] = 0;
            prev_y2[x] = 0;
        }
    }
    last_reset_flag = stm_data.reset_flag;
    
    // 1. Lấy gốc Trigger và các hệ số
    int base_trigger1 = (int)stm_data.trigger_idx1;
    int base_trigger2 = (int)stm_data.trigger_idx2; 

    float scale1 = stm_data.y_scale1; if (scale1 <= 0.1f) scale1 = 1.0f;
    int16_t offset1 = stm_data.y_offset1;
    float scale2 = stm_data.y_scale2; if (scale2 <= 0.1f) scale2 = 1.0f;
    int16_t offset2 = stm_data.y_offset2;
    float x_scale1 = stm_data.x_scale1; if (x_scale1 <= 0.1f) x_scale1 = 1.0f;
    float x_scale2 = stm_data.x_scale2; if (x_scale2 <= 0.1f) x_scale2 = 1.0f;

    int draw_width = 379; 

    // VẼ SÓNG TRONG KHUNG 380x240
    for (int x = 0; x < draw_width; x++) {
        // Xóa nét cũ
        if (prev_y1[x] >= 40 && prev_y1[x+1] >= 40) tft.drawLine(x, prev_y1[x], x + 1, prev_y1[x+1], TFT_BLACK);
        if (prev_y2[x] >= 40 && prev_y2[x+1] >= 40) tft.drawLine(x, prev_y2[x], x + 1, prev_y2[x+1], TFT_BLACK);
        
        // Kẻ lại lưới dọc
        if (x % 40 == 0) tft.drawFastVLine(x, 40, 240, 0x2104);

        // --- TOÁN HỌC NỘI SUY TRỤC X ---
        int idx1_curr = base_trigger1 + stm_data.x_offset1 + (int)(x * x_scale1);
        int idx1_next = base_trigger1 + stm_data.x_offset1 + (int)((x + 1) * x_scale1);
        
        int idx2_curr = base_trigger2 + stm_data.x_offset2 + (int)(x * x_scale2);
        int idx2_next = base_trigger2 + stm_data.x_offset2 + (int)((x + 1) * x_scale2);

        // --- CHỐNG MẤT SÓNG ---
        // Ghim sóng lại ở giới hạn cuối SAMPLES_PER_CH - 1
        idx1_curr = constrain(idx1_curr, 0, SAMPLES_PER_CH - 1);
        idx1_next = constrain(idx1_next, 0, SAMPLES_PER_CH - 1);
        idx2_curr = constrain(idx2_curr, 0, SAMPLES_PER_CH - 1);
        idx2_next = constrain(idx2_next, 0, SAMPLES_PER_CH - 1);

        // --- VẼ KÊNH 1 (XANH LÁ) ---
        if (stm_data.ch_mode & 0x01) {
            int y1 = 160 + ((2048 - stm_data.ch1[idx1_curr]) * 120 / 2048.0) / scale1 + offset1;
            int y2 = 160 + ((2048 - stm_data.ch1[idx1_next]) * 120 / 2048.0) / scale1 + offset1;
            y1 = constrain(y1, 40, 280); y2 = constrain(y2, 40, 280);
            
            tft.drawLine(x, y1, x + 1, y2, TFT_GREEN);
            prev_y1[x] = y1; if (x == draw_width - 1) prev_y1[x+1] = y2;
        } else {
            prev_y1[x] = 0; if (x == draw_width - 1) prev_y1[x+1] = 0;
        }

        // --- VẼ KÊNH 2 (VÀNG) ---
        if (stm_data.ch_mode & 0x02) {
            int y1 = 160 + ((2048 - stm_data.ch2[idx2_curr]) * 120 / 2048.0) / scale2 + offset2;
            int y2 = 160 + ((2048 - stm_data.ch2[idx2_next]) * 120 / 2048.0) / scale2 + offset2;
            y1 = constrain(y1, 40, 280); y2 = constrain(y2, 40, 280);
            
            tft.drawLine(x, y1, x + 1, y2, TFT_YELLOW);
            prev_y2[x] = y1; if (x == draw_width - 1) prev_y2[x+1] = y2;
        } else {
            prev_y2[x] = 0; if (x == draw_width - 1) prev_y2[x+1] = 0;
        }
    }

    // Dọn rác sóng hụt ở đuôi
    for(int x = draw_width; x < 379; x++) {
        if (prev_y1[x] >= 40 && prev_y1[x+1] >= 40) tft.drawLine(x, prev_y1[x], x + 1, prev_y1[x+1], TFT_BLACK);
        if (prev_y2[x] >= 40 && prev_y2[x+1] >= 40) tft.drawLine(x, prev_y2[x], x + 1, prev_y2[x+1], TFT_BLACK);
        prev_y1[x] = 0; prev_y2[x] = 0;
        if (x % 40 == 0) tft.drawFastVLine(x, 40, 240, 0x2104);
    }
    prev_y1[379] = 0; prev_y2[379] = 0;

    // Phục hồi lưới ngang
    for (int gy = 40; gy <= 280; gy += 40) tft.drawFastHLine(0, gy, 380, 0x2104);
    tft.drawFastHLine(0, 160, 380, TFT_DARKGREY);

    // GIAO DIỆN CHỮ
    tft.setTextFont(2);
    tft.setCursor(5, 5);
    if (stm_data.ch_mode & 0x01) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.printf("CH1: %4.1fV %4.0fHz Avg:%4.1fV ", stm_data.vpp1, stm_data.freq1, stm_data.vavg1);
    } else {
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK); tft.print("CH1: OFF                         ");
    }
    
    tft.setCursor(5, 22);
    if (stm_data.ch_mode & 0x01) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.printf("Rms:%4.1fV Z:%.1f|%.1f X:%-4d ", stm_data.vrms1, stm_data.y_scale1, stm_data.x_scale1, stm_data.x_offset1);
    } else { 
        tft.setTextColor(TFT_BLACK, TFT_BLACK); tft.print("                                 "); 
    }

    tft.setCursor(240, 5);
    if (stm_data.ch_mode & 0x02) {
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.printf("CH2: %4.1fV %4.0fHz Avg:%4.1fV ", stm_data.vpp2, stm_data.freq2, stm_data.vavg2);
    } else {
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK); tft.print("CH2: OFF                         ");
    }
    
    tft.setCursor(240, 22);
    if (stm_data.ch_mode & 0x02) {
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.printf("Rms:%4.1fV Z:%.1f|%.1f X:%-4d ", stm_data.vrms2, stm_data.y_scale2, stm_data.x_scale2, stm_data.x_offset2);
    } else { 
        tft.setTextColor(TFT_BLACK, TFT_BLACK); tft.print("                                 "); 
    }

    tft.setCursor(385, 130);
    if (stm_data.hold_flag == 1) {
        tft.setTextColor(TFT_RED, TFT_BLACK); tft.print(" [STOP]  ");
    } else {
        tft.setTextColor(TFT_GREEN, TFT_BLACK); tft.print(" [RUN]   ");
    }
    tft.setCursor(385, 160);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    if (stm_data.offset_axis == 0) tft.print(" MODE: Y ");
    else tft.print(" MODE: X ");
}

// ==========================================
// 5. TASK CHẠY WEB SERVER (CORE 0)
// ==========================================
TaskHandle_t WebTask;

void webTaskCode(void * pvParameters) {
    for(;;) {
        server.handleClient();
        vTaskDelay(2 / portTICK_PERIOD_MS); // Nhường CPU để không bị crash Core
    }
}

// ==========================================
// 6. SETUP CHÍNH
// ==========================================
void setup() {
    Serial.begin(115200);
    
    // 6.1. Khởi tạo LCD
    tft.init();
    tft.setRotation(1); // Hoặc 3 (tùy theo hướng màn hình cũ)
    tft.fillScreen(TFT_BLACK);
    // 6.2. Khởi tạo SPI Slave (Copy logic cấu hình SPI từ WEB.txt)
    spi_bus_config_t buscfg;
    buscfg.mosi_io_num = GPIO_MOSI;
    buscfg.miso_io_num = GPIO_MISO;
    buscfg.sclk_io_num = GPIO_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = PACKET_SIZE + 100;
    
    spi_slave_interface_config_t slvcfg;
    slvcfg.spics_io_num = GPIO_CS;
    slvcfg.queue_size = 3;
    slvcfg.mode = 0;
    slvcfg.flags = 0;
    slvcfg.post_setup_cb = NULL;
    slvcfg.post_trans_cb = NULL;
    
    esp_err_t ret = spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if(ret != ESP_OK) {
        Serial.printf("SPI Slave init failed: %d\n", ret);
        return;
    }
    Serial.println("SPI Slave initialized (MODE 0, MSB First)");
    Serial.printf("Pins: MOSI=%d, MISO=%d, SCLK=%d, CS=%d\n", GPIO_MOSI, GPIO_MISO, GPIO_SCLK, GPIO_CS);
    
    // 6.3. Khởi tạo WiFi AP & WebServer (Copy từ WEB.txt)
    WiFi.softAP(ssid, password);
    Serial.printf("WiFi AP: %s | IP: %s\n", ssid, WiFi.softAPIP().toString().c_str());
    server.on("/", []() { server.send(200, "text/html", html); });
    server.on("/wave", handleWave);
    server.on("/control", handleControl);
    server.begin();
    Serial.println("Web server started\n");
    // 6.4. Đẩy WebServer sang chạy ở Core 0
    xTaskCreatePinnedToCore(
    webTaskCode,   // Con trỏ hàm thực thi luồng
    "WebTask",     // Tên luồng (dùng cho debug)
    40000,         // Dung lượng bộ nhớ Stack được cấp phát (Word)
    NULL,          // Tham số truyền vào hàm (Không có)
    1,             // Mức độ ưu tiên của luồng (Priority)
    &WebTask,      // Con trỏ quản lý luồng (Task Handle)
    0              // Lõi thực thi (Core 0)
);
}

// ==========================================
// 7. VÒNG LẶP CHÍNH CHẠY SPI & LCD (CORE 1)
// ==========================================
void loop() {
    // 1. Nếu có lệnh điều khiển từ Web, nạp vào buffer
    if (has_new_cmd) {
        memcpy(tx_buf, &cmd_to_send, sizeof(Command_t));
        has_new_cmd = false; 
    }

    // 2. Chuẩn bị giao dịch SPI
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = PACKET_SIZE * 8;      
    t.rx_buffer = rx_buf;            
    t.tx_buffer = tx_buf;            
    
    // Đọc SPI (chờ tối đa 50ms)
    esp_err_t ret = spi_slave_transmit(HSPI_HOST, &t, pdMS_TO_TICKS(50));

    // 3. Nếu đọc thành công
    if (ret == ESP_OK) {
        int offset = -1;
        // Quét tìm Header 0xAA 0xBB để chống lệch khung truyền
        for (int i = 0; i < 20; i++) {
            if (rx_buf[i] == 0xAA && rx_buf[i+1] == 0xBB) { 
                offset = i; 
                break; 
            }
        }
        
        if (offset != -1) {
            // Lấy dữ liệu
            portENTER_CRITICAL(&dataMutex);
            memcpy(&stm_data, &rx_buf[offset], PACKET_SIZE);
            portEXIT_CRITICAL(&dataMutex);
            
            // Kiểm tra Footer để đảm bảo nguyên vẹn
            if (stm_data.footer[0] == 0xCC && stm_data.footer[1] == 0xDD) {
                data_valid = true;
                last_data_time = millis();
                valid_packet_count++; 
                
                // ===== DỮ LIỆU CHUẨN: GỌI HÀM VẼ LCD =====
                updateLCD();
            }
        }
    }

    // 4. Debug định kỳ
    if (millis() - last_debug_time > 1000) {
        Serial.printf("Status: %s | Packets: %d | F1: %.1fHz | Vpp1: %.2fV\n", 
                      data_valid ? "CONNECTED" : "DISCONNECTED", 
                      valid_packet_count, stm_data.freq1, stm_data.vpp1);
        last_debug_time = millis();
        
        // Quá 2s không có tín hiệu -> Báo mất kết nối
        if (millis() - last_data_time > 2000) data_valid = false;
    }    
    vTaskDelay(1 / portTICK_PERIOD_MS); 
}