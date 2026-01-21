#pragma once

/* HTML 页面内容 */
const char* index_html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="data:,">
    <title>ESP32-S3 Controller</title>
    <style>
        :root { --primary: #4361ee; --bg: #f8f9fa; --card: #ffffff; }
        body { font-family: 'Segoe UI', system-ui, sans-serif; background: var(--bg); margin: 0; display: flex; align-items: center; justify-content: center; min-height: 100vh; }
        .card { background: var(--card); padding: 2rem; border-radius: 1.5rem; box-shadow: 0 10px 25px rgba(0,0,0,0.05); width: 90%; max-width: 400px; text-align: center; }
        h1 { font-size: 1.5rem; margin-bottom: 0.5rem; color: #1a1a1a; }
        p { color: #666; margin-bottom: 2rem; font-size: 0.9rem; }
        .picker-container { position: relative; width: 100%; aspect-ratio: 1; margin-bottom: 2rem; }
        input[type=color] { appearance: none; -webkit-appearance: none; border: none; width: 100%; height: 100%; cursor: pointer; background: none; border-radius: 1rem; }
        input[type=color]::-webkit-color-swatch-wrapper { padding: 0; }
        input[type=color]::-webkit-color-swatch { border: 8px solid rgba(0,0,0,0.05); border-radius: 1rem; transition: transform 0.2s; }
        input[type=color]:hover::-webkit-color-swatch { transform: scale(1.02); }
        .btn { background: var(--primary); color: white; border: none; padding: 1rem 2rem; font-size: 1rem; font-weight: 600; border-radius: 0.75rem; cursor: pointer; width: 100%; transition: all 0.2s; box-shadow: 0 4px 15px rgba(67, 97, 238, 0.3); }
        .btn:active { transform: scale(0.98); box-shadow: 0 2px 8px rgba(67, 97, 238, 0.2); }
        #status { margin-top: 1.5rem; font-size: 0.85rem; padding: 0.75rem; border-radius: 0.5rem; display: none; }
        .success { display: block !important; background: #dcfce7; color: #166534; }
        .loading { display: block !important; background: #e0e7ff; color: #3730a3; }
    </style>
</head>
<body>
    <div class="card">
        <h1>RGB LED 控制</h1>
        <p>选择颜色并点击下方按钮同步至硬件</p>
        <div class="picker-container">
            <input type="color" id="picker" value="#4361ee">
        </div>
        <button class="btn" onclick="updateColor()">同步颜色</button>
        <div id="status"></div>
    </div>
    <script>
        function updateColor() {
            const color = document.getElementById('picker').value.substring(1);
            const status = document.getElementById('status');
            status.className = 'loading';
            status.innerText = '正在同步中...';
            
            fetch('/set_color?rgb=' + color)
                .then(r => {
                    if(r.ok) {
                        status.className = 'success';
                        status.innerText = '完成！当前颜色: #' + color.toUpperCase();
                        setTimeout(() => status.style.display = 'none', 3000);
                    } else { throw new Error(); }
                })
                .catch(e => {
                    status.className = '';
                    status.style.background = '#fee2e2';
                    status.style.color = '#991b1b';
                    status.style.display = 'block';
                    status.innerText = '同步失败，请重试';
                });
        }
    </script>
</body>
</html>
)rawliteral";
