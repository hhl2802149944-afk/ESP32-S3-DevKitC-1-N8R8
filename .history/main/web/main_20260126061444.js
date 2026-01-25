document.addEventListener('DOMContentLoaded', () => {
    // UI Elements
    const picker = document.getElementById('rgb-picker');
    const syncBtn = document.getElementById('sync-btn');
    const msgBar = document.getElementById('msg-bar');
    const speedSlider = document.getElementById('speed-slider');
    const speedVal = document.getElementById('speed-val');
    
    // RGB Value Labels
    const valR = document.getElementById('val-r');
    const valG = document.getElementById('val-g');
    const valB = document.getElementById('val-b');

    // Dashboard Elements
    const accDisplay = document.getElementById('accel-val');
    const gyroDisplay = document.getElementById('gyro-val');
    const gpsDisplay = document.getElementById('gps-val');
    const satDisplay = document.getElementById('gps-sats');
    const shutdownBtn = document.getElementById('shutdown-btn');

    // Helper: Send Command via HTTP Post
    async function sendCommand(cmd, successMsg) {
        msgBar.style.color = 'var(--text)';
        msgBar.innerText = 'Sending: ' + cmd + '...';
        
        try {
            const controller = new AbortController();
            const id = setTimeout(() => controller.abort(), 2000); // 2s timeout
            
            const response = await fetch('/api/cmd', {
                method: 'POST',
                headers: { 'Content-Type': 'text/plain' },
                body: cmd,
                signal: controller.signal
            });
            clearTimeout(id);

            if (response.ok) {
                if (successMsg) {
                    msgBar.style.color = 'var(--success)';
                    msgBar.innerText = successMsg;
                } else {
                    // msgBar.innerText = 'OK';
                }
                return true;
            } else {
                throw new Error(response.statusText);
            }
        } catch (error) {
            console.error('Command failed:', error);
            msgBar.style.color = 'var(--error)';
            msgBar.innerText = 'Error: ' + error.message;
            return false;
        }
    }

    // --- Control Event Listeners ---

    // 1. RGB Sync
    syncBtn.addEventListener('click', () => {
        const hex = picker.value.substring(1); // remove '#'
        sendCommand(`rgb${hex}`, `Color updated to #${hex.toUpperCase()}`);
    });

    // 2. RGB Slider UI update (local only)
    picker.addEventListener('input', (e) => {
        const hex = e.target.value.substring(1);
        valR.innerText = parseInt(hex.substring(0,2), 16);
        valG.innerText = parseInt(hex.substring(2,4), 16);
        valB.innerText = parseInt(hex.substring(4,6), 16);
    });

    // 3. Speed Slider
    speedSlider.addEventListener('input', (e) => {
        speedVal.innerText = e.target.value;
    });

    speedSlider.addEventListener('change', (e) => {
        const val = e.target.value;
        sendCommand(`spd${val}`, `Speed set to ${val}%`);
    });

    // 4. Global Motor Functions (called by HTML buttons)
    window.motorAction = (id, dir) => {
        const cmdMap = {
            '1': { 'forward': 'm1f', 'backward': 'm1b', 'stop': 'm1s' },
            '2': { 'forward': 'm2f', 'backward': 'm2b', 'stop': 'm2s' }
        };
        const cmd = cmdMap[id][dir];
        sendCommand(cmd); // Feedback will be "Sending..."
    };

    // 5. Shutdown
    shutdownBtn.addEventListener('click', () => {
        if (confirm('确定要让系统断电并进入深度睡眠吗？(按 BOOT 键唤醒)')) {
            sendCommand('off', 'System shutting down...');
        }
    });

    // --- Sensor Dashboard Polling ---
    
    async function updateDashboard() {
        try {
            const resp = await fetch('/api/sensor');
            if (!resp.ok) return;

            const data = await resp.json();
            
            // Expected JSON: { acc: [x,y,z], gyro: [x,y,z], gps: [lat, lon], sats: n }
            
            if (accDisplay && data.acc) {
                accDisplay.innerText = `A: ${data.acc[0].toFixed(2)}, ${data.acc[1].toFixed(2)}, ${data.acc[2].toFixed(2)}`;
            }
            
            if (gyroDisplay && data.gyro) {
                gyroDisplay.innerText = `G: ${data.gyro[0].toFixed(2)}, ${data.gyro[1].toFixed(2)}, ${data.gyro[2].toFixed(2)}`;
            }

            if (gpsDisplay && data.gps) {
                gpsDisplay.innerText = `${data.gps[0].toFixed(6)}, ${data.gps[1].toFixed(6)}`;
            }
            
            if (satDisplay && data.sats !== undefined) {
                satDisplay.innerText = `卫星数: ${data.sats}`;
            }

        } catch (e) {
            // console.debug('Polling skipped', e);
        }
    }

    // Start polling every 500ms
    setInterval(updateDashboard, 500);
});

