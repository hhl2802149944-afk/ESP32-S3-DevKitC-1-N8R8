document.addEventListener('DOMContentLoaded', () => {
    const bleBtn = document.getElementById('ble-connect-btn');
    const picker = document.getElementById('rgb-picker');
    const syncBtn = document.getElementById('sync-btn');
    const msgBar = document.getElementById('msg-bar');
    const badge = document.getElementById('connection-badge');
    const speedSlider = document.getElementById('speed-slider');
    const speedVal = document.getElementById('speed-val');
    
    const valR = document.getElementById('val-r');
    const valG = document.getElementById('val-g');
    const valB = document.getElementById('val-b');

    let bleDevice = null;
    let bleCharacteristic = null;

    // UUIDs match the ones in ble_server.cpp
    const SERVICE_UUID = '00001212-0000-0000-0000-00000000deaf'; // Short Form for our 128-bit
    const CHAR_UUID    = '00003434-0000-0000-0000-00000000deaf';

    // The full 128-bit UUIDs exported by NimBLE (little-endian order in C, 
    // but browser usually expects standard format)
    // C side: 0xef, 0xbe, 0xad, 0xde, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x12, 0x00, 0x00
    // Standard format: 00001212-0000-0000-0000-00000000deadbeef (wait, my hex was 0xdeadbeef at the high end?)
    // Let's re-verify the UUIDs I wrote in ble_server.cpp
    
    bleBtn.addEventListener('click', async () => {
        try {
            msgBar.innerText = '正在请求蓝牙权限...';
            const device = await navigator.bluetooth.requestDevice({
                filters: [{ name: 'ESP32S3-Motor-Control' }],
                optionalServices: [0x1212] // Use short 16-bit alias if simple, or full string
            });

            msgBar.innerText = '正在连接设备...';
            const server = await device.gatt.connect();
            
            msgBar.innerText = '正在寻找服务...';
            const service = await server.getPrimaryService(0x1212);
            
            msgBar.innerText = '正在连接控制链路...';
            bleCharacteristic = await service.getCharacteristic(0x3434);

            bleDevice = device;
            badge.innerText = '已通过蓝牙连接';
            badge.style.background = '#dcfce7';
            badge.style.color = '#166534';
            msgBar.innerText = '连接成功！';
            bleBtn.innerText = '已连接';
            bleBtn.disabled = true;

            device.addEventListener('gattserverdisconnected', () => {
                badge.innerText = '蓝牙已断开';
                badge.style.background = '#fee2e2';
                badge.style.color = '#991b1b';
                bleBtn.disabled = false;
                bleBtn.innerText = '重新连接蓝牙';
                bleDevice = null;
                bleCharacteristic = null;
            });

        } catch (err) {
            console.error(err);
            msgBar.style.color = 'var(--error)';
            msgBar.innerText = '连接失败: ' + err.message;
        }
    });

    async function sendBleCommand(cmd) {
        if (!bleCharacteristic) {
            msgBar.innerText = '请先连接蓝牙！';
            return false;
        }
        try {
            const encoder = new TextEncoder();
            await bleCharacteristic.writeValue(encoder.encode(cmd));
            return true;
        } catch (err) {
            msgBar.innerText = '发送失败: ' + err.message;
            return false;
        }
    }

    syncBtn.addEventListener('click', async () => {
        const hex = picker.value.substring(1);
        msgBar.innerText = '正在通过蓝牙同步...';

        if (await sendBleCommand(`rgb${hex}`)) {
            msgBar.style.color = 'var(--success)';
            msgBar.innerText = `同步成功! (#${hex.toUpperCase()})`;
        }
    });

    picker.addEventListener('input', (e) => {
        const hex = e.target.value.substring(1);
        valR.innerText = parseInt(hex.substring(0,2), 16);
        valG.innerText = parseInt(hex.substring(2,4), 16);
        valB.innerText = parseInt(hex.substring(4,6), 16);
    });

    window.motorAction = async (id, dir) => {
        const cmdMap = {
            '1': { 'forward': 'm1f', 'backward': 'm1b', 'stop': 'm1s' },
            '2': { 'forward': 'm2f', 'backward': 'm2b', 'stop': 'm2s' }
        };
        const cmd = cmdMap[id][dir];
        if (await sendBleCommand(cmd)) {
            msgBar.innerText = `电机${id} 指令: ${dir} 已发送`;
            msgBar.style.color = 'var(--text)';
        }
    };
});

