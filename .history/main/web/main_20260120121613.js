document.addEventListener('DOMContentLoaded', () => {
    const picker = document.getElementById('rgb-picker');
    const syncBtn = document.getElementById('sync-btn');
    const msgBar = document.getElementById('msg-bar');
    const badge = document.getElementById('connection-badge');
    
    const valR = document.getElementById('val-r');
    const valG = document.getElementById('val-g');
    const valB = document.getElementById('val-b');

    // 模拟连接状态检测
    setTimeout(() => {
        badge.innerText = '在线 (ESP32-S3)';
        badge.style.background = '#dcfce7';
        badge.style.color = '#166534';
    }, 1000);

    syncBtn.addEventListener('click', async () => {
        const hex = picker.value.substring(1);
        syncBtn.classList.add('loading');
        msgBar.innerText = '正在同步数据...';

        try {
            const response = await fetch(`/api/set_color?rgb=${hex}`);
            if (response.ok) {
                // 更新显示数值
                const r = parseInt(hex.substring(0,2), 16);
                const g = parseInt(hex.substring(2,4), 16);
                const b = parseInt(hex.substring(4,6), 16);
                valR.innerText = r;
                valG.innerText = g;
                valB.innerText = b;

                msgBar.style.color = 'var(--success)';
                msgBar.innerText = `同步成功! (#${hex.toUpperCase()})`;
            } else {
                throw new Error('Server responded with error');
            }
        } catch (err) {
            msgBar.style.color = 'var(--error)';
            msgBar.innerText = '错误: 无法连接到 ESP32';
        } finally {
            syncBtn.classList.remove('loading');
        }
    });

    // 实时更新数值显示 (视觉效果)
    picker.addEventListener('input', (e) => {
        const hex = e.target.value.substring(1);
        valR.innerText = parseInt(hex.substring(0,2), 16);
        valG.innerText = parseInt(hex.substring(2,4), 16);
        valB.innerText = parseInt(hex.substring(4,6), 16);
    });

    // 电机控制逻辑
    window.motorAction = async (id, dir) => {
        msgBar.innerText = `电机${id} 指令: ${dir}...`;
        try {
            await fetch(`/api/motor?id=${id}&dir=${dir}`);
            msgBar.innerText = `电机${id} 执行成功`;
            msgBar.style.color = 'var(--text)';
        } catch (err) {
            msgBar.innerText = '发送失败';
            msgBar.style.color = 'var(--error)';
        }
    };
});

