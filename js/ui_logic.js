const ctx = document.getElementById('chartCombined').getContext('2d');
combinedChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: [],
        datasets: [
            { label: 'Weight (g)', data: [], borderColor: '#0d6efd', backgroundColor: 'rgba(13, 110, 253, 0.05)', borderWidth: 2, yAxisID: 'y', fill: true, tension: 0.3, pointRadius: 0 },
            { label: 'Flow (g/s)', data: [], borderColor: '#ffc107', borderWidth: 2, borderDash: [5, 5], yAxisID: 'y1', tension: 0.1, pointRadius: 0 }
        ]
    },
    options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        scales: {
            x: { display: false },
            y: { type: 'linear', display: true, position: 'left', beginAtZero: true, title: {display: true, text: 'Weight (g)'} },
            y1: { type: 'linear', display: true, position: 'right', beginAtZero: true, grid: {drawOnChartArea: false}, title: {display: true, text: 'Flow Rate (g/s)'} }
        },
        plugins: { annotation: { annotations: { safeZone: { type: 'box', yMin: 0, yMax: 5, yScaleID: 'y1', backgroundColor: 'rgba(40, 167, 69, 0.1)', borderWidth: 0 } } } }
    }
});

function updateChartData(displayWeight, flowRate) {
    if (combinedChart.data.labels.length > 50) {
        combinedChart.data.labels.shift();
        combinedChart.data.datasets.forEach((dataset) => dataset.data.shift());
    }
    combinedChart.data.labels.push(new Date().toLocaleTimeString());
    
    combinedChart.data.datasets[0].data.push(displayWeight);
    combinedChart.data.datasets[1].data.push(flowRate);
    combinedChart.update('none');
}

function initSystem() {
    resetData();
    addLog("User Interface Initialized.");
    resetRobotInfo(); 
    offlineCheckInterval = setInterval(checkOfflineStatus, 1000);
    
    const progressBar = document.getElementById('progress-bar');
    if (progressBar) {
        progressBar.style.width = "0%";
        progressBar.innerText = "0%";
    }
}

function checkOfflineStatus() {
    if (Date.now() - lastUpdateTimestamp > 3000) updateConnectionStatus('offline');
}

function resetRobotInfo() {
    if(document.getElementById('val_vel')) document.getElementById('val_vel').innerText = "0.0";
    if(document.getElementById('val_acc')) document.getElementById('val_acc').innerText = "0.0";
    if(document.getElementById('pour_speed')) document.getElementById('pour_speed').innerText = "0.0";
}

function updateConnectionStatus(status, mode = "Unknown") {
    const light = document.getElementById('status-light');
    const text = document.getElementById('connection-text');
    const badge = document.getElementById('robot-mode-badge');

    light.className = "status-light"; 
    if(status === 'online') {
        light.classList.add('light-green');
        text.innerText = "System Online";
        text.className = "fw-bold text-success";
        badge.innerText = mode;
        badge.className = (mode === "Autonomous") ? "badge bg-success" : "badge bg-secondary";
    } else {
        light.classList.add('light-red');
        text.innerText = "System Offline";
        text.className = "fw-bold text-secondary";
        badge.innerText = "Unknown";
        badge.className = "badge bg-secondary";
        resetRobotInfo();
    }
}

function calculateTotalWeight() {
    const eth = parseFloat(document.getElementById('mat_ethanol').value) || 0;
    const ace = parseFloat(document.getElementById('mat_acetone').value) || 0;
    const water = parseFloat(document.getElementById('mat_water').value) || 0;
    
    document.getElementById('target_weight').value = eth + ace + water;
}

function updatePhaseDisplay(rawPhase) {
    if (!rawPhase) return;
    
    const newPhase = rawPhase.trim().charAt(0).toUpperCase() + rawPhase.trim().slice(1).toLowerCase();

    if(currentPhase !== newPhase) {
        if (newPhase === "Ready" || newPhase === "Stop" || newPhase === "Error" || newPhase === "Emergency") {
            pourCount = 0;
            hasMixed = false;
        }
        if (newPhase === "Pouring") pourCount++;

        currentPhase = newPhase;
        const badge = document.getElementById('phase-badge');
        
        if (newPhase === "Emergency" || newPhase === "Stop" || newPhase === "Error") {
            badge.innerText = "🚨 EMERGENCY";
        } else {
            badge.innerText = newPhase;
        }

        document.querySelectorAll('.step-item').forEach(el => el.classList.remove('step-active'));
        
        const stepId = "step-" + newPhase;
        const stepEl = document.getElementById(stepId);
        
        if(stepEl) {
            stepEl.classList.add('step-active');
            badge.className = "badge bg-primary phase-badge";
        } else if (newPhase === "Stop" || newPhase === "Error" || newPhase === "Emergency") {
            badge.className = "badge bg-danger phase-badge";
        } else {
            document.getElementById('step-Ready').classList.add('step-active');
            badge.className = "badge bg-secondary phase-badge";
        }

        let progressPct = 0;
        const totalPours = 3; 

        if (newPhase === "Ready" || newPhase === "Stop" || newPhase === "Error" || newPhase === "Emergency") {
            progressPct = 0; 
            hasMixed = false;
        } 
        else if (newPhase === "Transfer") {
            progressPct = (pourCount / totalPours) * 60 + 5; 
        } 
        else if (newPhase === "Pouring") {
            progressPct = (pourCount / totalPours) * 60 + 15; 
        } 
        else if (newPhase === "Mixing") {
            hasMixed = true; 
            progressPct = 85; 
        } 
        else if (newPhase === "Return") {
            if (hasMixed === true) {
                progressPct = 100; 
            } else {
                progressPct = (pourCount / totalPours) * 60 + 20; 
            }
        }
        
        if (progressPct > 100) progressPct = 100;

        const progressBar = document.getElementById('progress-bar');
        if(progressBar) {
            progressBar.style.width = progressPct + "%";
            
            if(newPhase === "Stop" || newPhase === "Error" || newPhase === "Emergency") {
                progressBar.classList.remove('bg-primary');
                progressBar.classList.add('bg-danger');
                progressBar.innerText = "HALTED"; 
            } else {
                progressBar.classList.remove('bg-danger');
                progressBar.classList.add('bg-primary');
                progressBar.innerText = Math.round(progressPct) + "%"; 
            }
        }

        // 💡 [실시간 타이머 정지 로직] 작업이 끝나면 초시계 스톱
        if(newPhase === "Ready" || newPhase === "Stop" || newPhase === "Emergency") {
            isRunning = false;
            if (liveTimerInterval) {
                clearInterval(liveTimerInterval);
                liveTimerInterval = null;
            }
        }
    }
}

function startExperiment() {
    const eth = document.getElementById('mat_ethanol').value;
    const ace = document.getElementById('mat_acetone').value;
    const water = document.getElementById('mat_water').value;
    const target = document.getElementById('target_weight').value;
    const mixing = document.getElementById('mixing_duration').value;
    
    const customMaterial = "Lab Recipe (에탄올" + eth + "/아세톤" + ace + "/물" + water + ")";
    
    if(confirm("[" + customMaterial + "]\n작업 시작?\n목표: " + target + "g / 혼합: " + mixing + "초")) {
        resetData();
        pourCount = 0; 
        hasMixed = false; 

        isRunning = true;
        startTime = Date.now();
        experimentData = []; 

        // 💡 [실시간 타이머 시작 로직] 0.1초 단위로 시간 측정
        if (liveTimerInterval) clearInterval(liveTimerInterval);
        document.getElementById('avg_cycle_time').innerText = "0.0";
        
        liveTimerInterval = setInterval(() => {
            if (isRunning) {
                const elapsed = (Date.now() - startTime) / 1000;
                const timeEl = document.getElementById('avg_cycle_time');
                if(timeEl) timeEl.innerText = elapsed.toFixed(1);
            } else {
                clearInterval(liveTimerInterval);
            }
        }, 100);

        if(database) {
            database.ref('commands').set({ 
                type: "start_pouring", 
                target_weight: target, 
                mixing_duration: mixing,
                material: customMaterial, 
                timestamp: Date.now() 
            });
        }
        addLog("🚀 Start: " + target + "g, Mix " + mixing + "s");
    }
}

function emergencyStop() { 
    if(database) {
        database.ref('commands').set({ type: "emergency_stop", timestamp: Date.now() }); 
        database.ref('system_stats/phase').set("Emergency"); 
    }
    addLog("🚨 STOP Signal Sent!"); 
}

function resetData() { 
    combinedChart.data.labels = []; 
    combinedChart.data.datasets.forEach(d => d.data = []); 
    combinedChart.update(); 
    document.getElementById('current_weight').innerText = "0.0"; 
    document.getElementById('current_flow').innerText = "0.0"; 
}

function addLog(msg) {
    const box = document.getElementById('log-box');
    if(box) {
        const time = new Date().toLocaleTimeString();
        box.innerHTML = "[" + time + "] " + msg + "<br>" + box.innerHTML;
    }
}

function exportToCSV() {
    if (experimentData.length === 0) { alert("저장할 데이터가 없습니다."); return; }
    let csvContent = "data:text/csv;charset=utf-8,Time(s),Weight(g),Flow(g/s)\r\n";
    experimentData.forEach(row => { csvContent += row.time + "," + row.weight + "," + row.flow.toFixed(2) + "\r\n"; });
    const encodedUri = encodeURI(csvContent);
    const link = document.createElement("a");
    link.setAttribute("href", encodedUri);
    link.setAttribute("download", "experiment_260220_v01.csv");
    document.body.appendChild(link);
    link.click();
}