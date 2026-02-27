const firebaseConfig = { apiKey: "AIzaSyBOVQ2tEPWe_GfIBavzqcw1ppfe2EqiRdE", authDomain: "colab1-78afc.firebaseapp.com", databaseURL: "https://colab1-78afc-default-rtdb.asia-southeast1.firebasedatabase.app", projectId: "colab1-78afc", storageBucket: "colab1-78afc.firebasestorage.app", messagingSenderId: "312617327218", appId: "1:312617327218:web:2564e4ed07449d381dc0eb" };
if (!firebase.apps.length) firebase.initializeApp(firebaseConfig);
database = firebase.database();

database.ref('sensor_data').on('value', (snapshot) => {
    const d = snapshot.val();
    if(!d) return;

    lastUpdateTimestamp = Date.now();
    updateConnectionStatus('online', 'Autonomous');

    let displayWeight = parseFloat(d.weight || 0);
    let flowRate = 0.0;
    const currentTime = Date.now();

    if (lastTime > 0) {
        const dt = (currentTime - lastTime) / 1000;
        if (dt > 0) flowRate = Math.max(0, (displayWeight - lastWeight) / dt);
    }

    document.getElementById('current_weight').innerText = displayWeight.toFixed(1);
    document.getElementById('current_flow').innerText = flowRate.toFixed(1);

    if (isRunning) {
        let elapsedTime = ((currentTime - startTime) / 1000).toFixed(1);
        experimentData.push({
            time: elapsedTime,
            weight: displayWeight.toFixed(2),
            flow: flowRate
        });
    }

    if (combinedChart.data.labels.length > 50) {
        combinedChart.data.labels.shift();
        combinedChart.data.datasets.forEach((dataset) => dataset.data.shift());
    }
    combinedChart.data.labels.push(new Date().toLocaleTimeString());
    
    combinedChart.data.datasets[0].data.push(displayWeight);
    combinedChart.data.datasets[1].data.push(flowRate);
    combinedChart.update('none');

    lastWeight = displayWeight;
    lastTime = currentTime;
});

database.ref('system_stats').on('value', (snapshot) => {
    const stats = snapshot.val();
    if(!stats) return;

    if(stats.tcp_vel !== undefined) document.getElementById('val_vel').innerText = parseFloat(stats.tcp_vel).toFixed(1);
    if(stats.tcp_acc !== undefined) document.getElementById('val_acc').innerText = parseFloat(stats.tcp_acc).toFixed(1);
    if(stats.pour_speed !== undefined) document.getElementById('pour_speed').innerText = parseFloat(stats.pour_speed).toFixed(1);
    
    if(stats.max_tilt_step !== undefined) document.getElementById('val_max_step').innerText = parseFloat(stats.max_tilt_step).toFixed(1);
    if(stats.stop_threshold !== undefined) document.getElementById('val_stop_th').innerText = parseFloat(stats.stop_threshold).toFixed(1);

    if(stats.phase) updatePhaseDisplay(stats.phase);
});

database.ref('robot_status').on('value', (snapshot) => {
    const data = snapshot.val();
    if (!data) return;

    if(data.current_angle !== undefined) { 
        const angleElement = document.getElementById('robot_angle');
        const cupElement = document.getElementById('cup-icon');
        
        if (angleElement) angleElement.innerText = data.current_angle; 
        if (cupElement) cupElement.style.transform = "rotate(" + data.current_angle + "deg)"; 
    }
});

database.ref('experiment_history').on('value', (snapshot) => {
    const tbody = document.getElementById('history-tbody');
    if (!snapshot.exists()) {
        tbody.innerHTML = '<tr><td colspan="6" class="text-muted py-2">실험 데이터가 없습니다.</td></tr>';
        document.getElementById('total_count').innerText = "0";
        document.getElementById('success_count').innerText = "0";
        document.getElementById('success_rate').innerText = "0";
        document.getElementById('avg_error_rate').innerText = "0.0";
        document.getElementById('avg_cycle_time').innerText = "0.0";
        return;
    }

    tbody.innerHTML = ''; 
    let records = [];
    let calcTotalCount = 0;
    let calcSuccessCount = 0;
    let sumErrorRate = 0.0;

    snapshot.forEach((child) => {
        const data = child.val();
        records.push(data);
        
        calcTotalCount++;
        if (data.success === true) calcSuccessCount++; 
        sumErrorRate += parseFloat(data.error_rate) || 0.0;
    });
    
    document.getElementById('total_count').innerText = calcTotalCount;
    document.getElementById('success_count').innerText = calcSuccessCount;
    document.getElementById('success_rate').innerText = calcTotalCount > 0 ? ((calcSuccessCount / calcTotalCount) * 100).toFixed(0) : 0;
    document.getElementById('avg_error_rate').innerText = calcTotalCount > 0 ? (sumErrorRate / calcTotalCount).toFixed(2) : "0.0";
    
    // 💡 [실시간 타이머 방어 로직] 작업이 멈춰있을 때만(타이머가 안 돌때만) 최신 기록의 시간을 표시
    if (!isRunning) {
        let lastCycle = records.length > 0 ? records[records.length - 1].cycle_time : "0.0";
        document.getElementById('avg_cycle_time').innerText = lastCycle;
    }
    
    for(let i = records.length - 1; i >= 0; i--) {
        const data = records[i];
        const tr = document.createElement('tr');
        
        let successBadge = data.is_emergency === true ? '<span class="badge" style="background-color: #8b0000; color: white;">긴급중단</span>' : (data.success ? '<span class="badge bg-success">성공</span>' : '<span class="badge bg-danger">실패</span>');
        const materialStr = data.material !== undefined ? data.material : '-';
        const ssError = data.ss_error_g !== undefined ? data.ss_error_g : '-';
        const cycle = data.cycle_time !== undefined ? data.cycle_time : '-';
        const displayErrorRate = data.error_rate !== undefined ? data.error_rate : "0.0";
        const errorClass = data.success === true ? 'text-success' : 'text-danger';

        tr.innerHTML = 
            "<td class='fw-bold'>" + (i + 1) + "</td>" +
            "<td>" + data.target_weight + "g<br><small class='text-muted'>" + materialStr + "</small></td>" +
            "<td class='text-primary fw-bold'>" + data.final_weight + "g<br><small class='text-danger'>(±" + ssError + "g)</small></td>" +
            "<td class='" + errorClass + " fw-bold'>" + displayErrorRate + "%</td>" +
            "<td>" + cycle + "s</td>" +
            "<td>" + successBadge + "</td>";
        
        tbody.appendChild(tr);
    }
});