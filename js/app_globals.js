// 전역 변수들
let isRunning = false;
let startTime = 0;
let lastWeight = 0, lastTime = 0;
let experimentData = [];

let lastUpdateTimestamp = 0;
let offlineCheckInterval = null;
let currentPhase = "Ready";

let hasMixed = false; 
let pourCount = 0; 

let combinedChart = null; 
let database = null;      

// 💡 실시간 사이클 타임(스톱워치)을 위한 변수 추가
let liveTimerInterval = null;