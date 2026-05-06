const stateEl = document.getElementById('state');
const clientIdEl = document.getElementById('clientId');
const feedSel = document.getElementById('feedSel');
const video1El = document.getElementById('video1');
const video2El = document.getElementById('video2');
const toggleVideoBtn = document.getElementById('toggleVideoBtn');
const toggleTelemetryBtn = document.getElementById('toggleTelemetryBtn');
let ws;
let wsSeq = 0;
let currentMode = 'idle';
let telemetryEnabled = true;
let videoEnabled = true;
let toastHost;

let AUTO_ACQUIRE = true; // set to false to disable auto acquire on page load

function clientId() { return clientIdEl.value || 'web-local'; }

function ensureToastHost() {
  if (toastHost) return toastHost;
  toastHost = document.createElement('div');
  toastHost.className = 'toast-host';
  document.body.appendChild(toastHost);
  return toastHost;
}

function notify(message, kind = 'success') {
  const host = ensureToastHost();
  const toast = document.createElement('div');
  toast.className = `toast ${kind}`;
  toast.textContent = message;
  host.appendChild(toast);

  window.setTimeout(() => {
    toast.classList.add('hide');
    window.setTimeout(() => {
      if (toast.parentNode === host) host.removeChild(toast);
    }, 220);
  }, 1800);
}

function extractMessage(payload, fallback) {
  if (payload && typeof payload === 'object' && typeof payload.message === 'string' && payload.message) {
    return payload.message;
  }
  return fallback;
}

function extractErrorMessage(text, fallback) {
  try {
    const parsed = JSON.parse(text);
    if (parsed && typeof parsed.detail === 'string') {
      return parsed.detail;
    }
  } catch (_) {}
  return text || fallback;
}

async function post(url, body, options = {}) {
  const { notifyOnSuccess = true, successMessage = 'Request handled', errorMessage = 'Request failed' } = options;
  const res = await fetch(url, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body)
  });

  if (!res.ok) {
    const text = await res.text();
    const msg = extractErrorMessage(text, errorMessage);
    notify(msg, 'error');
    throw new Error(msg);
  }

  const payload = await res.json();
  if (notifyOnSuccess) {
    notify(extractMessage(payload, successMessage), 'success');
  }
  return payload;
}

async function acquire() {
  await post('/api/controller/acquire', {client_id: clientId()});
}

async function releaseCtrl() {
  await post('/api/controller/release', {client_id: clientId()});
}

async function setMode() {
  const mode = document.getElementById('modeSel').value;
  await post('/api/mode', { mode });
  currentMode = mode; // optimistic update
}

async function drive() {
  const left = Number(document.getElementById('left').value);
  const right = Number(document.getElementById('right').value);
  await post(
    `/api/control/drive?client_id=${encodeURIComponent(clientId())}`,
    {left, right},
    {successMessage: 'drive command applied'}
  );
}

async function connectDualSense() {
  await post('/api/controller/dualsense/connect', {client_id: clientId()});
}

async function reloadPipelineConfig() {
  await post('/api/config/pipeline/reload', {});
}

async function stopNow() {
  document.getElementById('left').value = 0;
  document.getElementById('right').value = 0;
  await drive();
  await setModeSafeStop();
}

async function setModeSafeStop() {
  await post('/api/mode', {mode: 'safe_stop'});
}

async function heartbeat() {
  try {
    await post('/api/controller/heartbeat', {client_id: clientId()}, {notifyOnSuccess: false});
  } catch (_) {}
}


function wsBaseUrl() {
  return `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}`;
}

function currentWsPath() {
  const frame = feedSel?.value || 'telemetry';
  if (frame === 'telemetry') return '/ws/telemetry';
  return `/ws/pipeline?frame=${encodeURIComponent(frame)}`;
}

function connectWs() {
  if (!telemetryEnabled) {
    return;
  }

  wsSeq += 1;
  const seq = wsSeq;

  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.onclose = null; // avoid reconnect from intentional close
    ws.close();
  }

  ws = new WebSocket(`${wsBaseUrl()}${currentWsPath()}`);

  ws.onmessage = (event) => {
    try {
      const payload = JSON.parse(event.data);
      stateEl.textContent = JSON.stringify(payload, null, 2);
    } catch {
      stateEl.textContent = event.data;
    }
  };

  ws.onclose = () => {
    setTimeout(() => {
      if (seq === wsSeq && telemetryEnabled) connectWs();
    }, 1000);
  };
}

function closeWs() {
  wsSeq += 1;
  if (ws) {
    ws.onclose = null;
    ws.close();
    ws = null;
  }
}

function toggleTelemetryStream() {
  telemetryEnabled = !telemetryEnabled;

  if (telemetryEnabled) {
    if (toggleTelemetryBtn) toggleTelemetryBtn.textContent = 'Disable State Stream';
    stateEl.textContent = 'state stream enabled...';
    connectWs();
  } else {
    if (toggleTelemetryBtn) toggleTelemetryBtn.textContent = 'Enable State Stream';
    closeWs();
    stateEl.textContent = 'state stream disabled';
  }
}

function toggleVideoStreams() {
  videoEnabled = !videoEnabled;

  if (videoEnabled) {
    if (video1El) video1El.src = '/video/mjpeg';
    if (video2El) video2El.src = '/video/mjpeg2';
    if (toggleVideoBtn) toggleVideoBtn.textContent = 'Disable Video Streams';
  } else {
    if (video1El) video1El.removeAttribute('src');
    if (video2El) video2El.removeAttribute('src');
    if (toggleVideoBtn) toggleVideoBtn.textContent = 'Enable Video Streams';
  }
}

if (feedSel) {
  feedSel.addEventListener('change', connectWs);
}

connectWs();
setInterval(heartbeat, 500);

if (AUTO_ACQUIRE) {
  acquire().catch((err) => {
    console.error('Failed to auto acquire controller on page load:', err);
  });
}