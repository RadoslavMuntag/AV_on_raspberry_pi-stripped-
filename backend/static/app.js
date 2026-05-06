const stateEl = document.getElementById('state');
const clientIdEl = document.getElementById('clientId');
let ws;

function clientId() { return clientIdEl.value || 'web-local'; }

async function post(url, body) {
  const res = await fetch(url, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body)
  });
  if (!res.ok) {
    const text = await res.text();
    throw new Error(text);
  }
  return res.json();
}

async function acquire() {
  await post('/api/controller/acquire', {client_id: clientId()});
}

async function releaseCtrl() {
  await post('/api/controller/release', {client_id: clientId()});
}

async function setMode() {
  await post('/api/mode', {mode: document.getElementById('modeSel').value});
}

async function drive() {
  const left = Number(document.getElementById('left').value);
  const right = Number(document.getElementById('right').value);
  await fetch(`/api/control/drive?client_id=${encodeURIComponent(clientId())}`, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({left, right})
  });
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
    await post('/api/controller/heartbeat', {client_id: clientId()});
  } catch (_) {}
}

function connectWs() {
  ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws/telemetry`);
  ws.onmessage = (event) => {
    try {
      const payload = JSON.parse(event.data);
      stateEl.textContent = JSON.stringify(payload, null, 2);
    } catch {
      stateEl.textContent = event.data;
    }
  };
  ws.onclose = () => setTimeout(connectWs, 1000);
}

connectWs();
setInterval(heartbeat, 500);