var API_BASE = 'http://' + window.location.hostname + ':8080';
var WS_BASE = 'ws://' + window.location.hostname + ':8080';

var appState = {
  connected: false,
  agvStatus: {
    mode: '--',
    battery: 0,
    speed: 0,
    status: '--',
    position: { x: 0, y: 0, theta: 0 },
    velocity: { linear_x: 0, angular_z: 0 },
    uptime: '--',
    cpu: 0,
    memory: 0,
    temperature: 0
  },
  plcData: { connected: false, device: '--', ip: '--', port: '--', registers: [] },
  ioStates: { digital: [], analog: [] },
  yoloResult: { detections: [], image: '' },
  wifiStatus: { status: '--', ssid: '--', ip: '--', signal: '--', networks: [] },
  bluetoothDevices: [],
  waypoints: [],
  path: [],
  ros2Nodes: []
};

var wsConnections = {};
var pollIntervals = [];

function apiGet(path) {
  return fetch(API_BASE + path, { method: 'GET', headers: { 'Accept': 'application/json' } })
    .then(function(res) {
      if (!res.ok) throw new Error('HTTP ' + res.status);
      return res.json();
    })
    .catch(function(err) {
      showToast('API Error: ' + err.message, 'error');
      throw err;
    });
}

function apiPost(path, body) {
  return fetch(API_BASE + path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json', 'Accept': 'application/json' },
    body: JSON.stringify(body)
  })
    .then(function(res) {
      if (!res.ok) throw new Error('HTTP ' + res.status);
      return res.json();
    })
    .catch(function(err) {
      showToast('API Error: ' + err.message, 'error');
      throw err;
    });
}

function connectWebSocket(path, onMessage) {
  var url = WS_BASE + path;
  var ws = new WebSocket(url);
  wsConnections[path] = ws;

  ws.onopen = function() {
    setConnected(true);
  };

  ws.onmessage = function(event) {
    try {
      var data = JSON.parse(event.data);
      onMessage(data);
    } catch (e) {
      console.error('WS parse error:', e);
    }
  };

  ws.onclose = function() {
    setConnected(false);
    setTimeout(function() {
      connectWebSocket(path, onMessage);
    }, 3000);
  };

  ws.onerror = function() {
    ws.close();
  };

  return ws;
}

function setConnected(val) {
  appState.connected = val;
  var dot = document.getElementById('connDot');
  var text = document.getElementById('connText');
  var sidebarDot = document.querySelector('#sidebarConnStatus .status-dot');
  var sidebarText = document.querySelector('#sidebarConnStatus span:last-child');

  if (val) {
    dot.className = 'status-dot connected';
    text.textContent = 'Connected';
    sidebarDot.className = 'status-dot connected';
    sidebarText.textContent = 'Online';
  } else {
    dot.className = 'status-dot disconnected';
    text.textContent = 'Disconnected';
    sidebarDot.className = 'status-dot disconnected';
    sidebarText.textContent = 'Offline';
  }
}

function initNavigation() {
  var navItems = document.querySelectorAll('.nav-item');
  navItems.forEach(function(item) {
    item.addEventListener('click', function(e) {
      e.preventDefault();
      var page = this.getAttribute('data-page');
      navItems.forEach(function(n) { n.classList.remove('active'); });
      this.classList.add('active');
      document.querySelectorAll('.page').forEach(function(p) { p.classList.remove('active'); });
      var target = document.getElementById('page-' + page);
      if (target) target.classList.add('active');
    });
  });

  document.getElementById('sidebarToggle').addEventListener('click', function() {
    var sidebar = document.getElementById('sidebar');
    if (window.innerWidth <= 700) {
      sidebar.classList.toggle('mobile-open');
    } else {
      sidebar.classList.toggle('collapsed');
    }
  });
}

function updateDashboard(status) {
  if (!status) return;
  appState.agvStatus = Object.assign(appState.agvStatus, status);

  var s = appState.agvStatus;
  document.getElementById('dashMode').textContent = s.mode || '--';
  document.getElementById('dashBattery').textContent = (s.battery || 0) + '%';
  document.getElementById('dashSpeed').textContent = (s.speed || 0).toFixed(2) + ' m/s';
  document.getElementById('dashStatus').textContent = s.status || '--';

  var batteryFill = document.getElementById('dashBatteryFill');
  batteryFill.style.width = (s.battery || 0) + '%';
  batteryFill.className = 'battery-fill';
  if (s.battery < 20) batteryFill.classList.add('low');
  else if (s.battery < 50) batteryFill.classList.add('medium');

  document.getElementById('dashPosX').textContent = (s.position.x || 0).toFixed(3);
  document.getElementById('dashPosY').textContent = (s.position.y || 0).toFixed(3);
  document.getElementById('dashPosTheta').textContent = (s.position.theta || 0).toFixed(3);

  document.getElementById('dashVelLinear').textContent = (s.velocity.linear_x || 0).toFixed(3) + ' m/s';
  document.getElementById('dashVelAngular').textContent = (s.velocity.angular_z || 0).toFixed(3) + ' rad/s';

  document.getElementById('dashUptime').textContent = s.uptime || '--';
  document.getElementById('dashCpu').textContent = (s.cpu || 0) + '%';
  document.getElementById('dashMemory').textContent = (s.memory || 0) + '%';
  document.getElementById('dashTemp').textContent = (s.temperature || 0) + '\u00B0C';

  document.getElementById('dashTimestamp').textContent = new Date().toLocaleTimeString();
}

function addLog(message, type) {
  var logList = document.getElementById('dashLog');
  var entry = document.createElement('div');
  entry.className = 'log-entry' + (type ? ' log-' + type : '');
  var time = new Date().toLocaleTimeString();
  entry.innerHTML = '<span class="log-time">' + time + '</span>' + message;
  logList.insertBefore(entry, logList.firstChild);
  if (logList.children.length > 100) {
    logList.removeChild(logList.lastChild);
  }
}

function sendCmdVel(linear_x, angular_z) {
  apiPost('/api/v1/agv/control', { command: 'cmd_vel', linear_x: linear_x, angular_z: angular_z })
    .then(function() {
      addLog('Cmd vel: lx=' + linear_x + ' az=' + angular_z, 'success');
    })
    .catch(function() {
      addLog('Failed to send cmd vel', 'error');
    });
}

function sendCmdVelFromSliders() {
  var lx = parseFloat(document.getElementById('velLinearX').value);
  var az = parseFloat(document.getElementById('velAngularZ').value);
  sendCmdVel(lx, az);
}

function navigateTo(x, y, theta) {
  var tx = x !== undefined ? x : parseFloat(document.getElementById('navTargetX').value);
  var ty = y !== undefined ? y : parseFloat(document.getElementById('navTargetY').value);
  var tt = theta !== undefined ? theta : parseFloat(document.getElementById('navTargetTheta').value);

  apiPost('/api/v1/agv/navigate', { x: tx, y: ty, theta: tt })
    .then(function() {
      showToast('Navigation started to (' + tx + ', ' + ty + ')', 'success');
      addLog('Navigate to: (' + tx + ', ' + ty + ', ' + tt + ')', 'success');
    })
    .catch(function() {
      addLog('Navigation failed', 'error');
    });
}

function startPatrol(waypoints, loops) {
  var wp = waypoints || appState.waypoints;
  var lp = loops || parseInt(document.getElementById('patrolLoops').value) || 1;

  if (wp.length === 0) {
    showToast('No waypoints defined', 'warning');
    return;
  }

  apiPost('/api/v1/agv/patrol', { waypoints: wp, loops: lp })
    .then(function() {
      showToast('Patrol started (' + wp.length + ' waypoints, ' + lp + ' loops)', 'success');
      addLog('Patrol started', 'success');
    })
    .catch(function() {
      addLog('Patrol start failed', 'error');
    });
}

function clearWaypoints() {
  appState.waypoints = [];
  renderWaypoints();
  drawMap();
}

function addWaypoint(x, y) {
  appState.waypoints.push({ x: x, y: y, theta: 0 });
  renderWaypoints();
  drawMap();
}

function removeWaypoint(index) {
  appState.waypoints.splice(index, 1);
  renderWaypoints();
  drawMap();
}

function renderWaypoints() {
  var list = document.getElementById('waypointList');
  list.innerHTML = '';
  appState.waypoints.forEach(function(wp, i) {
    var item = document.createElement('div');
    item.className = 'waypoint-item';
    item.innerHTML = '<span>WP' + (i + 1) + ': (' + wp.x.toFixed(2) + ', ' + wp.y.toFixed(2) + ')</span>' +
      '<button class="wp-remove" onclick="removeWaypoint(' + i + ')">\u00D7</button>';
    list.appendChild(item);
  });
}

var mapCanvas = document.getElementById('mapCanvas');
var mapCtx = mapCanvas.getContext('2d');
var mapScale = 50;
var mapOffsetX = 0;
var mapOffsetY = 0;

function initMap() {
  resizeMapCanvas();

  mapCanvas.addEventListener('click', function(e) {
    var rect = mapCanvas.getBoundingClientRect();
    var scaleX = mapCanvas.width / rect.width;
    var scaleY = mapCanvas.height / rect.height;
    var cx = (e.clientX - rect.left) * scaleX;
    var cy = (e.clientY - rect.top) * scaleY;
    var worldX = (cx - mapCanvas.width / 2) / mapScale;
    var worldY = -(cy - mapCanvas.height / 2) / mapScale;
    addWaypoint(Math.round(worldX * 100) / 100, Math.round(worldY * 100) / 100);
  });

  drawMap();
}

function resizeMapCanvas() {
  var rect = mapCanvas.parentElement.getBoundingClientRect();
  mapCanvas.width = rect.width - 30;
  mapCanvas.height = 400;
}

function drawMap() {
  var ctx = mapCtx;
  var w = mapCanvas.width;
  var h = mapCanvas.height;

  ctx.fillStyle = '#0a0e17';
  ctx.fillRect(0, 0, w, h);

  ctx.strokeStyle = '#1a2332';
  ctx.lineWidth = 1;
  for (var gx = -10; gx <= 10; gx++) {
    var sx = w / 2 + gx * mapScale;
    ctx.beginPath();
    ctx.moveTo(sx, 0);
    ctx.lineTo(sx, h);
    ctx.stroke();
  }
  for (var gy = -10; gy <= 10; gy++) {
    var sy = h / 2 + gy * mapScale;
    ctx.beginPath();
    ctx.moveTo(0, sy);
    ctx.lineTo(w, sy);
    ctx.stroke();
  }

  ctx.strokeStyle = '#2a3a4e';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(w / 2, 0);
  ctx.lineTo(w / 2, h);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(0, h / 2);
  ctx.lineTo(w, h / 2);
  ctx.stroke();

  drawPath(appState.path);
  drawWaypoints(appState.waypoints);
  drawAGVPosition(appState.agvStatus.position.x, appState.agvStatus.position.y, appState.agvStatus.position.theta);
}

function drawAGVPosition(x, y, theta) {
  var ctx = mapCtx;
  var w = mapCanvas.width;
  var h = mapCanvas.height;
  var sx = w / 2 + x * mapScale;
  var sy = h / 2 - y * mapScale;

  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-theta);

  ctx.fillStyle = '#06b6d4';
  ctx.shadowColor = 'rgba(6, 182, 212, 0.6)';
  ctx.shadowBlur = 12;
  ctx.beginPath();
  ctx.moveTo(0, -12);
  ctx.lineTo(-8, 8);
  ctx.lineTo(8, 8);
  ctx.closePath();
  ctx.fill();

  ctx.shadowBlur = 0;
  ctx.strokeStyle = '#06b6d4';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(0, 0, 16, 0, Math.PI * 2);
  ctx.stroke();

  ctx.restore();
}

function drawWaypoints(waypoints) {
  var ctx = mapCtx;
  var w = mapCanvas.width;
  var h = mapCanvas.height;

  waypoints.forEach(function(wp, i) {
    var sx = w / 2 + wp.x * mapScale;
    var sy = h / 2 - wp.y * mapScale;

    ctx.fillStyle = '#f59e0b';
    ctx.shadowColor = 'rgba(245, 158, 11, 0.5)';
    ctx.shadowBlur = 8;
    ctx.beginPath();
    ctx.arc(sx, sy, 6, 0, Math.PI * 2);
    ctx.fill();

    ctx.shadowBlur = 0;
    ctx.fillStyle = '#f59e0b';
    ctx.font = '10px monospace';
    ctx.fillText('WP' + (i + 1), sx + 8, sy - 4);
  });
}

function drawPath(path) {
  if (!path || path.length < 2) return;
  var ctx = mapCtx;
  var w = mapCanvas.width;
  var h = mapCanvas.height;

  ctx.strokeStyle = 'rgba(6, 182, 212, 0.4)';
  ctx.lineWidth = 2;
  ctx.setLineDash([6, 4]);
  ctx.beginPath();
  path.forEach(function(p, i) {
    var sx = w / 2 + p.x * mapScale;
    var sy = h / 2 - p.y * mapScale;
    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  });
  ctx.stroke();
  ctx.setLineDash([]);
}

function readPlc(device, address, quantity) {
  var dev = device || document.getElementById('plcReadDevice').value;
  var addr = address !== undefined ? address : parseInt(document.getElementById('plcReadAddr').value);
  var qty = quantity || parseInt(document.getElementById('plcReadQty').value);
  var type = document.getElementById('plcReadType').value;

  apiGet('/api/v1/plc/' + dev + '/read?type=' + type + '&address=' + addr + '&quantity=' + qty)
    .then(function(data) {
      updatePlcTable(data);
      showToast('PLC read successful', 'success');
    })
    .catch(function() {
      addLog('PLC read failed', 'error');
    });
}

function writePlc(device, address, values) {
  var dev = device || document.getElementById('plcWriteDevice').value;
  var addr = address !== undefined ? address : parseInt(document.getElementById('plcWriteAddr').value);
  var vals = values || document.getElementById('plcWriteValues').value.split(',').map(function(v) { return v.trim(); });
  var type = document.getElementById('plcWriteType').value;

  apiPost('/api/v1/plc/' + dev + '/write', { type: type, address: addr, values: vals })
    .then(function() {
      showToast('PLC write successful', 'success');
      addLog('PLC write: ' + dev + ' addr=' + addr, 'success');
    })
    .catch(function() {
      addLog('PLC write failed', 'error');
    });
}

function refreshPlcStatus() {
  apiGet('/api/v1/plc/status')
    .then(function(data) {
      appState.plcData = Object.assign(appState.plcData, data);
      document.getElementById('plcConnStatus').textContent = data.connected ? 'Connected' : 'Disconnected';
      document.getElementById('plcConnStatus').style.color = data.connected ? 'var(--success)' : 'var(--danger)';
      document.getElementById('plcDevice').textContent = data.device || '--';
      document.getElementById('plcIP').textContent = data.ip || '--';
      document.getElementById('plcPort').textContent = data.port || '--';
    })
    .catch(function() {});
}

function updatePlcTable(data) {
  var body = document.getElementById('plcDataBody');
  body.innerHTML = '';
  if (!data || !data.values) return;
  var ts = new Date().toLocaleTimeString();
  data.values.forEach(function(val, i) {
    var row = document.createElement('tr');
    row.innerHTML = '<td>' + (data.address + i) + '</td><td>' + (data.type || '--') + '</td><td>' + val + '</td><td>' + ts + '</td>';
    body.appendChild(row);
  });
}

function updateIOStates(data) {
  if (!data) return;
  appState.ioStates = Object.assign(appState.ioStates, data);
  renderIOList();
}

function renderIOList() {
  var digitalList = document.getElementById('digitalIoList');
  var analogList = document.getElementById('analogIoList');
  digitalList.innerHTML = '';
  analogList.innerHTML = '';

  (appState.ioStates.digital || []).forEach(function(io) {
    var item = document.createElement('div');
    item.className = 'io-item';
    var checked = io.value ? 'checked' : '';
    item.innerHTML = '<div class="io-info"><span class="io-name">' + io.name + '</span><span class="io-detail">Pin ' + io.pin + ' | ' + io.direction + '</span></div>' +
      '<label class="toggle-switch"><input type="checkbox" ' + checked + ' onchange="setIO(\'' + io.name + '\',\'digital\',' + io.pin + ',this.checked?1:0)"><span class="toggle-slider"></span></label>';
    digitalList.appendChild(item);
  });

  (appState.ioStates.analog || []).forEach(function(io) {
    var item = document.createElement('div');
    item.className = 'io-item';
    item.innerHTML = '<div class="io-info"><span class="io-name">' + io.name + '</span><span class="io-detail">Pin ' + io.pin + ' | ' + io.direction + '</span></div>' +
      '<span class="io-value">' + (io.value !== undefined ? io.value.toFixed(3) : '--') + '</span>';
    analogList.appendChild(item);
  });

  if ((appState.ioStates.digital || []).length === 0) {
    digitalList.innerHTML = '<div class="empty-state">No digital IO data</div>';
  }
  if ((appState.ioStates.analog || []).length === 0) {
    analogList.innerHTML = '<div class="empty-state">No analog IO data</div>';
  }
}

function setIO(ioName, ioType, pin, value) {
  apiPost('/api/v1/io/set', { name: ioName, type: ioType, pin: pin, value: value })
    .then(function() {
      showToast(ioName + ' set to ' + value, 'success');
    })
    .catch(function() {});
}

function updateYoloResult(data) {
  if (!data) return;
  appState.yoloResult = Object.assign(appState.yoloResult, data);

  if (data.image) {
    document.getElementById('cameraImg').src = 'data:image/jpeg;base64,' + data.image;
  }

  var list = document.getElementById('detectionList');
  list.innerHTML = '';

  if (!data.detections || data.detections.length === 0) {
    list.innerHTML = '<div class="empty-state">No detections</div>';
    return;
  }

  var colors = ['#06b6d4', '#10b981', '#f59e0b', '#ef4444', '#8b5cf6', '#ec4899'];
  data.detections.forEach(function(det, i) {
    var color = colors[i % colors.length];
    var item = document.createElement('div');
    item.className = 'detection-item';
    item.innerHTML = '<div class="detection-color" style="background:' + color + '"></div>' +
      '<div class="detection-info"><div class="detection-class">' + det.class_name + '</div>' +
      '<div class="detection-conf">Confidence: ' + (det.confidence * 100).toFixed(1) + '%</div>' +
      '<div class="detection-bbox">BBox: [' + det.bbox.join(', ') + ']</div></div>';
    list.appendChild(item);
  });
}

function getDetections() {
  apiGet('/api/v1/vision/detections')
    .then(function(data) {
      updateYoloResult(data);
    })
    .catch(function() {});
}

function startTraining(config) {
  var cfg = config || {
    model: document.getElementById('trainModel').value,
    epochs: parseInt(document.getElementById('trainEpochs').value),
    batch_size: parseInt(document.getElementById('trainBatch').value),
    img_size: parseInt(document.getElementById('trainImgSize').value),
    dataset: document.getElementById('trainDataset').value
  };

  apiPost('/api/v1/vision/train', cfg)
    .then(function() {
      showToast('Training started', 'success');
      document.getElementById('trainingProgress').style.display = 'block';
      addLog('Vision training started: ' + cfg.model, 'success');
    })
    .catch(function() {
      addLog('Training start failed', 'error');
    });
}

function scanWiFi() {
  apiGet('/api/v1/wifi/scan')
    .then(function(data) {
      appState.wifiStatus.networks = data.networks || [];
      renderWiFiList(data.networks);
      showToast('WiFi scan complete: ' + (data.networks || []).length + ' networks', 'info');
    })
    .catch(function() {});
}

function renderWiFiList(networks) {
  var list = document.getElementById('wifiList');
  list.innerHTML = '';
  if (!networks || networks.length === 0) {
    list.innerHTML = '<div class="empty-state">No networks found</div>';
    return;
  }
  networks.forEach(function(net) {
    var item = document.createElement('div');
    item.className = 'wifi-item';
    item.innerHTML = '<span class="wifi-ssid">' + net.ssid + '</span><span class="wifi-signal">' + (net.signal || '--') + ' dBm</span>';
    item.addEventListener('click', function() {
      document.getElementById('wifiConnectForm').style.display = 'block';
      document.getElementById('wifiConnectSSID').value = net.ssid;
    });
    list.appendChild(item);
  });
}

function connectWiFi(ssid, password) {
  apiPost('/api/v1/wifi/connect', { ssid: ssid, password: password })
    .then(function() {
      showToast('WiFi connecting to ' + ssid, 'success');
      addLog('WiFi connect: ' + ssid, 'success');
    })
    .catch(function() {
      addLog('WiFi connect failed', 'error');
    });
}

function connectWiFiFromForm() {
  var ssid = document.getElementById('wifiConnectSSID').value;
  var pass = document.getElementById('wifiConnectPass').value;
  connectWiFi(ssid, pass);
}

function scanBluetooth() {
  apiGet('/api/v1/bluetooth/scan')
    .then(function(data) {
      appState.bluetoothDevices = data.devices || [];
      renderBluetoothList(data.devices);
      showToast('Bluetooth scan complete: ' + (data.devices || []).length + ' devices', 'info');
    })
    .catch(function() {});
}

function renderBluetoothList(devices) {
  var list = document.getElementById('btList');
  list.innerHTML = '';
  if (!devices || devices.length === 0) {
    list.innerHTML = '<div class="empty-state">No devices found</div>';
    return;
  }
  devices.forEach(function(dev) {
    var item = document.createElement('div');
    item.className = 'bt-item';
    item.innerHTML = '<span class="bt-name">' + dev.name + '</span><span class="bt-address">' + dev.address + '</span>';
    item.addEventListener('click', function() {
      connectBluetooth(dev.address);
    });
    list.appendChild(item);
  });
}

function connectBluetooth(address, profile) {
  apiPost('/api/v1/bluetooth/connect', { address: address, profile: profile || '' })
    .then(function() {
      showToast('Bluetooth connecting to ' + address, 'success');
      addLog('BT connect: ' + address, 'success');
    })
    .catch(function() {
      addLog('BT connect failed', 'error');
    });
}

function emergencyStop() {
  showModal('Emergency Stop', 'Are you sure you want to trigger an emergency stop?', function() {
    apiPost('/api/v1/agv/control', { command: 'emergency_stop' })
      .then(function() {
        showToast('EMERGENCY STOP ACTIVATED', 'error');
        addLog('EMERGENCY STOP', 'error');
      })
      .catch(function() {
        addLog('Emergency stop failed', 'error');
      });
  });
}

function saveSettings() {
  var settings = {
    agv_name: document.getElementById('settingAgvName').value,
    max_speed: parseFloat(document.getElementById('settingMaxSpeed').value),
    safety_distance: parseFloat(document.getElementById('settingSafetyDist').value),
    map_frame: document.getElementById('settingMapFrame').value,
    robot_frame: document.getElementById('settingRobotFrame').value
  };

  apiPost('/api/v1/settings', settings)
    .then(function() {
      showToast('Settings saved', 'success');
      document.getElementById('agvNameDisplay').textContent = settings.agv_name;
      addLog('Settings saved', 'success');
    })
    .catch(function() {
      addLog('Settings save failed', 'error');
    });
}

function refreshRos2Nodes() {
  apiGet('/api/v1/ros2/nodes')
    .then(function(data) {
      appState.ros2Nodes = data.nodes || [];
      renderRos2Nodes(data.nodes);
    })
    .catch(function() {});
}

function renderRos2Nodes(nodes) {
  var list = document.getElementById('ros2NodeList');
  list.innerHTML = '';
  if (!nodes || nodes.length === 0) {
    list.innerHTML = '<div class="empty-state">No nodes found</div>';
    return;
  }
  nodes.forEach(function(node) {
    var item = document.createElement('div');
    item.className = 'node-item';
    var statusClass = node.status === 'running' ? 'running' : 'stopped';
    item.innerHTML = '<span class="node-name">' + node.name + '</span><span class="node-status ' + statusClass + '">' + node.status + '</span>';
    list.appendChild(item);
  });
}

function launchRos2Node() {
  var pkg = document.getElementById('ros2LaunchPkg').value;
  var file = document.getElementById('ros2LaunchFile').value;
  if (!pkg || !file) {
    showToast('Package and launch file required', 'warning');
    return;
  }
  apiPost('/api/v1/ros2/launch', { package: pkg, launch_file: file })
    .then(function() {
      showToast('Launch command sent: ' + pkg + ' ' + file, 'success');
      addLog('ROS2 launch: ' + pkg + ' ' + file, 'success');
    })
    .catch(function() {
      addLog('ROS2 launch failed', 'error');
    });
}

function showToast(message, type) {
  var container = document.getElementById('toastContainer');
  var toast = document.createElement('div');
  toast.className = 'toast toast-' + (type || 'info');
  toast.textContent = message;
  container.appendChild(toast);

  setTimeout(function() {
    toast.classList.add('toast-out');
    setTimeout(function() {
      if (toast.parentNode) toast.parentNode.removeChild(toast);
    }, 300);
  }, 4000);
}

function showModal(title, body, onConfirm) {
  var modal = document.getElementById('modal');
  document.getElementById('modalTitle').textContent = title;
  document.getElementById('modalBody').textContent = body;
  modal.classList.add('open');

  var confirmBtn = document.getElementById('modalConfirmBtn');
  var newBtn = confirmBtn.cloneNode(true);
  confirmBtn.parentNode.replaceChild(newBtn, confirmBtn);
  newBtn.id = 'modalConfirmBtn';
  newBtn.addEventListener('click', function() {
    closeModal();
    if (onConfirm) onConfirm();
  });
}

function closeModal() {
  document.getElementById('modal').classList.remove('open');
}

function initSliders() {
  var lxSlider = document.getElementById('velLinearX');
  var azSlider = document.getElementById('velAngularZ');
  var lxVal = document.getElementById('velLinearXVal');
  var azVal = document.getElementById('velAngularZVal');

  lxSlider.addEventListener('input', function() {
    lxVal.textContent = parseFloat(this.value).toFixed(2);
  });
  azSlider.addEventListener('input', function() {
    azVal.textContent = parseFloat(this.value).toFixed(2);
  });
}

function initWebSockets() {
  connectWebSocket('/ws/agv_status', function(data) {
    updateDashboard(data);
  });

  connectWebSocket('/ws/yolo_result', function(data) {
    updateYoloResult(data);
  });

  connectWebSocket('/ws/io_states', function(data) {
    updateIOStates(data);
  });

  connectWebSocket('/ws/plc_data', function(data) {
    appState.plcData = Object.assign(appState.plcData, data);
    if (data.registers) updatePlcTable(data.registers);
  });
}

function startPolling() {
  pollIntervals.push(setInterval(function() {
    apiGet('/api/v1/agv/status')
      .then(function(data) {
        updateDashboard(data);
        setConnected(true);
      })
      .catch(function() {
        setConnected(false);
      });
  }, 2000));

  pollIntervals.push(setInterval(function() {
    apiGet('/api/v1/plc/status')
      .then(function(data) {
        appState.plcData = Object.assign(appState.plcData, data);
        document.getElementById('plcConnStatus').textContent = data.connected ? 'Connected' : 'Disconnected';
        document.getElementById('plcConnStatus').style.color = data.connected ? 'var(--success)' : 'var(--danger)';
        document.getElementById('plcDevice').textContent = data.device || '--';
        document.getElementById('plcIP').textContent = data.ip || '--';
        document.getElementById('plcPort').textContent = data.port || '--';
      })
      .catch(function() {});
  }, 5000));

  pollIntervals.push(setInterval(function() {
    apiGet('/api/v1/wifi/status')
      .then(function(data) {
        appState.wifiStatus = Object.assign(appState.wifiStatus, data);
        document.getElementById('wifiStatus').textContent = data.status || '--';
        document.getElementById('wifiSSID').textContent = data.ssid || '--';
        document.getElementById('wifiIP').textContent = data.ip || '--';
        document.getElementById('wifiSignal').textContent = data.signal || '--';
      })
      .catch(function() {});
  }, 5000));

  pollIntervals.push(setInterval(function() {
    apiGet('/api/v1/bluetooth/status')
      .then(function(data) {
        document.getElementById('btStatus').textContent = data.status || '--';
        document.getElementById('btConnected').textContent = data.connected_device || '--';
      })
      .catch(function() {});
  }, 5000));

  pollIntervals.push(setInterval(function() {
    apiGet('/api/v1/io/states')
      .then(function(data) {
        updateIOStates(data);
      })
      .catch(function() {});
  }, 3000));

  pollIntervals.push(setInterval(function() {
    apiGet('/api/v1/ros2/nodes')
      .then(function(data) {
        appState.ros2Nodes = data.nodes || [];
        renderRos2Nodes(data.nodes);
      })
      .catch(function() {});
  }, 10000));

  pollIntervals.push(setInterval(function() {
    drawMap();
  }, 1000));
}

function initPowerManagement() {
  appState.batteryState = {
    voltage: 0, current: 0, charge_level: 0, temperature: 0,
    health_percent: 0, charging_state: 'unknown', charge_rate: 0,
    discharge_rate: 0, estimated_time_remaining: 0, charge_cycles: 0,
    battery_type: '--'
  };
  appState.powerMode = 'balanced';
  appState.powerHistory = [];

  function updatePowerPage(data) {
    if (!data) return;
    var el = function(id) { return document.getElementById(id); };
    if (el('powerChargeLevel')) el('powerChargeLevel').textContent = (data.charge_level || 0).toFixed(1);
    if (el('powerVoltage')) el('powerVoltage').textContent = (data.voltage || 0).toFixed(1);
    if (el('powerCurrent')) el('powerCurrent').textContent = (data.current || 0).toFixed(2);
    if (el('powerTimeRemaining')) el('powerTimeRemaining').textContent = ((data.estimated_time_remaining || 0) / 60).toFixed(0);
    if (el('powerChargingState')) el('powerChargingState').textContent = data.charging_state || '--';
    if (el('powerTemperature')) el('powerTemperature').textContent = (data.temperature || 0).toFixed(1) + ' °C';
    if (el('powerHealth')) el('powerHealth').textContent = (data.health_percent || 0).toFixed(0) + ' %';
    if (el('powerChargeCycles')) el('powerChargeCycles').textContent = data.charge_cycles || 0;
    if (el('powerBatteryType')) el('powerBatteryType').textContent = data.battery_type || '--';
    if (el('powerChargeRate')) el('powerChargeRate').textContent = (data.charge_rate || 0).toFixed(2) + ' A';
    if (el('powerDischargeRate')) el('powerDischargeRate').textContent = (data.discharge_rate || 0).toFixed(2) + ' A';
    if (el('powerBatteryFill')) {
      var pct = Math.max(0, Math.min(100, data.charge_level || 0));
      el('powerBatteryFill').style.width = pct + '%';
      el('powerBatteryFill').className = 'battery-fill' + (pct < 20 ? ' low' : pct < 50 ? ' medium' : '');
    }
    if (el('powerMode')) el('powerMode').textContent = appState.powerMode || '--';
    if (el('powerAutoCharge')) el('powerAutoCharge').textContent = '启用';
    appState.powerHistory.push({ time: Date.now(), level: data.charge_level || 0, voltage: data.voltage || 0 });
    if (appState.powerHistory.length > 120) appState.powerHistory.shift();
    drawPowerChart();
  }

  function drawPowerChart() {
    var canvas = document.getElementById('powerChartCanvas');
    if (!canvas) return;
    var ctx = canvas.getContext('2d');
    var w = canvas.width = canvas.offsetWidth;
    var h = canvas.height = canvas.offsetHeight;
    ctx.clearRect(0, 0, w, h);
    var hist = appState.powerHistory;
    if (hist.length < 2) return;
    var maxLevel = 100;
    ctx.strokeStyle = '#00d4aa';
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (var i = 0; i < hist.length; i++) {
      var x = (i / (hist.length - 1)) * w;
      var y = h - (hist[i].level / maxLevel) * h;
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.lineTo(w, h);
    ctx.lineTo(0, h);
    ctx.closePath();
    var grad = ctx.createLinearGradient(0, 0, 0, h);
    grad.addColorStop(0, 'rgba(0, 212, 170, 0.3)');
    grad.addColorStop(1, 'rgba(0, 212, 170, 0.0)');
    ctx.fillStyle = grad;
    ctx.fill();
    ctx.strokeStyle = '#00a8cc';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (var i = 0; i < hist.length; i++) {
      var x = (i / (hist.length - 1)) * w;
      var y = h - ((hist[i].voltage - 39) / (54.6 - 39)) * h;
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }

  function setPowerMode(mode) {
    apiPost('/api/v1/power/mode', { model_path: mode }).then(function() {
      appState.powerMode = mode;
      showToast('电源模式已切换: ' + mode, 'success');
    }).catch(function() {});
  }

  function setCharging(command) {
    apiPost('/api/v1/power/charging', { command: command }).then(function(res) {
      showToast('充电命令: ' + command + (res.success ? ' 成功' : ' 失败'), res.success ? 'success' : 'error');
    }).catch(function() {});
  }

  document.getElementById('setModePerformance').addEventListener('click', function() { setPowerMode('performance'); });
  document.getElementById('setModeBalanced').addEventListener('click', function() { setPowerMode('balanced'); });
  document.getElementById('setModePowerSave').addEventListener('click', function() { setPowerMode('power_save'); });
  document.getElementById('startCharging').addEventListener('click', function() { setCharging('start_charging'); });
  document.getElementById('stopCharging').addEventListener('click', function() { setCharging('stop_charging'); });
  document.getElementById('dockToCharger').addEventListener('click', function() { setCharging('force_dock'); });
  document.getElementById('refreshPower').addEventListener('click', function() {
    apiGet('/api/v1/power/status').then(function(data) { updatePowerPage(data); }).catch(function() {});
  });

  pollIntervals.push(setInterval(function() {
    apiGet('/api/v1/power/status').then(function(data) {
      if (data) {
        appState.batteryState = data;
        updatePowerPage(data);
      }
    }).catch(function() {});
  }, 3000));
}

function init3dScanner() {
  appState.pointCloud = {
    points: [],
    xMin: Infinity,
    xMax: -Infinity,
    yMin: Infinity,
    yMax: -Infinity,
    zMin: Infinity,
    zMax: -Infinity,
    frameCount: 0,
    isScanning: false,
    startTime: null,
    rotation: { yaw: 0, pitch: 0 },
    zoom: 1
  };

  function updateScanPage() {
    var pc = appState.pointCloud;
    document.getElementById('pointCount').textContent = pc.points.length;
    document.getElementById('frameCount').textContent = pc.frameCount;
    
    if (pc.startTime && pc.isScanning) {
      var elapsed = Math.floor((Date.now() - pc.startTime) / 1000);
      var mins = Math.floor(elapsed / 60).toString().padStart(2, '0');
      var secs = (elapsed % 60).toString().padStart(2, '0');
      document.getElementById('scanDuration').textContent = mins + ':' + secs;
    }

    if (pc.points.length > 0) {
      var fmt = function(v) { return v.toFixed(3); };
      document.getElementById('xRange').textContent = fmt(pc.xMin) + ' ~ ' + fmt(pc.xMax);
      document.getElementById('yRange').textContent = fmt(pc.yMin) + ' ~ ' + fmt(pc.yMax);
      document.getElementById('zRange').textContent = fmt(pc.zMin) + ' ~ ' + fmt(pc.zMax);
      
      var vol = (pc.xMax - pc.xMin) * (pc.yMax - pc.yMin) * (pc.zMax - pc.zMin);
      var density = vol > 0 ? (pc.points.length / vol).toFixed(1) : '0';
      document.getElementById('density').textContent = density + ' pts/m³';
      
      var memoryMB = (pc.points.length * 3 * 4 / (1024 * 1024)).toFixed(2);
      document.getElementById('memoryUsage').textContent = memoryMB + ' MB';
      
      document.getElementById('pointCloudEmpty').style.display = 'none';
    } else {
      document.getElementById('pointCloudEmpty').style.display = 'block';
    }
  }

  function renderPointCloud() {
    var canvas = document.getElementById('pointCloudCanvas');
    var ctx = canvas.getContext('2d');
    var pc = appState.pointCloud;
    
    var w = canvas.width = canvas.offsetWidth;
    var h = canvas.height = canvas.offsetHeight;
    ctx.clearRect(0, 0, w, h);
    
    if (pc.points.length === 0) return;
    
    var centerX = (pc.xMin + pc.xMax) / 2;
    var centerY = (pc.yMin + pc.yMax) / 2;
    var centerZ = (pc.zMin + pc.zMax) / 2;
    
    var scaleX = (w * 0.4) / Math.max(1, Math.max(pc.xMax - centerX, centerX - pc.xMin));
    var scaleY = (h * 0.4) / Math.max(1, Math.max(pc.yMax - centerY, centerY - pc.yMin));
    var scale = Math.min(scaleX, scaleY) * pc.zoom;
    
    var yaw = pc.rotation.yaw;
    var pitch = pc.rotation.pitch;
    var cosY = Math.cos(yaw), sinY = Math.sin(yaw);
    var cosP = Math.cos(pitch), sinP = Math.sin(pitch);
    
    for (var i = 0; i < pc.points.length; i++) {
      var p = pc.points[i];
      var x = p.x - centerX;
      var y = p.y - centerY;
      var z = p.z - centerZ;
      
      var x1 = x * cosY - y * sinY;
      var y1 = x * sinY + y * cosY;
      var y2 = y1 * cosP - z * sinP;
      var z2 = y1 * sinP + z * cosP;
      
      var px = w/2 + x1 * scale;
      var py = h/2 - y2 * scale;
      
      var depth = (z2 - pc.zMin) / Math.max(1, pc.zMax - pc.zMin);
      var r = Math.floor(100 + depth * 155);
      var g = Math.floor(200 - depth * 100);
      var b = Math.floor(100 + depth * 100);
      
      ctx.fillStyle = 'rgb(' + r + ',' + g + ',' + b + ')';
      ctx.beginPath();
      ctx.arc(px, py, 1.5, 0, Math.PI * 2);
      ctx.fill();
    }
    
    ctx.fillStyle = 'rgba(0, 212, 170, 0.8)';
    ctx.beginPath();
    ctx.arc(w/2, h/2, 5, 0, Math.PI * 2);
    ctx.fill();
  }

  function addPoints(data) {
    var pc = appState.pointCloud;
    if (!data.points_x || data.points_x.length === 0) return;
    
    for (var i = 0; i < data.points_x.length; i++) {
      var x = data.points_x[i];
      var y = data.points_y[i];
      var z = data.points_z[i];
      pc.points.push({x: x, y: y, z: z});
      pc.xMin = Math.min(pc.xMin, x);
      pc.xMax = Math.max(pc.xMax, x);
      pc.yMin = Math.min(pc.yMin, y);
      pc.yMax = Math.max(pc.yMax, y);
      pc.zMin = Math.min(pc.zMin, z);
      pc.zMax = Math.max(pc.zMax, z);
    }
    pc.frameCount++;
    updateScanPage();
  }

  document.getElementById('startScanning').addEventListener('click', function() {
    apiPost('/api/v1/scan3d/start', {
      scan_pattern: document.getElementById('scanPattern').value,
      scan_resolution: parseFloat(document.getElementById('scanResolution').value),
      max_points: parseInt(document.getElementById('maxPoints').value)
    }).then(function(res) {
      if (res.success) {
        appState.pointCloud.isScanning = true;
        appState.pointCloud.startTime = Date.now();
        document.getElementById('scanStatus').textContent = '扫描中';
        document.getElementById('scanStatusChange').textContent = '运行中';
        showToast('开始3D扫描', 'success');
      }
    }).catch(function() {});
  });

  document.getElementById('stopScanning').addEventListener('click', function() {
    apiPost('/api/v1/scan3d/stop', {}).then(function() {
      appState.pointCloud.isScanning = false;
      document.getElementById('scanStatus').textContent = '已停止';
      document.getElementById('scanStatusChange').textContent = '就绪';
      showToast('停止3D扫描', 'info');
    }).catch(function() {});
  });

  document.getElementById('generateMap').addEventListener('click', function() {
    apiPost('/api/v1/scan3d/generate_map', {
      map_name: document.getElementById('mapName').value,
      export_path: '/tmp',
      format: document.getElementById('exportFormat').value,
      include_path: true
    }).then(function(res) {
      if (res.success) {
        showToast('扫描图已生成: ' + res.output_file, 'success');
      }
    }).catch(function() {});
  });

  document.getElementById('clearPoints').addEventListener('click', function() {
    appState.pointCloud.points = [];
    appState.pointCloud.xMin = Infinity;
    appState.pointCloud.xMax = -Infinity;
    appState.pointCloud.yMin = Infinity;
    appState.pointCloud.yMax = -Infinity;
    appState.pointCloud.zMin = Infinity;
    appState.pointCloud.zMax = -Infinity;
    appState.pointCloud.frameCount = 0;
    updateScanPage();
    showToast('点云已清空', 'info');
  });

  document.getElementById('exportMap').addEventListener('click', function() {
    apiPost('/api/v1/scan3d/generate_map', {
      map_name: document.getElementById('mapName').value,
      export_path: '/tmp',
      format: document.getElementById('exportFormat').value,
      include_path: true
    }).then(function(res) {
      if (res.success) {
        showToast('地图已导出: ' + res.output_file, 'success');
      }
    }).catch(function() {});
  });

  var canvas = document.getElementById('pointCloudCanvas');
  var isDragging = false, lastX, lastY;
  canvas.addEventListener('mousedown', function(e) {
    isDragging = true;
    lastX = e.clientX;
    lastY = e.clientY;
  });
  canvas.addEventListener('mousemove', function(e) {
    if (isDragging) {
      var dx = e.clientX - lastX;
      var dy = e.clientY - lastY;
      appState.pointCloud.rotation.yaw += dx * 0.01;
      appState.pointCloud.rotation.pitch += dy * 0.01;
      lastX = e.clientX;
      lastY = e.clientY;
    }
  });
  canvas.addEventListener('mouseup', function() { isDragging = false; });
  canvas.addEventListener('mouseleave', function() { isDragging = false; });
  canvas.addEventListener('wheel', function(e) {
    e.preventDefault();
    var zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
    appState.pointCloud.zoom *= zoomFactor;
    appState.pointCloud.zoom = Math.max(0.1, Math.min(10, appState.pointCloud.zoom));
  });

  var pcRenderLoop;
  function runRenderLoop() {
    renderPointCloud();
    updateScanPage();
    pcRenderLoop = requestAnimationFrame(runRenderLoop);
  }
  runRenderLoop();

  pollIntervals.push(setInterval(function() {
    apiGet('/api/v1/scan3d/points').then(function(data) {
      if (data && data.points_x) {
        addPoints(data);
      }
    }).catch(function() {});
  }, 500));
}

/**
 * 初始化摄像头配置页面
 * 
 * 设置默认配置、绑定加载/保存按钮事件、
 * 启动预览图轮询。
 */
function initCameraConfig() {
  // 初始化默认摄像头配置
  appState.cameraConfig = {
    device: '0',
    use_rtsp: false,
    fps: 30,
    width: 640,
    height: 480
  };

  // 加载按钮点击事件：从后端获取配置并更新界面
  document.getElementById('loadCameraConfig').addEventListener('click', function() {
    apiGet('/api/v1/camera/config').then(function(data) {
      if (data) {
        appState.cameraConfig = data;
        document.getElementById('cameraDevice').value = data.device;
        document.getElementById('useRtsp').checked = data.use_rtsp;
        document.getElementById('cameraFps').value = data.fps;
        document.getElementById('cameraWidth').value = data.width;
        document.getElementById('cameraHeight').value = data.height;
        showToast('摄像头配置加载成功', 'success');
      }
    }).catch(function() {});
  });

  // 保存按钮点击事件：从界面读取配置并保存到后端
  document.getElementById('saveCameraConfig').addEventListener('click', function() {
    appState.cameraConfig = {
      device: document.getElementById('cameraDevice').value,
      use_rtsp: document.getElementById('useRtsp').checked,
      fps: parseInt(document.getElementById('cameraFps').value),
      width: parseInt(document.getElementById('cameraWidth').value),
      height: parseInt(document.getElementById('cameraHeight').value)
    };
    apiPost('/api/v1/camera/config', appState.cameraConfig).then(function() {
      showToast('摄像头配置保存成功', 'success');
    }).catch(function() {});
  });

  // 轮询获取摄像头预览图，仅在配置页面激活时执行
  pollIntervals.push(setInterval(function() {
    if (document.getElementById('page-config-camera').classList.contains('active')) {
      apiGet('/api/v1/camera/preview').then(function(data) {
        if (data && data.image) {
          drawCameraPreview(data.image);
        }
      }).catch(function() {});
    }
  }, 300));
}

/**
 * 在 canvas 上绘制摄像头预览图
 * 
 * @param {string} base64Image - Base64 编码的 JPEG 图像
 */
function drawCameraPreview(base64Image) {
  var canvas = document.getElementById('cameraPreview');
  if (!canvas) return;
  var ctx = canvas.getContext('2d');
  var img = new Image();
  img.onload = function() {
    // 图像加载完成后，绘制到 canvas 上并隐藏占位文本
    ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
    document.getElementById('cameraPreviewEmpty').style.display = 'none';
  };
  img.src = 'data:image/jpeg;base64,' + base64Image;
}

/**
 * 初始化 PLC 配置页面
 * 
 * 设置设备列表、绑定各按钮事件、启动连接状态轮询。
 */
function initPlcConfig() {
  // PLC 设备列表和选中设备索引
  appState.plcDevices = [];
  appState.selectedPlcIndex = -1;

  /**
   * 刷新 PLC 设备表格显示
   */
  function refreshPlcTable() {
    var tbody = document.getElementById('plcDeviceTable');
    tbody.innerHTML = '';
    for (var i = 0; i < appState.plcDevices.length; i++) {
      var dev = appState.plcDevices[i];
      var tr = document.createElement('tr');
      // 构造表格行 HTML，包含单选框、设备信息和连接状态
      tr.innerHTML = '<td><input type="radio" name="plcSelect" value="' + i + '"' + (i === appState.selectedPlcIndex ? ' checked' : '') + '></td><td>' + (dev.name || 'unknown') + '</td><td>' + (dev.ip || '127.0.0.1') + '</td><td>' + (dev.port || 502) + '</td><td>' + (dev.slave_id || 1) + '</td><td>' + (dev.is_master ? '主站' : '从站') + '</td><td><span class="status-dot ' + (dev.connected ? 'connected' : 'disconnected') + '" style="display:inline-block"></span> ' + (dev.connected ? '已连接' : '未连接') + '</td>';
      tbody.appendChild(tr);
    }
    // 重新绑定单选框的选择事件
    var radios = document.getElementsByName('plcSelect');
    for (var i = 0; i < radios.length; i++) {
      radios[i].addEventListener('change', function(e) {
        appState.selectedPlcIndex = parseInt(e.target.value);
        updatePlcEditCard();
      });
    }
  }

  /**
   * 更新 PLC 编辑卡片显示
   * 
   * 根据选中的设备更新表单字段的值，
   * 若未选中则隐藏卡片。
   */
  function updatePlcEditCard() {
    if (appState.selectedPlcIndex >= 0 && appState.selectedPlcIndex < appState.plcDevices.length) {
      var dev = appState.plcDevices[appState.selectedPlcIndex];
      document.getElementById('plcEditCard').style.display = 'grid';
      document.getElementById('editPlcName').value = dev.name || '';
      document.getElementById('editPlcIp').value = dev.ip || '127.0.0.1';
      document.getElementById('editPlcPort').value = dev.port || 502;
      document.getElementById('editPlcSlave').value = dev.slave_id || 1;
      document.getElementById('editPlcIsMaster').checked = dev.is_master !== false;
      document.getElementById('editCoilStart').value = dev.coil_read_start || 0;
      document.getElementById('editCoilCount').value = dev.coil_read_count || 16;
      document.getElementById('editRegisterStart').value = dev.register_read_start || 0;
      document.getElementById('editRegisterCount').value = dev.register_read_count || 16;
    } else {
      document.getElementById('plcEditCard').style.display = 'none';
    }
  }

  // 加载配置按钮点击事件
  document.getElementById('loadPlcConfig').addEventListener('click', function() {
    apiGet('/api/v1/plc/config').then(function(data) {
      if (data) {
        appState.plcDevices = data.devices || [];
        appState.selectedPlcIndex = -1;
        refreshPlcTable();
        updatePlcEditCard();
        showToast('PLC配置加载成功', 'success');
      }
    }).catch(function() {});
  });

  // 保存配置按钮点击事件
  document.getElementById('savePlcConfig').addEventListener('click', function() {
    apiPost('/api/v1/plc/config', { devices: appState.plcDevices }).then(function() {
      showToast('PLC配置保存成功', 'success');
    }).catch(function() {});
  });

  // 添加新 PLC 设备按钮点击事件
  document.getElementById('addPlcDevice').addEventListener('click', function() {
    var newDevice = {
      name: 'device_' + (appState.plcDevices.length + 1),
      ip: '192.168.1.' + (100 + appState.plcDevices.length),
      port: 502,
      slave_id: 1,
      coil_read_start: 0,
      coil_read_count: 16,
      register_read_start: 0,
      register_read_count: 16,
      is_master: true,
      connected: false
    };
    appState.plcDevices.push(newDevice);
    appState.selectedPlcIndex = appState.plcDevices.length - 1;
    refreshPlcTable();
    updatePlcEditCard();
  });

  // 删除选中 PLC 设备按钮点击事件
  document.getElementById('removePlcDevice').addEventListener('click', function() {
    if (appState.selectedPlcIndex >= 0) {
      appState.plcDevices.splice(appState.selectedPlcIndex, 1);
      appState.selectedPlcIndex = -1;
      refreshPlcTable();
      updatePlcEditCard();
      showToast('已删除PLC设备', 'info');
    }
  });

  // 更新选中 PLC 设备配置按钮点击事件
  document.getElementById('updatePlcDevice').addEventListener('click', function() {
    if (appState.selectedPlcIndex >= 0) {
      var dev = appState.plcDevices[appState.selectedPlcIndex];
      dev.name = document.getElementById('editPlcName').value;
      dev.ip = document.getElementById('editPlcIp').value;
      dev.port = parseInt(document.getElementById('editPlcPort').value);
      dev.slave_id = parseInt(document.getElementById('editPlcSlave').value);
      dev.is_master = document.getElementById('editPlcIsMaster').checked;
      dev.coil_read_start = parseInt(document.getElementById('editCoilStart').value);
      dev.coil_read_count = parseInt(document.getElementById('editCoilCount').value);
      dev.register_read_start = parseInt(document.getElementById('editRegisterStart').value);
      dev.register_read_count = parseInt(document.getElementById('editRegisterCount').value);
      refreshPlcTable();
      showToast('设备配置已更新', 'success');
    }
  });

  // 发送从站（AGV）控制命令按钮点击事件
  document.getElementById('sendSlaveCommand').addEventListener('click', function() {
    var cmd = {
      linear_x: parseFloat(document.getElementById('slaveSpeedX').value),
      linear_y: parseFloat(document.getElementById('slaveSpeedY').value),
      angular_z: parseFloat(document.getElementById('slaveSpeedAng').value)
    };
    apiPost('/api/v1/plc/send_slave', cmd).then(function() {
      showToast('从站命令已发送', 'success');
    }).catch(function() {});
  });

  // 轮询更新 PLC 设备连接状态，仅在配置页面激活时执行
  pollIntervals.push(setInterval(function() {
    if (document.getElementById('page-config-plc').classList.contains('active')) {
      apiGet('/api/v1/plc/devices/status').then(function(data) {
        if (data && data.devices) {
          for (var i = 0; i < data.devices.length; i++) {
            var idx = appState.plcDevices.findIndex(function(d) { return d.name === data.devices[i].name; });
            if (idx >= 0) {
              appState.plcDevices[idx].connected = data.devices[i].connected;
            }
          }
          refreshPlcTable();
        }
      }).catch(function() {});
    }
  }, 1000));
}

/**
 * 初始化仿真测试页面
 */
function initSimulation() {
  // 仿真状态
  appState.simulationStatus = {
    simulation_enabled: false,
    camera_simulation: false,
    plc_simulation: false,
    vision_simulation: false
  };

  /**
   * 刷新仿真状态显示
   */
  function refreshSimStatus() {
    var camStatus = appState.simulationStatus.camera_simulation;
    var plcStatus = appState.simulationStatus.plc_simulation;
    var visStatus = appState.simulationStatus.vision_simulation;

    document.getElementById('simCameraStatus').textContent = camStatus ? '仿真模式' : '真实模式';
    document.getElementById('simPlcStatus').textContent = plcStatus ? '仿真模式' : '真实模式';
    document.getElementById('simVisionStatus').textContent = visStatus ? '仿真模式' : '真实模式';

    document.getElementById('simCamera').checked = camStatus;
    document.getElementById('simPlc').checked = plcStatus;
    document.getElementById('simVision').checked = visStatus;
  }

  /**
   * 加载仿真状态
   */
  function loadSimStatus() {
    apiGet('/api/v1/simulation/status').then(function(data) {
      if (data) {
        appState.simulationStatus = data;
        refreshSimStatus();
      }
    }).catch(function() {});
  }

  /**
   * 渲染配置历史
   */
  function renderConfigHistory(history) {
    var tbody = document.getElementById('configHistoryTable');
    tbody.innerHTML = '';
    if (!history || history.length === 0) {
      tbody.innerHTML = '<tr><td colspan="4" class="empty-state">无数据</td></tr>';
      return;
    }
    history.forEach(function(item) {
      var tr = document.createElement('tr');
      tr.innerHTML = '<td>' + (item.timestamp || '--') + '</td><td>' + (item.config_type || '--') + '</td><td>' + (item.old_value || '--') + '</td><td>' + (item.new_value || '--') + '</td>';
      tbody.appendChild(tr);
    });
  }

  // 应用仿真设置按钮
  document.getElementById('applySimulation').addEventListener('click', function() {
    var payload = {
      simulation_enabled: true,
      camera_simulation: document.getElementById('simCamera').checked,
      plc_simulation: document.getElementById('simPlc').checked,
      vision_simulation: document.getElementById('simVision').checked
    };
    apiPost('/api/v1/simulation/status', payload).then(function() {
      loadSimStatus();
      showToast('仿真设置已应用', 'success');
    }).catch(function() {});
  });

  // 刷新状态按钮
  document.getElementById('refreshSimStatus').addEventListener('click', function() {
    loadSimStatus();
  });

  // 加载配置历史按钮
  document.getElementById('loadConfigHistory').addEventListener('click', function() {
    apiGet('/api/v1/data/config/history').then(function(data) {
      if (data && data.history) {
        renderConfigHistory(data.history);
        showToast('配置历史已加载', 'success');
      }
    }).catch(function() {});
  });

  // 初始加载状态
  loadSimStatus();
}

/**
 * 初始化模型训练页面
 */
function initVisionTrain() {
  appState.visionTrain = {
    modelInfo: {},
    trainingStatus: {},
    availableModels: []
  };

  function loadVisionInfo() {
    apiGet('/api/v1/vision/info').then(function(data) {
      appState.visionTrain.modelInfo = data || {};
      var el = document.getElementById('vtModelType');
      if (el) el.textContent = (data && data.model_name) ? data.model_name : '--';
    }).catch(function() {});
  }

  function loadTrainingStatus() {
    apiGet('/api/v1/vision/train/status').then(function(data) {
      appState.visionTrain.trainingStatus = data || {};
      renderTrainingStatus(data);
    }).catch(function() {});
  }

  function renderTrainingStatus(status) {
    if (!status) return;
    var statusText = status.status || 'idle';
    var el = document.getElementById('vtTrainStatus');
    if (el) {
      var labels = { idle: '就绪', training: '训练中', fine_tuning: '微调中', completed: '已完成', failed: '失败' };
      el.textContent = labels[statusText] || statusText;
    }
    el = document.getElementById('vtCurrentEpoch');
    if (el) el.textContent = status.epoch || 0;
    el = document.getElementById('vtTotalEpochs');
    if (el) el.textContent = status.total_epochs || 0;
    el = document.getElementById('vtLoss');
    if (el) el.textContent = status.loss ? parseFloat(status.loss).toFixed(4) : '--';
    el = document.getElementById('vtAccuracy');
    if (el) el.textContent = status.map50 ? (parseFloat(status.map50) * 100).toFixed(1) : '--';
    el = document.getElementById('vtTrainTime');
    if (el) el.textContent = status.training_time ? (status.training_time / 60).toFixed(1) + ' min' : '--';
    el = document.getElementById('vtOutputPath');
    if (el) el.textContent = status.best_model_path || '--';

    // 更新进度条
    var progressEl = document.getElementById('vtProgressFill');
    var progressText = document.getElementById('vtProgressText');
    var progressPercent = document.getElementById('vtProgressPercent');
    if (status.total_epochs > 0) {
      var pct = Math.round(((status.epoch || 0) / status.total_epochs) * 100);
      if (progressEl) progressEl.style.width = pct + '%';
      if (progressText) progressText.textContent = statusText === 'completed' ? '训练完成' : statusText === 'failed' ? '训练失败' : '训练中...';
      if (progressPercent) progressPercent.textContent = pct + '%';
    }
  }

  // 开始训练按钮
  var startTrainBtn = document.getElementById('vtStartTrain');
  if (startTrainBtn) {
    startTrainBtn.addEventListener('click', function() {
      var datasetPath = document.getElementById('vtDatasetPath');
      var baseModel = document.getElementById('vtBaseModel');
      var epochs = document.getElementById('vtEpochs');
      var lr = document.getElementById('vtLearningRate');
      var imgSize = document.getElementById('vtImgSize');
      var batchSize = document.getElementById('vtBatchSize');
      var fineTuneModel = document.getElementById('vtFineTuneModel');

      if (!datasetPath || !datasetPath.value.trim()) {
        showToast('请输入数据集路径', 'error');
        return;
      }

      var payload = {
        dataset_path: datasetPath.value,
        model_type: baseModel ? baseModel.value : 'yolov8n',
        epochs: epochs ? parseInt(epochs.value) || 50 : 50,
        learning_rate: lr ? parseFloat(lr.value) || 0.001 : 0.001,
        imgsz: imgSize ? parseInt(imgSize.value) || 640 : 640,
        batch_size: batchSize ? parseInt(batchSize.value) || 16 : 16,
      };
      var ftModel = fineTuneModel ? fineTuneModel.value.trim() : '';
      if (ftModel) {
        payload.fine_tune_from = ftModel;
      }

      apiPost('/api/v1/vision/train', payload).then(function(data) {
        showToast('训练已启动', 'success');
        startTrainingPolling();
      }).catch(function(err) {
        showToast('训练启动失败: ' + err.message, 'error');
      });
    });
  }

  // 取消训练按钮
  var cancelBtn = document.getElementById('vtCancelTrain');
  if (cancelBtn) {
    cancelBtn.addEventListener('click', function() {
      apiPost('/api/v1/vision/train/cancel', {}).then(function(data) {
        showToast('已请求取消训练', 'success');
      }).catch(function() {
        showToast('取消训练失败', 'error');
      });
    });
  }

  // 导出模型按钮
  var exportBtn = document.getElementById('vtExportModel');
  if (exportBtn) {
    exportBtn.addEventListener('click', function() {
      var modelPath = document.getElementById('vtExportModelPath');
      var exportFormat = document.getElementById('vtExportFormat');
      if (!modelPath || !modelPath.value.trim()) {
        showToast('请输入模型路径', 'error');
        return;
      }
      apiPost('/api/v1/vision/export', {
        model_path: modelPath.value,
        format: exportFormat ? exportFormat.value : 'onnx'
      }).then(function(data) {
        showToast('模型导出成功', 'success');
      }).catch(function() {
        showToast('模型导出失败', 'error');
      });
    });
  }

  // 训练状态轮询
  var trainingPollingInterval = null;
  function startTrainingPolling() {
    if (trainingPollingInterval) clearInterval(trainingPollingInterval);
    trainingPollingInterval = setInterval(function() {
      loadTrainingStatus();
      var status = appState.visionTrain.trainingStatus;
      if (status && (status.status === 'completed' || status.status === 'failed')) {
        clearInterval(trainingPollingInterval);
        trainingPollingInterval = null;
      }
    }, 3000);
  }

  // 初始加载
  loadVisionInfo();
  loadTrainingStatus();
}

function initRemoteControl() {
  var rcState = {
    linear_x: 0,
    angular_z: 0,
    speed: 0,
    steering: 0,
    joyDragging: false,
    lastJoySend: 0
  };

  var MAX_LINEAR = 1.5;
  var MAX_ANGULAR = 1.5;
  var JOY_THROTTLE_MS = 100;

  var container = document.getElementById('joystickContainer');
  var knob = document.getElementById('joystickKnob');

  if (container && knob) {
    function getJoyCenter() {
      var rect = container.getBoundingClientRect();
      return { x: rect.left + rect.width / 2, y: rect.top + rect.height / 2, radius: rect.width / 2 };
    }

    function updateKnob(clientX, clientY) {
      var center = getJoyCenter();
      var dx = clientX - center.x;
      var dy = clientY - center.y;
      var dist = Math.sqrt(dx * dx + dy * dy);
      var maxDist = center.radius - knob.offsetWidth / 2;
      if (maxDist <= 0) maxDist = 1;

      if (dist > maxDist) {
        dx = dx / dist * maxDist;
        dy = dy / dist * maxDist;
      }

      knob.style.left = (center.radius + dx - knob.offsetWidth / 2) + 'px';
      knob.style.top = (center.radius + dy - knob.offsetHeight / 2) + 'px';

      var normX = dx / maxDist;
      var normY = -dy / maxDist;

      rcState.angular_z = parseFloat((normX * MAX_ANGULAR).toFixed(3));
      rcState.linear_x = parseFloat((normY * MAX_LINEAR).toFixed(3));

      var joyLinearEl = document.getElementById('rcJoyLinear');
      var joyAngularEl = document.getElementById('rcJoyAngular');
      if (joyLinearEl) joyLinearEl.textContent = rcState.linear_x.toFixed(2);
      if (joyAngularEl) joyAngularEl.textContent = rcState.angular_z.toFixed(2);

      sendJoystickCommand();
    }

    function resetKnob() {
      knob.style.left = '50%';
      knob.style.top = '50%';
      knob.style.transform = 'translate(-50%, -50%)';
      rcState.linear_x = 0;
      rcState.angular_z = 0;

      var joyLinearEl = document.getElementById('rcJoyLinear');
      var joyAngularEl = document.getElementById('rcJoyAngular');
      if (joyLinearEl) joyLinearEl.textContent = '0.00';
      if (joyAngularEl) joyAngularEl.textContent = '0.00';

      apiPost('/api/v1/motor/joystick', { linear_x: 0, angular_z: 0 }).catch(function() {});
    }

    container.addEventListener('mousedown', function(e) {
      e.preventDefault();
      rcState.joyDragging = true;
      knob.style.transform = 'none';
      updateKnob(e.clientX, e.clientY);
    });

    document.addEventListener('mousemove', function(e) {
      if (rcState.joyDragging) {
        e.preventDefault();
        updateKnob(e.clientX, e.clientY);
      }
    });

    document.addEventListener('mouseup', function() {
      if (rcState.joyDragging) {
        rcState.joyDragging = false;
        resetKnob();
      }
    });

    container.addEventListener('touchstart', function(e) {
      e.preventDefault();
      rcState.joyDragging = true;
      knob.style.transform = 'none';
      var touch = e.touches[0];
      updateKnob(touch.clientX, touch.clientY);
    }, { passive: false });

    document.addEventListener('touchmove', function(e) {
      if (rcState.joyDragging) {
        e.preventDefault();
        var touch = e.touches[0];
        updateKnob(touch.clientX, touch.clientY);
      }
    }, { passive: false });

    document.addEventListener('touchend', function() {
      if (rcState.joyDragging) {
        rcState.joyDragging = false;
        resetKnob();
      }
    });
  }

  function sendJoystickCommand() {
    var now = Date.now();
    if (now - rcState.lastJoySend < JOY_THROTTLE_MS) return;
    rcState.lastJoySend = now;
    apiPost('/api/v1/motor/joystick', { linear_x: rcState.linear_x, angular_z: rcState.angular_z }).catch(function() {});
  }

  var speedSlider = document.getElementById('rcSpeedSlider');
  if (speedSlider) {
    speedSlider.addEventListener('input', function() {
      var val = parseFloat(this.value);
      rcState.speed = val;
      var display = document.getElementById('rcSpeedVal');
      if (display) display.textContent = val.toFixed(1);
      apiPost('/api/v1/motor/speed', { speed: val }).catch(function() {});
    });
  }

  var speedUpBtn = document.getElementById('rcSpeedUp');
  if (speedUpBtn) {
    speedUpBtn.addEventListener('click', function() {
      var slider = document.getElementById('rcSpeedSlider');
      if (!slider) return;
      var val = Math.min(MAX_LINEAR, parseFloat(slider.value) + 0.1);
      val = Math.round(val * 10) / 10;
      slider.value = val;
      rcState.speed = val;
      var display = document.getElementById('rcSpeedVal');
      if (display) display.textContent = val.toFixed(1);
      apiPost('/api/v1/motor/speed', { speed: val }).catch(function() {});
    });
  }

  var speedDownBtn = document.getElementById('rcSpeedDown');
  if (speedDownBtn) {
    speedDownBtn.addEventListener('click', function() {
      var slider = document.getElementById('rcSpeedSlider');
      if (!slider) return;
      var val = Math.max(-MAX_LINEAR, parseFloat(slider.value) - 0.1);
      val = Math.round(val * 10) / 10;
      slider.value = val;
      rcState.speed = val;
      var display = document.getElementById('rcSpeedVal');
      if (display) display.textContent = val.toFixed(1);
      apiPost('/api/v1/motor/speed', { speed: val }).catch(function() {});
    });
  }

  var speedStopBtn = document.getElementById('rcSpeedStop');
  if (speedStopBtn) {
    speedStopBtn.addEventListener('click', function() {
      var slider = document.getElementById('rcSpeedSlider');
      if (slider) slider.value = 0;
      rcState.speed = 0;
      var display = document.getElementById('rcSpeedVal');
      if (display) display.textContent = '0.0';
      apiPost('/api/v1/motor/speed', { speed: 0 }).catch(function() {});
    });
  }

  var steerSlider = document.getElementById('rcSteerSlider');
  if (steerSlider) {
    steerSlider.addEventListener('input', function() {
      var val = parseFloat(this.value);
      rcState.steering = val;
      var display = document.getElementById('rcSteerVal');
      if (display) display.textContent = val.toFixed(1);
      apiPost('/api/v1/motor/steering', { angle: val }).catch(function() {});
    });
  }

  var steerLeftBtn = document.getElementById('rcSteerLeft');
  if (steerLeftBtn) {
    steerLeftBtn.addEventListener('click', function() {
      var slider = document.getElementById('rcSteerSlider');
      if (!slider) return;
      var val = Math.max(-MAX_ANGULAR, parseFloat(slider.value) - 0.1);
      val = Math.round(val * 10) / 10;
      slider.value = val;
      rcState.steering = val;
      var display = document.getElementById('rcSteerVal');
      if (display) display.textContent = val.toFixed(1);
      apiPost('/api/v1/motor/steering', { angle: val }).catch(function() {});
    });
  }

  var steerRightBtn = document.getElementById('rcSteerRight');
  if (steerRightBtn) {
    steerRightBtn.addEventListener('click', function() {
      var slider = document.getElementById('rcSteerSlider');
      if (!slider) return;
      var val = Math.min(MAX_ANGULAR, parseFloat(slider.value) + 0.1);
      val = Math.round(val * 10) / 10;
      slider.value = val;
      rcState.steering = val;
      var display = document.getElementById('rcSteerVal');
      if (display) display.textContent = val.toFixed(1);
      apiPost('/api/v1/motor/steering', { angle: val }).catch(function() {});
    });
  }

  var steerCenterBtn = document.getElementById('rcSteerCenter');
  if (steerCenterBtn) {
    steerCenterBtn.addEventListener('click', function() {
      var slider = document.getElementById('rcSteerSlider');
      if (slider) slider.value = 0;
      rcState.steering = 0;
      var display = document.getElementById('rcSteerVal');
      if (display) display.textContent = '0.0';
      apiPost('/api/v1/motor/steering', { angle: 0 }).catch(function() {});
    });
  }

  function isRemoteControlPageVisible() {
    var page = document.getElementById('page-remote-control');
    if (!page) return false;
    return page.classList.contains('active');
  }

  document.addEventListener('keydown', function(e) {
    if (!isRemoteControlPageVisible()) return;

    var key = e.key.toLowerCase();
    var changed = false;

    if (key === 'w') {
      rcState.linear_x = Math.min(MAX_LINEAR, rcState.linear_x + 0.1);
      rcState.linear_x = Math.round(rcState.linear_x * 10) / 10;
      changed = true;
    } else if (key === 's') {
      rcState.linear_x = Math.max(-MAX_LINEAR, rcState.linear_x - 0.1);
      rcState.linear_x = Math.round(rcState.linear_x * 10) / 10;
      changed = true;
    } else if (key === 'a') {
      rcState.angular_z = Math.min(MAX_ANGULAR, rcState.angular_z + 0.2);
      rcState.angular_z = Math.round(rcState.angular_z * 10) / 10;
      changed = true;
    } else if (key === 'd') {
      rcState.angular_z = Math.max(-MAX_ANGULAR, rcState.angular_z - 0.2);
      rcState.angular_z = Math.round(rcState.angular_z * 10) / 10;
      changed = true;
    } else if (key === ' ') {
      e.preventDefault();
      apiPost('/api/v1/motor/emergency_brake', {}).catch(function() {});
      return;
    } else if (key === 'q') {
      var slider = document.getElementById('rcSpeedSlider');
      if (slider) {
        var val = Math.max(-MAX_LINEAR, parseFloat(slider.value) - 0.1);
        val = Math.round(val * 10) / 10;
        slider.value = val;
        rcState.speed = val;
        var display = document.getElementById('rcSpeedVal');
        if (display) display.textContent = val.toFixed(1);
        apiPost('/api/v1/motor/speed', { speed: val }).catch(function() {});
      }
      return;
    } else if (key === 'e') {
      var slider = document.getElementById('rcSpeedSlider');
      if (slider) {
        var val = Math.min(MAX_LINEAR, parseFloat(slider.value) + 0.1);
        val = Math.round(val * 10) / 10;
        slider.value = val;
        rcState.speed = val;
        var display = document.getElementById('rcSpeedVal');
        if (display) display.textContent = val.toFixed(1);
        apiPost('/api/v1/motor/speed', { speed: val }).catch(function() {});
      }
      return;
    }

    if (changed) {
      e.preventDefault();
      var joyLinearEl = document.getElementById('rcJoyLinear');
      var joyAngularEl = document.getElementById('rcJoyAngular');
      if (joyLinearEl) joyLinearEl.textContent = rcState.linear_x.toFixed(2);
      if (joyAngularEl) joyAngularEl.textContent = rcState.angular_z.toFixed(2);
      apiPost('/api/v1/motor/joystick', { linear_x: rcState.linear_x, angular_z: rcState.angular_z }).catch(function() {});
    }
  });

  var modeManualBtn = document.getElementById('rcModeManual');
  if (modeManualBtn) {
    modeManualBtn.addEventListener('click', function() {
      apiPost('/api/v1/agv/control', { command: 'start', parameters: ['manual'] })
        .then(function() { showToast('切换到手动模式', 'success'); })
        .catch(function() { showToast('切换手动模式失败', 'error'); });
    });
  }

  var modeAutoBtn = document.getElementById('rcModeAuto');
  if (modeAutoBtn) {
    modeAutoBtn.addEventListener('click', function() {
      apiPost('/api/v1/agv/control', { command: 'start', parameters: ['auto'] })
        .then(function() { showToast('切换到自动模式', 'success'); })
        .catch(function() { showToast('切换自动模式失败', 'error'); });
    });
  }

  var emergencyBtn = document.getElementById('rcEmergencyBrake');
  if (emergencyBtn) {
    emergencyBtn.addEventListener('click', function() {
      apiPost('/api/v1/motor/emergency_brake', {})
        .then(function() { showToast('紧急制动已触发', 'error'); })
        .catch(function() { showToast('紧急制动失败', 'error'); });
    });
  }

  pollIntervals.push(setInterval(function() {
    apiGet('/api/v1/motor/state').then(function(data) {
      if (!data) return;
      var el;

      el = document.getElementById('rcCurrentSpeed');
      if (el) {
        var avgSpeed = ((data.left_wheel_speed || 0) + (data.right_wheel_speed || 0)) / 2;
        el.textContent = avgSpeed.toFixed(3);
      }

      el = document.getElementById('rcCurrentSteering');
      if (el) el.textContent = (data.steering_angle || 0).toFixed(3);

      el = document.getElementById('rcLeftWheel');
      if (el) el.textContent = (data.left_wheel_speed || 0).toFixed(3);

      el = document.getElementById('rcRightWheel');
      if (el) el.textContent = (data.right_wheel_speed || 0).toFixed(3);

      el = document.getElementById('rcMotorMode');
      if (el) el.textContent = data.mode || '--';

      el = document.getElementById('rcBrakeActive');
      if (el) el.textContent = data.brake_active ? 'Yes' : 'No';

      el = document.getElementById('rcLeftCurrent');
      if (el) el.textContent = (data.left_motor_current || 0).toFixed(3);

      el = document.getElementById('rcRightCurrent');
      if (el) el.textContent = (data.right_motor_current || 0).toFixed(3);

      el = document.getElementById('rcTargetSpeed');
      if (el) el.textContent = (data.target_speed || 0).toFixed(3);

      el = document.getElementById('rcTargetSteering');
      if (el) el.textContent = (data.target_steering_angle || 0).toFixed(3);
    }).catch(function() {});
  }, 500));
}

document.addEventListener('DOMContentLoaded', function() {
  initNavigation();
  initSliders();
  initMap();
  renderIOList();
  renderWaypoints();
  renderRos2Nodes([]);
  initPowerManagement();
  init3dScanner();
  initCameraConfig();
  initPlcConfig();
  initSimulation();
  initVisionTrain();
  initRemoteControl();

  addLog('System initialized', 'info');

  initWebSockets();
  startPolling();

  window.addEventListener('resize', function() {
    resizeMapCanvas();
    drawMap();
  });

  var modalEl = document.getElementById('modal');
  if (modalEl) {
    modalEl.addEventListener('click', function(e) {
      if (e.target === this) closeModal();
    });
  }
});
