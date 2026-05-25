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

document.addEventListener('DOMContentLoaded', function() {
  initNavigation();
  initSliders();
  initMap();
  renderIOList();
  renderWaypoints();
  renderRos2Nodes([]);
  initPowerManagement();

  addLog('System initialized', 'info');

  initWebSockets();
  startPolling();

  window.addEventListener('resize', function() {
    resizeMapCanvas();
    drawMap();
  });

  document.getElementById('modal').addEventListener('click', function(e) {
    if (e.target === this) closeModal();
  });
});
