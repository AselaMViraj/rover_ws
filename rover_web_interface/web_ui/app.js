// Rover Web Commander - Logical Core

// 1. Connection Logic
const ros = new ROSLIB.Ros({
    url: `ws://${window.location.hostname}:9090`
});

const statusEl = document.getElementById('status');
ros.on('connection', () => {
    statusEl.innerText = 'Connected';
    statusEl.className = 'connected';
});

ros.on('error', (error) => {
    statusEl.innerText = 'Error Connecting';
    statusEl.className = 'disconnected';
    console.error(error);
});

ros.on('close', () => {
    statusEl.innerText = 'Disconnected';
    statusEl.className = 'disconnected';
});

// 2. Map Rendering
const mapDiv = document.getElementById('map');
const viewer = new ROS2D.Viewer({
    divID: 'map',
    width: mapDiv.clientWidth,
    height: mapDiv.clientHeight
});

// Create Global Occupancy Grid client
const gridClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: viewer.scene,
    continuous: true
});

// Add Local Costmap Layer (transparent)
const localCostmapClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: viewer.scene,
    topic: '/local_costmap/costmap',
    continuous: true,
    opacity: 0.5
});

// Add Global Costmap Layer (transparent)
const globalCostmapClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: viewer.scene,
    topic: '/global_costmap/costmap',
    continuous: true,
    opacity: 0.3
});

// Scale map to fit on initial load
let initialScaleDone = false;
gridClient.on('change', () => {
    if (!initialScaleDone && gridClient.currentGrid) {
        const width = gridClient.currentGrid.width;
        const height = gridClient.currentGrid.height;

        // Calculate proportional scale to fit viewer
        const scale = Math.min(
            viewer.width / width,
            viewer.height / height
        ) * 0.9; // 90% of available space

        viewer.scene.scaleX = scale;
        viewer.scene.scaleY = scale;

        viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
        initialScaleDone = true;
    }
});

// 3. Interaction Logic
const zoomSpeed = 1.1;
mapDiv.addEventListener('wheel', (event) => {
    event.preventDefault();
    const zoom = event.deltaY > 0 ? 1 / zoomSpeed : zoomSpeed;
    viewer.scene.scaleX *= zoom;
    viewer.scene.scaleY *= zoom;
});

let isDragging = false;
let startPos = { x: 0, y: 0 };

viewer.scene.on('stagemousedown', (event) => {
    if (event.nativeEvent.button === 0) { // Left click for marking
        const pos = viewer.scene.globalToRos(event.stageX, event.stageY);
        tempPose = pos;
        waypointForm.classList.remove('hidden');
        document.getElementById('waypoint-name').focus();
    } else { // Other clicks for panning
        isDragging = true;
        startPos = { x: event.stageX, y: event.stageY };
    }
});

viewer.scene.on('stagemousemove', (event) => {
    if (isDragging) {
        const dx = event.stageX - startPos.x;
        const dy = event.stageY - startPos.y;
        viewer.scene.x += dx;
        viewer.scene.y += dy;
        startPos = { x: event.stageX, y: event.stageY };
    }
});

viewer.scene.on('stagemouseup', () => {
    isDragging = false;
});

// Disable context menu to allow right-click panning
mapDiv.addEventListener('contextmenu', (e) => e.preventDefault());

// 4. ROS Communication
const addWaypointTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/add_waypoint',
    messageType: 'std_msgs/String'
});

const waypointsListTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/waypoints_list',
    messageType: 'std_msgs/String'
});

const navToWaypointTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/nav_to_waypoint',
    messageType: 'std_msgs/String'
});

// 5. Waypoint Management
let tempPose = null;
const waypointForm = document.getElementById('waypoint-form');

document.getElementById('save-waypoint').addEventListener('click', () => {
    const name = document.getElementById('waypoint-name').value;
    if (name && tempPose) {
        const msg = new ROSLIB.Message({
            data: `${name},${tempPose.x},${tempPose.y},0.0,1.0`
        });
        addWaypointTopic.publish(msg);
        waypointForm.classList.add('hidden');
        document.getElementById('waypoint-name').value = '';
    }
});

document.getElementById('cancel-waypoint').addEventListener('click', () => {
    waypointForm.classList.add('hidden');
});

waypointsListTopic.subscribe((message) => {
    try {
        const waypoints = JSON.parse(message.data);
        updateWaypointsList(waypoints);
    } catch (e) { console.error("Error parsing waypoints:", e); }
});

function updateWaypointsList(waypoints) {
    const listEl = document.getElementById('waypoints-list');
    listEl.innerHTML = '';

    Object.keys(waypoints).forEach(name => {
        const wp = waypoints[name];
        const item = document.createElement('li');
        item.className = 'waypoint-item';
        item.innerHTML = `
            <span>${name}</span>
            <div class="waypoint-controls">
                <button class="btn-go" onclick="goToWaypoint('${name}')">Go</button>
            </div>
        `;
        listEl.appendChild(item);
    });
}

function goToWaypoint(name) {
    const msg = new ROSLIB.Message({
        data: name
    });
    navToWaypointTopic.publish(msg);
    console.log(`Requested navigation to ${name}`);
}

window.goToWaypoint = goToWaypoint;

window.addEventListener('resize', () => {
    viewer.width = mapDiv.clientWidth;
    viewer.height = mapDiv.clientHeight;
});
