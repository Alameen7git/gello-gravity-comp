/**
 * GELLO UR5 Dashboard - Next-Gen Three.js Visualization
 * This file handles the 3D rendering of the UR5 arm and WebSocket data.
 */

// Scene Setup
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
const container = document.getElementById('viewport');
renderer.setSize(container.offsetWidth, container.offsetHeight);
renderer.setPixelRatio(window.devicePixelRatio);
container.appendChild(renderer.domElement);

// Lighting
const ambientLight = new THREE.AmbientLight(0x404040, 2);
scene.add(ambientLight);
const directionalLight = new THREE.DirectionalLight(0xffffff, 1.5);
directionalLight.position.set(5, 5, 5);
scene.add(directionalLight);
const secondaryLight = new THREE.PointLight(0x00f2fe, 1, 10);
secondaryLight.position.set(-2, 2, 2);
scene.add(secondaryLight);

// Robot Parts Materials
const grayMat = new THREE.MeshStandardMaterial({ color: 0x333333, metalness: 0.8, roughness: 0.2 });
const lightGrayMat = new THREE.MeshStandardMaterial({ color: 0x888888, metalness: 0.8, roughness: 0.2 });
const accentMat = new THREE.MeshStandardMaterial({ color: 0x00f2fe, emissive: 0x00f2fe, emissiveIntensity: 0.5 });

// Build UR5 Structure (Simplified Primitives to match URDF/Kinematics)
// Dimensions approx based on 0.25x scale (~UR5-like)
class RobotLink extends THREE.Group {
    constructor(parent, length, radius = 0.04, color = 0x888888) {
        super();
        this.joint = new THREE.Group();
        this.add(this.joint);
        
        // Link Cylinder
        const geometry = new THREE.CylinderGeometry(radius, radius, length, 16);
        const mesh = new THREE.Mesh(geometry, grayMat);
        mesh.position.y = length / 2;
        this.joint.add(mesh);
        
        // Offset for child
        this.childConnector = new THREE.Group();
        this.childConnector.position.y = length;
        this.joint.add(this.childConnector);
    }
}

// Construct visual robot (matching UR5 hierarchy roughly)
const base = new THREE.Mesh(new THREE.CylinderGeometry(0.1, 0.1, 0.05, 32), lightGrayMat);
scene.add(base);

const joints = []; // Store joint groups for rotation
const link1 = new RobotLink(base, 0.08, 0.05); // Base to Shoulder
base.add(link1);
joints.push(link1); // J1 (Pan)

const link2 = new RobotLink(link1.childConnector, 0.25, 0.04); // Shoulder to Elbow
link1.childConnector.add(link2);
link2.rotation.z = Math.PI / 2;
joints.push(link2); // J2 (Lift)

const link3 = new RobotLink(link2.childConnector, 0.25, 0.04); // Elbow to Wrist 1
link2.childConnector.add(link3);
joints.push(link3); // J3 (Elbow)

const link4 = new RobotLink(link3.childConnector, 0.1, 0.03); // Wrist 1 to 2
link3.childConnector.add(link4);
link4.rotation.z = Math.PI / 2;
joints.push(link4); // J4 (Wrist 1)

const link5 = new RobotLink(link4.childConnector, 0.08, 0.03); // Wrist 2 to 3
link4.childConnector.add(link5);
link5.rotation.x = Math.PI / 2;
joints.push(link5); // J5 (Wrist 2)

const link6 = new RobotLink(link5.childConnector, 0.05, 0.02); // Wrist 3 to Tip
link5.childConnector.add(link6);
joints.push(link6); // J6 (Wrist 3)

camera.position.set(1, 1, 1);
camera.lookAt(0, 0.2, 0);

// Animation Loop
function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
}
animate();

// Resizing
window.addEventListener('resize', () => {
    renderer.setSize(container.offsetWidth, container.offsetHeight);
    camera.aspect = container.offsetWidth / container.offsetHeight;
    camera.updateProjectionMatrix();
});

// WebSocket Communication
const socket = io();
const gainSlider = document.getElementById('gain-slider');
const gainVal = document.getElementById('gain-val');
const jointList = document.getElementById('joint-list');
const connStatus = document.getElementById('conn-status');

// Initialize Telemetry Cards
for (let i = 1; i <= 6; i++) {
    const card = document.createElement('div');
    card.className = 'joint-card';
    card.innerHTML = `
        <div class="joint-header">
            <span class="joint-name">JOINT ${i}</span>
            <span class="joint-value" id="j-val-${i}">0.00°</span>
        </div>
        <div class="bar-container">
            <div class="bar-fill" id="j-bar-${i}" style="width: 50%;"></div>
        </div>
    `;
    jointList.appendChild(card);
}

socket.on('connect', () => {
    connStatus.textContent = 'SYSTEM CONNECTED';
    connStatus.style.background = 'rgba(76, 175, 80, 0.1)';
});

socket.on('disconnect', () => {
    connStatus.textContent = 'SYSTEM DISCONNECTED';
    connStatus.style.background = 'rgba(244, 67, 54, 0.1)';
});

// Real-time Robot Telemetry
socket.on('telemetry', (data) => {
    const q = data.q; // Current joint angles in degrees
    const tau = data.tau; // Torques
    
    for (let i = 0; i < 6; i++) {
        const angle = q[i];
        // Update 3D Model
        if (joints[i]) {
            // Mapping signs and axes to visual model
            if (i === 0) joints[i].rotation.y = THREE.MathUtils.degToRad(angle);
            else if (i === 1 || i === 2 || i === 3) joints[i].rotation.z = THREE.MathUtils.degToRad(angle);
            else if (i === 4) joints[i].rotation.x = THREE.MathUtils.degToRad(angle);
            else if (i === 5) joints[i].rotation.y = THREE.MathUtils.degToRad(angle);
        }
        
        // Update Gauges
        document.getElementById(`j-val-${i+1}`).textContent = `${angle.toFixed(2)}°`;
        const percent = ((angle + 180) / 360) * 100;
        document.getElementById(`j-bar-${i+1}`).style.width = `${percent}%`;
    }
});

// Control Handlers
gainSlider.addEventListener('input', (e) => {
    const val = parseFloat(e.target.value);
    gainVal.textContent = val.toFixed(2);
    socket.emit('update_gain', { value: val });
});

document.getElementById('start-btn').addEventListener('click', () => {
    socket.emit('toggle_run');
});
document.getElementById('connect-btn').addEventListener('click', () => {
    socket.emit('connect_hardware');
});
document.getElementById('calibrate-btn').addEventListener('click', () => {
    socket.emit('calibrate');
});

socket.on('notification', (data) => {
    // Simple toast notification or log
    console.log(data.message);
});
