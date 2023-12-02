import * as THREE from 'three';
import WebGL from 'three/addons/capabilities/WebGL.js';


const scene = new THREE.Scene();
scene.background = new THREE.Color(0x999999);
scene.fog = new THREE.Fog(0x999999, 0, 50);
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);

const geometry = new THREE.BoxGeometry(1, 1, 1);
const material = new THREE.MeshBasicMaterial({color: 0x003300});
const cube = new THREE.Mesh(geometry, material);
scene.add(cube);
camera.position.z = 5;

function animate() {
    requestAnimationFrame(animate);
    cube.rotation.z += 0.01;
    cube.rotation.y += 0.01;
    renderer.render(scene, camera);
}

if (WebGL.isWebGLAvailable()) {
    document.body.appendChild(renderer.domElement);
    animate();
} else {
    const warning = WebGL.getWebGLErrorMessage();
    document.body.appendChild(warning);
}
