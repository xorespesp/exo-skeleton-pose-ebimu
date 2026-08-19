// Entry point: build the scene, load the rig, and wire a lil-gui panel for the connection,
// the pose stream, and the rest-pose commands.

import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
import { createScene, loadCharacter } from './scene.js';
import {
    captureBindPose, applyPoseFrame, resetToBindPose, resolveDriveSource,
    dumpPoseFrame, formatPoseFrameDump, DRIVE_SOURCE,
} from './skeleton.js';
import { PoseClient } from './pose-client.js';

const { scene, startRenderLoop } = createScene(document.getElementById('container'));

const { model, bones } = await loadCharacter(scene);
const bindPose = captureBindPose(bones);

const DRIVE_MODE_AUTO = 'auto';
let lastDriveSource = null; // the source the rig currently stands on
startRenderLoop();

const debugOverlayEl = document.getElementById('debug-overlay');

// --- ui state -----------------------------------------------------------------
const ui = {
    url: 'ws://localhost:9002',
    connection: 'disconnected',
    // readouts
    source_name: '(none)',
    rest_pose: 'none',
    frame_seq: 0,
    // Which wire field drives the rig. 'auto' follows the frame's rest-pose flag, so the rig runs
    // off the clinical angles until a rest is captured and off the rotations after; the other two
    // pin it, which is what lets the same instant be seen both ways.
    drive_mode: DRIVE_MODE_AUTO,
    drive_source: '-', // the source actually in use on the last frame
    // Give a joint the frame left empty its twin's value, which puts a whole character on screen
    // from the one leg a side view sees.
    mirror_unmeasured: false,
    show_frame_dump: false,
};

// Transient control flags (not shown in the panel).
let connected = false;   // WebSocket open
let connecting = false;  // connect in flight
let opened = false;      // a source is open on the server
let pending = false;     // a command is awaiting its Ack

// --- pose client --------------------------------------------------------------
const client = new PoseClient();

client.onOpen = () => {
    // The socket is up but the protocol handshake is still in flight; 
    // commands stay disabled until the server accepts our version. (onReady)
    ui.connection = 'handshaking';
    refresh();
};
client.onReady = () => {
    connecting = false; connected = true;
    ui.connection = 'connected';
    startStatusPoll(); // resync ServerStatus while connected
    refresh();
};
client.onProtocolError = (reason) => {
    // The server has closed the socket on us; onClose resets the rest of the state.
    ui.connection = 'protocol error';
    console.error(`rejected by the server: ${reason}`);
};
client.onClose = () => {
    connecting = false; connected = false; opened = false; pending = false;
    // A rejected client is closed by the server; keep the reason on screen.
    if (ui.connection !== 'protocol error') { ui.connection = 'disconnected'; }
    ui.source_name = '(none)';
    ui.rest_pose = 'none';
    stopStatusPoll();
    refresh();
};
client.onAck = (ack, requestId) => {
    pending = false;
    console.log(`ack[#${requestId}]: ok=${ack.ok()} "${ack.message()}"`);
    refresh();
};
client.onSourceEnded = (ev) => {
    // Stream stopped on its own (recording EOF or device lost).
    console.log(`source ended: is_error=${ev.isError()} "${ev.message()}"`);
};
client.onStatus = (st) => {
    opened = st.isStreaming();
    ui.source_name = opened
        ? `${st.sourceName()} [${st.sourceBackend()}] (${st.width()}x${st.height()})`
        : '(none)';
    ui.rest_pose = st.hasRestPose() ? 'calibrated' : 'none';
    refresh();
};
client.onPoseFrame = (frame) => {
    // A change of source changes the reference pose the rig stands on, and the two do not reach
    // the same bones, so the rig goes back to its bind pose before the new source takes over.
    const driveSource = resolveDriveSource(ui.drive_mode, frame);
    if (driveSource !== lastDriveSource) {
        resetToBindPose(bones, bindPose);
        lastDriveSource = driveSource;
    }
    ui.drive_source = driveSource;

    applyPoseFrame(bones, bindPose, frame, driveSource, ui.mirror_unmeasured);
    if (ui.show_frame_dump) {
        debugOverlayEl.textContent = formatPoseFrameDump(dumpPoseFrame(frame));
    }
    ui.frame_seq = frame.frameSeq();
};

// --- actions ------------------------------------------------------------------
function doConnect() {
    if (connected || connecting) { return; }
    connecting = true;
    ui.connection = 'connecting';
    client.connect(ui.url);
    refresh();
}
function doDisconnect() { client.disconnect(); }
function doOpen() { pending = true; client.sendStart(); refresh(); }
function doClose() { pending = true; client.sendStop(); refresh(); }
function doCalibrate() { pending = true; client.sendCalibrateRestPose(); refresh(); }
function doClearRest() { pending = true; client.sendClearRestPose(); refresh(); }

// Low-frequency ServerStatus poll (for resync; status is also pushed on every change)
let statusPollTimer = null;
function startStatusPoll() {
    stopStatusPoll();
    statusPollTimer = setInterval(() => client.sendGetServerStatus(), 2000);
}
function stopStatusPoll() {
    if (statusPollTimer !== null) { clearInterval(statusPollTimer); statusPollTimer = null; }
}

// --- gui panel ----------------------------------------------------------------
const acts = {
    connect: doConnect, 
    disconnect: doDisconnect, 
    open: doOpen, 
    close: doClose,
    calibrate: doCalibrate, 
    clearRest: doClearRest,
};

const gui = new GUI({ title: 'exo-skeleton-pose', width: 300 });
gui.add(ui, 'url').name('server url');
const cConnect = gui.add(acts, 'connect').name('Connect');
const cDisconnect = gui.add(acts, 'disconnect').name('Disconnect');
gui.add(ui, 'connection').name('status').listen().disable();

const src = gui.addFolder('Pose Stream');
const cOpen = src.add(acts, 'open').name('Start');
const cClose = src.add(acts, 'close').name('Stop');
src.add(ui, 'source_name').name('source').listen().disable();
src.add(ui, 'frame_seq').name('frame seq').listen().disable();

const rest = gui.addFolder('Rest Pose');
const cCalibrate = rest.add(acts, 'calibrate').name('Calibrate');
const cClear = rest.add(acts, 'clearRest').name('Clear');
rest.add(ui, 'rest_pose').name('state').listen().disable();

const drive = gui.addFolder('Drive');
drive.add(ui, 'drive_mode', [DRIVE_MODE_AUTO, DRIVE_SOURCE.localAnimRot, DRIVE_SOURCE.clinicalAngle])
    .name('source');
drive.add(ui, 'drive_source').name('in use').listen().disable();
drive.add(ui, 'mirror_unmeasured')
    .name('mirror unmeasured leg')
    .onChange(() => resetToBindPose(bones, bindPose));

const debug = gui.addFolder('Debug');
debug.add(ui, 'show_frame_dump')
    .name('Dump PoseFrame')
    .onChange((on) => { debugOverlayEl.style.display = on ? 'block' : 'none'; });

// Enable/disable controls to match the current connection + source state.
function refresh() {
    model.visible = opened; // only show the rig while a source is streaming
    cConnect.enable(!connected && !connecting);
    cDisconnect.enable(connected);
    cOpen.enable(connected && !opened && !pending);
    cClose.enable(connected && opened && !pending);
    cCalibrate.enable(connected && opened && !pending);
    cClear.enable(connected && opened && !pending);
}

refresh();
