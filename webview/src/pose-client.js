// WebSocket client for the exo-skeleton-pose protocol. (exo_pose_proto.fbs)

import * as flatbuffers from 'flatbuffers';
import {
    Message, Payload, ProtocolVersion,
    Hello, StartPoseStream, StopPoseStream, CalibrateRestPose, ClearRestPose, GetServerStatus,
    PoseFrame, ServerStatus, SourceStreamEnded, Ack,
} from './generated/exo/proto.js';

// request_id value reserved for messages the server sends on its own notifications;
// (clients must use a non-zero id)
const RESERVED_SERVER_NOTIFY_REQ_ID = 0;

// WebSocket close code the server closes with when we break the protocol.
const CLOSE_PROTOCOL_ERROR = 1002;

export class PoseClient {
    constructor() {
        this.ws = null;

        this._nextRequestId = RESERVED_SERVER_NOTIFY_REQ_ID + 1; // stamped on each command, echoed back in the Ack
        this._helloRequestId = null; // the Hello's request id while its Ack is outstanding, else null

        // Callbacks (assign after constructing): (obj) => void
        this.onOpen = null;
        this.onReady = null; // handshake accepted; commands are allowed from here on
        this.onProtocolError = null; // (reason) => void; the server has closed the socket on us
        this.onClose = null;
        this.onPoseFrame = null;
        this.onStatus = null;
        this.onSourceEnded = null;
        this.onAck = null; // (ack, requestId) => void
    }

    connect(url) {
        this.ws = new WebSocket(url);
        this.ws.binaryType = 'arraybuffer';
        this.ws.onopen = () => {
            // Handshake first: the protocol wants Hello as the very first message. 
            // (the server refuses every command until our version matches its own)
            const b = new flatbuffers.Builder(64);
            Hello.startHello(b);
            Hello.addProtoVersion(b, ProtocolVersion.Current);
            this._helloRequestId = this._send(b, Payload.Hello, Hello.endHello(b));

            this.onOpen?.();
        };
        this.ws.onclose = (ev) => {
            // A protocol violation arrives as a close frame and nothing else, so it lands here.
            // (the server sends no payload to a client it cannot trust to read one)
            if (ev.code === CLOSE_PROTOCOL_ERROR) {
                this.onProtocolError?.(ev.reason || 'protocol error');
            }

            this._helloRequestId = null; // the socket is gone; a reconnect starts its own handshake
            this.onClose?.();
        };
        this.ws.onerror = () => this.ws?.close();
        this.ws.onmessage = (ev) => this._decode(new Uint8Array(ev.data));
    }

    disconnect() {
        this.ws?.close();
    }

    // --- commands (client -> server) ---------------------------------------------

    sendStart() {
        const b = new flatbuffers.Builder(64);
        StartPoseStream.startStartPoseStream(b);
        this._send(b, Payload.StartPoseStream, StartPoseStream.endStartPoseStream(b));
    }

    sendStop() {
        const b = new flatbuffers.Builder(64);
        StopPoseStream.startStopPoseStream(b);
        this._send(b, Payload.StopPoseStream, StopPoseStream.endStopPoseStream(b));
    }

    sendCalibrateRestPose() {
        const b = new flatbuffers.Builder(64);
        CalibrateRestPose.startCalibrateRestPose(b);
        this._send(b, Payload.CalibrateRestPose, CalibrateRestPose.endCalibrateRestPose(b));
    }

    sendClearRestPose() {
        const b = new flatbuffers.Builder(64);
        ClearRestPose.startClearRestPose(b);
        this._send(b, Payload.ClearRestPose, ClearRestPose.endClearRestPose(b));
    }

    // Request the current status; the server replies with a ServerStatus. (no Ack)
    sendGetServerStatus() {
        const b = new flatbuffers.Builder(64);
        GetServerStatus.startGetServerStatus(b);
        this._send(b, Payload.GetServerStatus, GetServerStatus.endGetServerStatus(b));
    }

    // --- internals ----------------------------------------------------------------

    _send(b, payloadType, payloadOffset) {
        const requestId = this._nextRequestId++;
        Message.startMessage(b);
        Message.addPayloadType(b, payloadType);
        Message.addPayload(b, payloadOffset);
        Message.addRequestId(b, requestId);
        b.finish(Message.endMessage(b));
        if (this.ws?.readyState === WebSocket.OPEN) { this.ws.send(b.asUint8Array()); }
        return requestId;
    }

    _decode(bytes) {
        const msg = Message.getRootAsMessage(new flatbuffers.ByteBuffer(bytes));
        const requestId = msg.requestId();
        switch (msg.payloadType()) {
            case Payload.PoseFrame:    this.onPoseFrame?.(msg.payload(new PoseFrame())); break;
            case Payload.ServerStatus: this.onStatus?.(msg.payload(new ServerStatus())); break;
            case Payload.SourceStreamEnded: this.onSourceEnded?.(msg.payload(new SourceStreamEnded())); break;
            case Payload.Ack:          this._handleAck(msg.payload(new Ack()), requestId); break;
            default: break;
        }
    }

    _handleAck(ack, requestId) {
        const isHandshakeAck = (requestId === this._helloRequestId);
        if (isHandshakeAck) {
            this._helloRequestId = null; // the handshake is settled; from here on every Ack belongs to a caller's command
            this.onReady?.();
            return;
        }

        this.onAck?.(ack, requestId);
    }
}
