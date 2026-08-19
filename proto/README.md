# Pose protocol

Network protocol between the `exo-skeleton-pose` server (`serve` subcommand) and its clients. 
`exo_pose_proto.fbs` is the single source of truth; all bindings are generated from it with [FlatBuffers](https://flatbuffers.dev/) `flatc`.

## Transport

- WebSocket, binary frames (`ws://`, no TLS)
- Every frame is one `Message` whose `payload` union carries exactly one table.

## Message flow

| Direction | Messages |
|-----------|----------|
| Client -> Server | `Hello` (mandatory first message), `StartPoseStream`, `StopPoseStream`, `CalibrateRestPose`, `ClearRestPose`, `GetServerStatus` |
| Server -> Client | `PoseFrame` (per frame, broadcast), `ServerStatus` (after handshake / on change / on request), `SourceStreamEnded`, `Ack` (per command) |

What `StartPoseStream` opens is the server config's choice, so the command carries no arguments. 
Camera image data is intentionally not part of the protocol.

## Codegen

The C++ header is generated automatically during the CMake build. To generate
bindings for other languages from the same schema:

```
flatc --ts     -o <out> exo_pose_proto.fbs   # JavaScript / TypeScript (webview)
flatc --csharp -o <out> exo_pose_proto.fbs   # C#
```
