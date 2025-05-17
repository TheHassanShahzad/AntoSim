// lib/camera_page.dart

import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter_mjpeg/flutter_mjpeg.dart';
import 'mqtt_service.dart';

typedef MoveCallback = void Function(double x, double y);

/// A reusable joystick control that manages its own knob state.
class JoystickWidget extends StatefulWidget {
  final double size;
  final MoveCallback onMove;
  static const String ipAddress = '192.168.0.80';
  static const String streamUrl = 'http://$ipAddress:5000/video';

  const JoystickWidget({
    Key? key,
    required this.size,
    required this.onMove,
  }) : super(key: key);

  @override
  State<JoystickWidget> createState() => _JoystickWidgetState();
}

class _JoystickWidgetState extends State<JoystickWidget> {
  Offset _knobOffset = Offset.zero;

  void _onDrag(DragUpdateDetails details) {
    final radius = widget.size / 2;
    final centre = Offset(radius, radius);
    var delta = details.localPosition - centre;
    if (delta.distance > radius) {
      delta = Offset.fromDirection(delta.direction, radius);
    }
    setState(() => _knobOffset = delta);
    final x = (delta.dx / radius).clamp(-1.0, 1.0);
    final y = (delta.dy / radius).clamp(-1.0, 1.0);
    widget.onMove(x, y);
  }

  void _onDragEnd(_) {
    setState(() => _knobOffset = Offset.zero);
    widget.onMove(0, 0);
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onPanUpdate: _onDrag,
      onPanEnd: _onDragEnd,
      child: CustomPaint(
        size: Size(widget.size, widget.size),
        painter: _JoystickPainter(_knobOffset),
      ),
    );
  }
}

class _JoystickPainter extends CustomPainter {
  final Offset knobOffset;
  _JoystickPainter(this.knobOffset);

  @override
  void paint(Canvas canvas, Size size) {
    final centre = Offset(size.width / 2, size.height / 2);
    final radius = size.width / 2;

    canvas.drawCircle(
      centre,
      radius,
      Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 4
        ..color = Colors.grey,
    );
    canvas.drawCircle(
      centre + knobOffset,
      radius / 3,
      Paint()..color = Colors.grey.shade700,
    );
  }

  @override
  bool shouldRepaint(covariant _JoystickPainter old) =>
      old.knobOffset != knobOffset;
}

/// Full-screen view with a 1s delay before mounting the MJPEG stream,
/// and exit-on-tap guard moved below the joystick.
class FullScreenCameraPage extends StatefulWidget {
  final String streamUrl;
  final MqttService mqttService;

  const FullScreenCameraPage({
    Key? key,
    required this.streamUrl,
    required this.mqttService,
  }) : super(key: key);

  @override
  State<FullScreenCameraPage> createState() => _FullScreenCameraPageState();
}

class _FullScreenCameraPageState extends State<FullScreenCameraPage> {
  bool _showStream = false;
  Timer? _delayTimer;

  @override
  void initState() {
    super.initState();
    // Delay 1000ms before showing the stream
    _delayTimer = Timer(const Duration(milliseconds: 1000), () {
      if (mounted) setState(() => _showStream = true);
    });
  }

  @override
  void dispose() {
    _delayTimer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black,
      body: SafeArea(
        child: Stack(
          children: [
            if (_showStream)
              const _CameraView(JoystickWidget.streamUrl),
            if (!_showStream)
              const Center(child: CircularProgressIndicator()),

            // 1) Exit detector now beneath the joystick
            Positioned.fill(
              child: GestureDetector(
                behavior: HitTestBehavior.opaque,
                onTap: () => Navigator.pop(context),
              ),
            ),

            // 2) Joystick on top so it can receive drags
            Positioned(
              bottom: 32,
              right: 32,
              child: JoystickWidget(
                size: 200,
                onMove: (x, y) => widget.mqttService.publishDrive(x, y),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

/// Scales the MJPEG feed to fill available space while keeping aspect ratio.
class _CameraView extends StatelessWidget {
  final String streamUrl;
  static const double nativeWidth = 2000;
  static const double nativeHeight = 1000;

  const _CameraView(this.streamUrl, {Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(builder: (ctx, constraints) {
      return Center(
        child: ConstrainedBox(
          constraints: BoxConstraints(
            maxWidth: constraints.maxWidth,
            maxHeight: constraints.maxHeight,
          ),
          child: FittedBox(
            fit: BoxFit.contain,
            alignment: Alignment.center,
            child: SizedBox(
              width: nativeWidth,
              height: nativeHeight,
              child: Mjpeg(
                stream: streamUrl,
                isLive: true,
                fit: BoxFit.fill,
              ),
            ),
          ),
        ),
      );
    });
  }
}

/// The main camera page with inline preview + joystick overlay.
class CameraPage extends StatelessWidget {
  final VoidCallback onMenuToggle;
  final bool isRailVisible;
  final MqttService mqttService;

  const CameraPage({
    Key? key,
    required this.onMenuToggle,
    required this.isRailVisible,
    required this.mqttService,
  }) : super(key: key);

  void _openFullScreen(BuildContext context) {
    Navigator.of(context).push(
      MaterialPageRoute(
        builder: (_) => FullScreenCameraPage(
          streamUrl: JoystickWidget.streamUrl,
          mqttService: mqttService,
        ),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final cs = Theme.of(context).colorScheme;
    final inlineWidth = MediaQuery.of(context).size.width * 0.8;
    final aspectRatio = _CameraView.nativeWidth / _CameraView.nativeHeight;

    return SafeArea(
      child: Column(
        children: [
          if (!isRailVisible)
            Align(
              alignment: Alignment.topLeft,
              child: IconButton(
                icon: const Icon(Icons.menu),
                color: cs.onPrimary,
                onPressed: onMenuToggle,
              ),
            ),
          const Spacer(),
          GestureDetector(
            onTap: () => _openFullScreen(context),
            child: Center(
              child: Container(
                width: inlineWidth,
                decoration: BoxDecoration(
                  color: cs.primaryContainer,
                  borderRadius: BorderRadius.circular(16),
                  border: Border.all(
                    color: cs.onPrimaryContainer,
                    width: 2,
                  ),
                ),
                clipBehavior: Clip.hardEdge,
                child: AspectRatio(
                  aspectRatio: aspectRatio,
                  child: Stack(
                    children: [
                      const _CameraView(JoystickWidget.streamUrl),
                      Positioned(
                        bottom: 8,
                        right: 8,
                        child: JoystickWidget(
                          size: inlineWidth * 0.25,
                          onMove: (x, y) => mqttService.publishDrive(x, y),
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
          ),
          const Spacer(),
        ],
      ),
    );
  }
}
