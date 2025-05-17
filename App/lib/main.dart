
import 'dart:async';
import 'package:flutter/material.dart';
import 'package:geolocator/geolocator.dart';
import 'theme.dart';
import 'mqtt_service.dart';
import 'send_gps_page.dart';
import 'camera_page.dart';

void main() {
  runApp(const MyApp());
}

/// Root of the app
class MyApp extends StatelessWidget {
  const MyApp({super.key});
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Antobot Demo',
      theme: MyTheme.theme,
      home: const HomeShell(),
    );
  }
}

/// Main shell with NavigationRail and shared MQTT/GPS logic
class HomeShell extends StatefulWidget {
  const HomeShell({super.key});
  @override
  State<HomeShell> createState() => _HomeShellState();
}

class _HomeShellState extends State<HomeShell> {
  int selectedIndex = 0;
  bool isSending = false;
  bool isRailVisible = true;

  Position? _currentPosition;
  late final MqttService _mqttService;
  StreamSubscription<Position>? _positionSub;

  @override
  void initState() {
    super.initState();

    // 1) Initialize and connect MQTT
    _mqttService = MqttService(brokerIp: JoystickWidget.ipAddress);
    _mqttService.connect();

    // 2) Start GPS stream
    _initLocationStream();
  }

  Future<void> _initLocationStream() async {
    if (!await Geolocator.isLocationServiceEnabled()) return;
    var perm = await Geolocator.checkPermission();
    if (perm == LocationPermission.denied) {
      perm = await Geolocator.requestPermission();
      if (perm == LocationPermission.denied) return;
    }
    if (perm == LocationPermission.deniedForever) return;

    const settings = LocationSettings(
      accuracy: LocationAccuracy.high,
      distanceFilter: 0,
    );
    _positionSub = Geolocator
        .getPositionStream(locationSettings: settings)
        .listen((pos) {
      setState(() => _currentPosition = pos);
      if (isSending) {
        _mqttService.publishGps(pos.latitude, pos.longitude);
      }
    });
  }

  @override
  void dispose() {
    _positionSub?.cancel();
    _mqttService.disconnect();
    super.dispose();
  }

  void toggleSending() => setState(() => isSending = !isSending);
  void toggleRailVisibility() => setState(() => isRailVisible = !isRailVisible);
  void _onDestinationSelected(int index) => setState(() => selectedIndex = index);

  Widget _buildPage() {
    switch (selectedIndex) {
      case 0:
        return SendGpsPage(
          isSending: isSending,
          currentPosition: _currentPosition,
          onToggle: toggleSending,
          onMenuToggle: toggleRailVisibility,
          isRailVisible: isRailVisible,
        );
      case 1:
        return CameraPage(
          onMenuToggle: toggleRailVisibility,
          isRailVisible: isRailVisible,
          mqttService: _mqttService,
        );
      default:
        return const Center(child: Text('Page not found'));
    }
  }

  @override
  Widget build(BuildContext context) {
    final cs = Theme.of(context).colorScheme;

    return Scaffold(
      backgroundColor: cs.primary,
      body: Row(
        children: [
          if (isRailVisible)
            Container(
              color: cs.primaryContainer,
              child: Column(
                children: [
                  SafeArea(
                    child: IconButton(
                      icon: const Icon(Icons.menu),
                      color: cs.onPrimaryContainer,
                      onPressed: toggleRailVisibility,
                    ),
                  ),
                  Expanded(
                    child: NavigationRail(
                      selectedIndex: selectedIndex,
                      onDestinationSelected: _onDestinationSelected,
                      backgroundColor: Colors.transparent,
                      destinations: const [
                        NavigationRailDestination(
                          icon: Icon(Icons.gps_fixed),
                          label: Text('Send GPS'),
                        ),
                        NavigationRailDestination(
                          icon: Icon(Icons.camera_alt_outlined),
                          label: Text('Camera'),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          const VerticalDivider(width: 1),
          Expanded(child: _buildPage()),
        ],
      ),
    );
  }
}
