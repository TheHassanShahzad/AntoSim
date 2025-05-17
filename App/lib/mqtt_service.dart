// lib/mqtt_service.dart

import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';

/// A simple MQTT client for publishing GPS and drive commands.
class MqttService {
  final String brokerIp;    // e.g. "192.168.0.80"
  final String clientId;    // any unique ID
  final String gpsTopic;    // default GPS topic
  final String driveTopic;  // default drive (joystick) topic

  late final MqttServerClient client;

  MqttService({
    required this.brokerIp,
    this.clientId = 'flutter_mqtt_client',
    this.gpsTopic = 'antobot/gps',
    this.driveTopic = 'antobot/joystick',
  }) {
    client = MqttServerClient(brokerIp, clientId)
      ..port = 1883
      ..logging(on: false)
      ..keepAlivePeriod = 20
      ..onConnected = _onConnected
      ..onDisconnected = _onDisconnected
      ..onSubscribed = _onSubscribed
      ..onSubscribeFail = _onSubscribeFail
      ..pongCallback = _pong;
  }

  /// Connects to the broker. Call once before publishing.
  Future<void> connect() async {
    client.connectionMessage = MqttConnectMessage()
        .withClientIdentifier(clientId)
        .startClean()
        .withWillQos(MqttQos.atLeastOnce);
    try {
      await client.connect();
    } catch (e) {
      print('[MQTT] Connection failed: $e');
      client.disconnect();
    }
  }

  /// Publish GPS payload {"lat":..., "lon":...} to [gpsTopic].
  void publishGps(double lat, double lon, {String? topic}) {
    final t = topic ?? gpsTopic;
    if (client.connectionStatus?.state != MqttConnectionState.connected) {
      print('[MQTT] Not connected, skipping GPS publish');
      return;
    }
    final builder = MqttClientPayloadBuilder()
      ..addString('{"lat":$lat,"lon":$lon}');
    client.publishMessage(t, MqttQos.atLeastOnce, builder.payload!);
    print('[MQTT] GPS($t) → {"lat":$lat,"lon":$lon}');
  }

  /// Publish drive commands {"x":..., "y":...} to [driveTopic] or override via [topic].
  void publishDrive(double x, double y, {String? topic}) {
    final t = topic ?? driveTopic;
    if (client.connectionStatus?.state != MqttConnectionState.connected) {
      print('[MQTT] Not connected, skipping Drive publish');
      return;
    }
    final builder = MqttClientPayloadBuilder()
      ..addString('{"x":${x.toStringAsFixed(2)},'
                  '"y":${y.toStringAsFixed(2)}}');
    client.publishMessage(t, MqttQos.atLeastOnce, builder.payload!);
    print('[MQTT] Drive($t) → {"x":${x.toStringAsFixed(2)},'
          '"y":${y.toStringAsFixed(2)}}');
  }

  /// Clean disconnect
  void disconnect() {
    client.disconnect();
  }

  // ─── CALLBACKS ─────────────────────────────────────────────────────────

  void _onConnected()    => print('[MQTT] Connected');
  void _onDisconnected() => print('[MQTT] Disconnected');
  void _onSubscribed(String topic)    => print('[MQTT] Subscribed to $topic');
  void _onSubscribeFail(String topic) => print('[MQTT] Failed to subscribe $topic');
  void _pong()                        => print('[MQTT] Pong received');
}
