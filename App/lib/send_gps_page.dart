import 'package:flutter/material.dart';
import 'package:geolocator/geolocator.dart';

/// Page for sending GPS data via MQTT to a selected device.
/// Only handles UI; the parent manages the stream & publishing.
class SendGpsPage extends StatefulWidget {
  final bool isSending;               // Currently sending?
  final Position? currentPosition;    // Latest GPS fix
  final VoidCallback onToggle;
  final VoidCallback onMenuToggle;
  final bool isRailVisible;

  const SendGpsPage({
    super.key,
    required this.isSending,
    required this.currentPosition,
    required this.onToggle,
    required this.onMenuToggle,
    required this.isRailVisible,
  });

  @override
  State<SendGpsPage> createState() => _SendGpsPageState();
}

class _SendGpsPageState extends State<SendGpsPage> {
  String _selectedDevice = 'Antobot'; // <-- now stateful

  @override
  Widget build(BuildContext context) {
    final theme = Theme.of(context);
    final cs = theme.colorScheme;
    final coords = widget.currentPosition != null
        ? ' (${widget.currentPosition!.latitude.toStringAsFixed(5)}, '
          '${widget.currentPosition!.longitude.toStringAsFixed(5)})'
        : ' (waiting...)';

    return SafeArea(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.center,
        children: [
          // ☰ Menu icon when rail hidden
          if (!widget.isRailVisible)
            Align(
              alignment: Alignment.topLeft,
              child: IconButton(
                icon: const Icon(Icons.menu),
                color: cs.onPrimary,
                onPressed: widget.onMenuToggle,
              ),
            ),

          // 🔽 Device selector dropdown
          DropdownButton<String>(
            value: _selectedDevice,            // <-- use state here
            dropdownColor: cs.primaryContainer,
            style: theme.textTheme.bodyMedium!
                .copyWith(color: cs.onPrimary, fontSize: 18),
            items: ['Antobot', 'Microcontroller'].map((value) {
              return DropdownMenuItem<String>(
                value: value,
                child: Text(
                  value,
                  style: theme.textTheme.bodyMedium!
                      .copyWith(color: cs.onPrimaryContainer, fontSize: 18),
                ),
              );
            }).toList(),
            selectedItemBuilder: (_) => ['Antobot', 'Microcontroller']
                .map((value) {
              return Text(
                value,
                style: theme.textTheme.bodyMedium!
                    .copyWith(color: cs.onPrimary, fontSize: 18),
              );
            }).toList(),
            onChanged: (newValue) {
              if (newValue != null) {
                setState(() {
                  _selectedDevice = newValue; // <-- update state
                });
              }
            },
          ),

          const Spacer(),

          // 📍 GPS toggle button
          ElevatedButton(
            onPressed: widget.onToggle,
            style: ElevatedButton.styleFrom(
              padding: const EdgeInsets.all(20),
              shape: RoundedRectangleBorder(
                borderRadius: BorderRadius.circular(100),
              ),
            ),
            child: Text(
              widget.isSending ? "Sending GPS Data..." : "Click to send GPS",
              style: theme.textTheme.titleLarge,
            ),
          ),

          const SizedBox(height: 20),

          // 📡 Status + coords
          Text(
            widget.isSending
                ? "Sending GPS Data via MQTT to $_selectedDevice$coords"
                : "",
            style: theme.textTheme.displaySmall!
                .copyWith(color: cs.onPrimary),
            textAlign: TextAlign.center,
          ),

          const Spacer(),
        ],
      ),
    );
  }
}
