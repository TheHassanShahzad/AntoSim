import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';

class MyTheme {
  static ThemeData get theme {
    // Build the base theme to access its colorScheme
    final base = ThemeData(
      useMaterial3: true,
      colorScheme: ColorScheme.fromSeed(
        seedColor: Colors.lightGreen,
        brightness: Brightness.light,
      ),
    );

    return base.copyWith(
      textTheme: TextTheme(
        displayLarge: GoogleFonts.robotoFlex(
          fontSize: 72,
          fontWeight: FontWeight.bold,
        ),
        titleLarge: GoogleFonts.robotoFlex(
          fontSize: 24,
          fontWeight: FontWeight.bold,
        ),
        bodyMedium: GoogleFonts.merriweather(),
        displaySmall: GoogleFonts.shareTechMono(
          fontSize:14,
          fontStyle: FontStyle.italic
        ),
      ).apply(
        bodyColor: base.colorScheme.onSurface, // applies to most text
        displayColor: base.colorScheme.onSurface, // applies to large styles like headlines
      ),
    );
  }
}
