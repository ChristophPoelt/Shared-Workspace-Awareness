import math
import numpy as np


class ClockwiseRail:
    def __init__(self, length_long=3.43, length_short=0.992, radius=0.279, zero_offset=0.8):
        """
        Rail-Modell (Rechteck mit runden Ecken), im Uhrzeigersinn (CW).

        Koordinatensystem (Bounding Box):
        - (0,0) ist die Ecke oben links (Bounding Box des gesamten Rails).
        - X wird nach RECHTS größer.
        - Y wird nach UNTEN größer.

        Der "interne" Pfad startet (d=0) am Anfang der oberen Geraden (nach dem TL-Bogen).
        Der "User"-Nullpunkt ist um 'zero_offset' auf der oberen Geraden verschoben.
        """
        self.lx = length_long
        self.ly = length_short
        self.r = radius
        self.offset = zero_offset

        # Länge eines Viertelkreises
        self.len_corner = (math.pi * self.r) / 2.0

        # Gesamtlänge der Runde
        self.total_length = 2 * self.lx + 2 * self.ly + 4 * self.len_corner

    def _normalize(self, user_pos):
        """
        Wandelt User-Position (beliebiger Float) in interne Pfad-Position um [0, total_length].
        """
        # 1. Offset verrechnen (User 0.0 -> Intern 0.8)
        d = user_pos + self.offset

        # 2. Modulo-Operator für Normalisierung (fängt negative Werte & Überlänge ab)
        return d % self.total_length

    def _get_pose_internal(self, d):
        """
        Interne Berechnung: Gibt (x, y, yaw) für eine normalisierte Distanz 'd' zurück.
        """
        # --- 1. Segment: Obere Gerade (nach Rechts) ---
        if d < self.lx:
            return (self.r + d), 0.0, 0.0  # Yaw 0 = Rechts
        d -= self.lx

        # --- 2. Segment: Kurve Oben-Rechts (Rechts -> Unten) ---
        if d < self.len_corner:
            theta = d / self.r
            cx = self.r + self.lx
            cy = self.r
            return (cx + self.r * math.sin(theta)), (cy - self.r * math.cos(theta)), theta
        d -= self.len_corner

        # --- 3. Segment: Rechte Gerade (nach Unten) ---
        if d < self.ly:
            return (self.r + self.lx + self.r), (self.r + d), (math.pi / 2)
        d -= self.ly

        # --- 4. Segment: Kurve Unten-Rechts (Unten -> Links) ---
        if d < self.len_corner:
            theta = d / self.r
            cx = self.r + self.lx
            cy = self.r + self.ly
            return (cx + self.r * math.cos(theta)), (cy + self.r * math.sin(theta)), ((math.pi / 2) + theta)
        d -= self.len_corner

        # --- 5. Segment: Untere Gerade (nach Links) ---
        if d < self.lx:
            return ((self.r + self.lx) - d), (self.r + self.ly + self.r), math.pi
        d -= self.lx

        # --- 6. Segment: Kurve Unten-Links (Links -> Oben) ---
        if d < self.len_corner:
            theta = d / self.r
            cx = self.r
            cy = self.r + self.ly
            return (cx - self.r * math.sin(theta)), (cy + self.r * math.cos(theta)), (math.pi + theta)
        d -= self.len_corner

        # --- 7. Segment: Linke Gerade (nach Oben) ---
        if d < self.ly:
            return 0.0, ((self.r + self.ly) - d), (3 * math.pi / 2)
        d -= self.ly

        # --- 8. Segment: Kurve Oben-Links (Oben -> Rechts) ---
        theta = d / self.r
        cx = self.r
        cy = self.r
        return (cx - self.r * math.cos(theta)), (cy - self.r * math.sin(theta)), ((3 * math.pi / 2) + theta)

    def calculate_yaw_to_target(self, current_pos_raw, target_pos_raw):
        """
        HAUPTFUNKTION:
        Berechnet den Winkel relativ zur Schiene (in den Kurven werden die Tangenten als Grundlage verwendet)
        zwischen Schiene und Verbindungslinie (Carriage -> Goal)

        Args:
            current_pos_raw: Aktuelle Position (beliebiger Wert)
            target_pos_raw: Zielposition (beliebiger Wert)

        Returns:
            float: Winkel in Radiants [-pi, pi].
                   0.0 = Ziel ist genau "vorne" in Fahrtrichtung.
        """
        # 1. Normalisieren und Koordinaten berechnen
        d_curr = self._normalize(current_pos_raw)
        d_targ = self._normalize(target_pos_raw)

        px, py, rail_yaw = self._get_pose_internal(d_curr)
        tx, ty, _ = self._get_pose_internal(d_targ)

        # 2. Vektor zum Ziel (Global)
        dx = tx - px
        dy = ty - py

        # Distanz prüfen (Selbst-Referenz vermeiden)
        if math.sqrt(dx * dx + dy * dy) < 0.001:
            return 0.0

        # 3. Globalen Winkel zum Ziel berechnen
        # (atan2 kommt mit Y-Down zurecht, solange rail_yaw im selben System ist)
        global_target_angle = math.atan2(dy, dx)

        # 4. Differenz zur aktuellen Fahrtrichtung
        angle_diff = global_target_angle - rail_yaw

        # 5. Normalisieren auf [-pi, pi] für kürzesten Drehweg
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        return angle_diff


rail = ClockwiseRail(length_long=3.43, length_short=0.992, radius=0.279, zero_offset=0.8)

current_pos = 4.28
target_pos = 4.50

angle_needed = rail.calculate_yaw_to_target(current_pos, target_pos)
angle_needed_deg = math.degrees(angle_needed)

print(f"Roboter muss sich um {angle_needed:.4f} rad drehen.")
print(f"                     {angle_needed_deg:.2f} Grad")
