#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace Hexapod::Config {

    // --- Geometrie (mm) ---
    inline constexpr float COXA_LEN    = 56.0f;
    inline constexpr float FEMUR_LEN   = 70.0f;
    inline constexpr float TIBIA_LEN   = 120.0f;
    inline constexpr float COXA_OFFSET = 0.0f;

    // --- Gang-Parameter ---
    inline constexpr float STANDING_HEIGHT = -80.0f;
    inline constexpr float STANCE_RADIUS   = 130.0f;
    inline constexpr float STEP_HEIGHT     = 25.0f;
    inline constexpr float CYCLE_TIME      = 1.0f;
    inline constexpr float DUTY_FACTOR     = 0.5f;

    inline constexpr uint8_t NUM_LEGS   = 6;
    inline constexpr uint8_t NUM_SERVOS = 18;

    struct Mount {
        float x;
        float y;
        float z;
        float yaw_deg;
    };

    // Mount-Geometrie: 6 Beine exakt 60 Grad versetzt (echt hexagonale Anordnung,
    // keine Front/Mitte/Heck-Rechteckform). yaw_deg = Ausrichtung der Coxa-Nulllage
    // relativ zur Rumpf-Vorwaerts-Achse (+X), gemessen ab Rumpfzentrum; x/y liegen
    // exakt auf dieser Achse (radiale Montage) - Voraussetzung dafuer, dass
    // transform_to_leg_frame() pro Bein ein korrektes Ziel berechnet.
    //
    // Radius pro Beinpaar aus der bisherigen Konfiguration uebernommen
    // (Front/Heck ~77.78mm, Mitte ~65mm) - falls das nicht exakt der echten
    // Montage entspricht, hier anpassen (Radius = Abstand Rumpfzentrum -> Coxa-Achse).
    inline constexpr std::array<Mount, 6> MOUNTS{{
        {  67.36f, -38.89f, 0.0f,  -30.0f}, // Idx 0 (VR - Bein 1)
        {  67.36f,  38.89f, 0.0f,   30.0f}, // Idx 1 (VL - Bein 2)
        {   0.00f,  65.00f, 0.0f,   90.0f}, // Idx 2 (ML - Bein 3)
        { -67.36f,  38.89f, 0.0f,  150.0f}, // Idx 3 (HL - Bein 4)
        { -67.36f, -38.89f, 0.0f, -150.0f}, // Idx 4 (HR - Bein 5)
        {   0.00f, -65.00f, 0.0f,  -90.0f}  // Idx 5 (MR - Bein 6)
    }};

    inline constexpr std::array<uint8_t, 6> PHASE_GROUPS{0, 1, 0, 1, 0, 1};

    // Drehrichtungen. DIR_COXA={1,1,1,1,1,1} (alle einheitlich) ist das Ergebnis
    // realer Tests auf der Hardware - NICHT rein geometrisch hergeleitet. Eine
    // theoretische Annahme (Vorzeichen muesste sich zwischen linker/rechter
    // Koerperseite spiegeln) war fuer diese Verkabelung falsch: alle 6 Coxa-
    // Servos reagieren einheitlich auf dieselbe Kommandorichtung. Bei zukuenftigen
    // Aenderungen an MOUNTS/Geometrie lieber wieder empirisch pruefen (Papier +
    // gerade Linie, siehe Bugfix-Historie) statt das Vorzeichen neu herzuleiten.
    inline constexpr std::array<float, 6> DIR_COXA {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    inline constexpr std::array<float, 6> DIR_FEMUR{-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};
    inline constexpr std::array<float, 6> DIR_TIBIA{-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};

    // Huft-Basen: mit korrekter 60-Grad-Geometrie + world-frame-Transform braucht
    // es das manuelle "Zusammenziehen" der aeusseren Huftservos aus der alten
    // Winkelversion nicht mehr - COXA_WALK_DEG testweise = COXA_REST_DEG gesetzt.
    // Falls beim Testen doch wieder Bein-gegen-Bein-Verhalten auftritt, war die
    // alte Kompensation NICHT nur ein Workaround fuer die alte Logik - bitte
    // rueckmelden statt selbst wieder reinzumischen (sonst wirkt's ggf. doppelt).
    inline constexpr std::array<float, 6> COXA_REST_DEG{135.0f, 135.0f, 135.0f, 135.0f, 135.0f, 135.0f};
    inline constexpr std::array<float, 6> COXA_WALK_DEG{135.0f, 135.0f, 135.0f, 135.0f, 135.0f, 135.0f};

    // Servo-Ruhelage (0-270 Grad Servospanne) fuer Femur/Tibia; Coxa-Ruhelage ist
    // beinabhaengig und kommt aus COXA_REST_DEG/COXA_WALK_DEG.
    inline constexpr float FEMUR_REST_DEG = 135.0f;
    inline constexpr float TIBIA_REST_DEG = 135.0f;

    // --- Mechanische Coxa-Nullpunkt-Kalibrierung ---
    // WICHTIG: transform_to_leg_frame() nimmt an, dass IK-Winkel=0 fuer die Coxa
    // exakt in Richtung MOUNTS[i].yaw_deg zeigt. Stimmt das real montierte Servo-
    // horn nicht exakt mit dieser Annahme ueberein (z.B. um wenige Grad verdreht
    // aufgesteckt), rechnet die IK zwar korrekt, aber das tatsaechliche Bein zeigt
    // in eine leicht andere Richtung als angenommen - das fuehrt beim Stehen kaum
    // auf, sorgt beim Laufen aber dafuer, dass jedes Bein einen anderen effektiven
    // Nullpunkt hat und die Beine gegeneinander arbeiten.
    //
    // Diese Werte sind eine rein mechanische Korrektur (in Servo-Grad, additiv,
    // NICHT Teil der IK-Geometrie) und werden ueber den "calzero"-Befehl in
    // main.cpp ermittelt: dabei faehrt jedes Bein einzeln in IK-Nullstellung
    // (coxa_rad=0), und man misst/beobachtet die Abweichung von der erwarteten
    // MOUNTS[i].yaw_deg-Richtung. Default 0.0 = keine Korrektur (erstmal messen!).
    inline constexpr std::array<float, 6> COXA_ZERO_OFFSET_DEG{0.0f, 0.0f, 0.0f, -18.0f, 5.0f, 0.0f};

    // Gleiche Idee wie COXA_ZERO_OFFSET_DEG, aber fuer die Tibia: additive
    // Servo-Grad-Korrektur pro Bein (Index 0=VR..5=MR), falls das Tibia-Servohorn
    // nicht exakt bei 135 Grad (TIBIA_REST_DEG) montiert ist. Optisch/manuell
    // ermitteln: Bein einzeln in Ruhelage fahren (z.B. per "stand"), pruefen ob
    // der Unterschenkel bei allen Beinen gleich steht, Abweichung in Servo-Grad
    // eintragen (positiv = Servo dreht weiter auf, siehe eigene Vorzeichenrichtung
    // pruefen: Wert eintragen, testen, bei falscher Richtung Vorzeichen umdrehen).
    inline constexpr std::array<float, 6> TIBIA_ZERO_OFFSET_DEG{8.0f, 9.0f, 8.0f, -5.0f, 10.0f, 10.0f};

    // Gleiche Idee, aber fuer die Femur - dritter und letzter Kalibrierungswert
    // pro Bein. Messung wie bei Tibia: 'stand', Winkelmesser an den Femur (Ober-
    // schenkel) anlegen statt an die Tibia, Median als Referenz, Differenz eintragen.
    inline constexpr std::array<float, 6> FEMUR_ZERO_OFFSET_DEG{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // --- Body-Pose / Leveling (fuer spaetere ToF/IMU-Integration) ---
    inline constexpr float MAX_BODY_TILT_DEG = 20.0f;

    // --- Sicherheits-Slew-Limiter (ServoDriver) ---
    // Verhindert, dass EIN EINZELNER Befehl (egal von wo - Boot-Init, Kalibrierung,
    // oder ein Bug im Zustandsautomat) einen Servo abrupt mit voller Kraft an eine
    // weit entfernte Zielposition springen laesst. Grund: Beim Reset (z.B. Boot-
    // Button zum Flashen) ist die tatsaechliche physische Beinstellung unbekannt
    // (koennte z.B. noch 'Blume' sein), waehrend der Code sofort eine ganz andere
    // IK-Zielpose berechnet - ohne Limiter wuerde der Servo angewiesen, den vollen
    // Weg SOFORT mit maximaler Kraft zurueckzulegen: reales mechanisches Risiko
    // (Schalter gebrochen, 25A-Sicherung ausgeloest - siehe Vorfall).
    //
    // MAX_SLEW_DEG_PER_SEC: maximale erlaubte Aenderungsrate in Servo-Grad/Sekunde,
    // global fuer alle 18 Kanaele, deutlich unter der Maximalgeschwindigkeit der
    // 45kg-Servos - ein kompletter 0-270-Grad-Sprung dauert damit >1s statt <0.2s.
    inline constexpr float MAX_SLEW_DEG_PER_SEC = 250.0f;

    // Deckelt das "Zeitguthaben" der Slew-Berechnung. Ohne diesen Deckel wuerde
    // ein lange nicht angesprochener Kanal (z.B. der allererste Befehl nach dem
    // Booten) faelschlich ein grosses Bewegungsbudget "aufgespart" haben und
    // trotzdem in einem Schritt weit springen koennen. Mit 150ms darf ein
    // einzelner set_command_angle()-Aufruf hoechstens MAX_SLEW_DEG_PER_SEC*0.15s
    // bewegen, egal wie lange der Kanal vorher inaktiv war.
    inline constexpr float MAX_SLEW_IDLE_CREDIT_MS = 150.0f;

    // --- Lenktrimmung ---
    // Kleine konstante Rotationsrate (Grad/Sekunde), die beim Geradeauslaufen
    // zusaetzlich zur Translation mitgegeben wird - gleiches Prinzip wie die
    // Trimmung bei RC-Fahrzeugen/Flugzeugen. Faengt kleinen Restversatz ab, der
    // rein mechanisch bedingt ist (ungleicher Fussdruck/Reibung, Servo-Toleranzen)
    // und sich nicht weiter durch Kalibrierung wegbekommen laesst.
    //
    // Vorzeichen/Betrag rein empirisch ermitteln: Roboter eine definierte Strecke
    // geradeaus laufen lassen, seitlichen Versatz messen, hier in kleinen Schritten
    // (z.B. 0.1-0.3 Grad/Sekunde) in die Richtung anpassen, die den Versatz
    // verringert. 0.0 = keine Korrektur (Default).
    inline constexpr float STEERING_TRIM_DEG_PER_SEC = 0.2f;

    // --- Richtungslaufen & Drehen ---
    // Betrag der Schrittweite (mm) fuer vorwaerts/rueckwaerts/seitwaerts -
    // ersetzt die vorher direkt in main.cpp hartkodierte 35.0f.
    inline constexpr float WALK_STRIDE_MM = 35.0f;

    // Nominale Drehrate (Grad/Sekunde), NUR zur Berechnung, wie viele volle
    // Gangzyklen ein 'turn <winkel>'-Befehl braucht - die tatsaechlich
    // verwendete Rate wird danach so nachjustiert, dass der Zielwinkel exakt
    // nach einer GANZEN Zahl Zyklen erreicht wird (Fuesse landen wieder genau
    // in Phase 0, kein "krummer" Abbruch mitten im Schritt).
    inline constexpr float TURN_SPEED_DEG_PER_SEC = 30.0f;

    // Empirischer Korrekturfaktor: tatsaechlich erreichter Drehwinkel geteilt
    // durch angeforderten Winkel, gemessen auf echter Hardware. Drehen auf dem
    // Untergrund ueberschiesst (vermutlich Schwung/Traegheit), und zwar NICHT
    // exakt linear mit der Winkelgroesse - dieser Wert ist iterativ eingemessen
    // (letzte Anpassung: 1.6 -> 1.5, weil 'turn 180' bei 1.6 gemessen um ca.
    // 10 Grad zu wenig drehte). Weiter einmessen: 'turn <winkel>' mit einem
    // grossen Blatt Papier + Winkelmarkierung unterm Roboter testen, echten
    // Winkel messen, Faktor = erreichter_winkel / angeforderter_winkel neu
    // setzen, wiederholen. Falls kleine (z.B. 45 Grad) und grosse Winkel (z.B.
    // 180 Grad) weiterhin deutlich unterschiedlich abweichen, reicht ein
    // einzelner linearer Faktor nicht mehr aus - bitte rueckmelden statt nur
    // den Wert weiter zu verschieben (dann brauchts eine Korrekturtabelle
    // statt einer einzelnen Konstante).
    inline constexpr float TURN_OVERSHOOT_FACTOR = 1.5f;

} // namespace Hexapod::Config