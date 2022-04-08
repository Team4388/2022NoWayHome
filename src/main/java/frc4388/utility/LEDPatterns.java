package frc4388.utility;

/**
 * Add your docs here.
 */
public enum LEDPatterns {
  /* PALLETTE PATTERNS */
  RAINBOW_RAINBOW(-0.99f), PARTY_RAINBOW(-0.97f), OCEAN_RAINBOW(-0.95f), LAVA_RAINBOW(-0.93f), FOREST_RAINBOW(-0.91f),
    RAINBOW_GLITTER(-0.89f), CONFETTI(-0.87f), RED_SHOT(-0.85f), BLUE_SHOT(-0.83f), WHITE_SHOT(-0.81f), RAINBOW_SINELON(-0.79f),
    PARTY_SINELON(-0.77f), OCEAN_SINELON(-0.75f), LAVA_SINELON(-0.73f), FOREST_SINELON(-0.71f), RAINBOW_BPM(-0.69f),
    PARTY_BPM(-0.67f), OCEAN_BPM(-0.65f), LAVA_BPM(-0.63f), FOREST_BPM(-0.61f), FIRE_MEDIUM(-0.59f), FIRE_LARGE(-0.57f),
    RAINBOW_TWINKLES(-0.55f), PARTY_TWINKLES(-0.53f), OCEAN_TWINKLES(-0.51f), LAVA_TWINKLES(-0.49f), FOREST_TWINKLES(-0.47f),
    RAINBOW_WAVES(-0.45f), PARTY_WAVES(-0.43f), OCEAN_WAVES(-0.41f), LAVA_WAVES(-0.39f), FOREST_WAVES(-0.37f),
    RED_SCANNER(-0.35f), GRAY_SCANNER(-0.33f), RED_CHASE(-0.31f), BLUE_CHASE(-0.29f), GRAY_CHASE(-0.27f), RED_HEARTBEAT(-0.25f),
    BLUE_HEARTBEAT(-0.23f), WHITE_HEARTBEAT(-0.21f), GRAY_HEARBEAT(-0.19f), RED_BREATH(-0.17f), BLUE_BREATH(-0.15f),
    GRAY_BREATH(-0.13f), RED_STROBE(-0.11f), BLUE_STROBE(-0.09f), GOLD_STROBE(-0.07f), WHITE_STROBE(-0.05f),
  
  /* COLOR 1 PATTERNS */
  C1_END_TO_END(-0.03f), C1_SCANNER(-0.01f), C1_CHASE(0.01f), C1_HEARTBEAT_SLOW(0.03f), C1_HEARTBEAT_MEDIUM(0.05f),
    C1_HEARTBEAT_FAST(0.07f), C1_BREATH_SLOW(0.09f), C1_BREATH_FAST(0.11f), C1_SHOT(0.13f), C1_STROBE(0.15f),
    
  /* COLOR 2 PATTERNS */
  C2_END_TO_END(0.17f), C2_SCANNER(0.19f), C2_CHASE(0.21f), C2_HEARTBEAT_SLOW(0.23f), C2_HEARTBEAT_MEDIUM(0.25f), 
    C2_HEARTBEAT_FAST(0.27f), C2_BREATH_SLOW(0.29f), C2_BREATH_FAST(0.31f), C2_SHOT(0.33f), C2_STROBE(0.35f),
    
  /* COLOR 1 AND 2 PATTERNS */
  C1C2_SPARKLE(0.37f), C2C1_SPARKLE(0.39f), C1C2_GRADIENT(0.41f), C1C2_BPM(0.43f), C1C2_BLEND(0.45f), C1C2_TWINKLES(0.51f),
    C1C2_WAVES(0.53f), C1C2_SINELON(0.55f),
     
  /* SOLID COLORS */
  SOLID_PINK_HOT(0.57f), SOLID_RED_DARK(0.59f), SOLID_RED(0.61f), SOLID_RED_ORANGE(0.63f), SOLID_ORANGE(0.65f),
    SOLID_GOLD(0.67f), SOLID_YELLOW(0.69f), SOLID_GREEN_LAWN(0.71f), SOLID_GREEN_LIME(0.73f), SOLID_GREEN_DARK(0.75f),
    SOLID_GREEN(0.77f), SOLID_BLUE_GREEN(0.79f), SOLID_BLUE_AQUA(0.81f), SOLID_BLUE_SKY(0.83f), SOLID_BLUE_DARK(0.85f),
    SOLID_BLUE(0.87f), SOLID_BLUE_VIOLET(0.89f), SOLID_VIOLET(0.91f), SOLID_WHITE(0.93f), SOLID_GRAY(0.95f),
    SOLID_GRAY_DARK(0.97f), SOLID_BLACK(0.99f);
  
  /* GETTERS/SETTERS */
  private final float id;
  LEDPatterns(float id) {
    this.id = id;
  }
  public float getValue() {
    return id;
  }

  public float percentToPWM() {
    return (1000 + (getValue() * 1000));
  }
}