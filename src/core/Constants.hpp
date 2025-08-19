#pragma once

#include <raylib.h>

namespace nbody::constants {
inline constexpr int windowWidth = 1280;
inline constexpr int windowHeight = 720;
inline constexpr int targetFps = 120;

inline constexpr ::Color background{10, 10, 14, 255};

inline constexpr float pickRadiusPx = 24.0F;
inline constexpr float selectThresholdSq = 9.0F;
inline constexpr float dragVelScale = 0.01F;

inline constexpr float dragLineWidth = 2.0F;
inline constexpr float dragCircleRadius = 3.0F;
inline constexpr float ringExtraRadius = 4.0F;
inline constexpr float ringThickness = 3.5F;
inline constexpr float ringInnerOffset = 2.0F;
inline constexpr float ringStartAngle = 0.0F;
inline constexpr float ringEndAngle = 360.0F;
inline constexpr int ringSegments = 32;
inline constexpr float velLineWidth = 1.5F;
inline constexpr float accLineWidth = 1.0F;
inline constexpr float velVectorScale = 10.0F;
inline constexpr float accVectorScale = 500.0F;
inline constexpr float minBodyRadius = 6.0F;
inline constexpr float selectedCircleAlpha = 0.5F;

inline constexpr float gridSpacing = 50.0F;
inline constexpr float gridAxisEpsilon = 1e-4F;
inline constexpr float gridStepsEpsilon = 1e-6F;
inline constexpr ::Color gridColor{40, 40, 40, 255};
inline constexpr ::Color axisColor{80, 80, 80, 255};

inline constexpr int trailAlphaMin = 20;
inline constexpr int trailAlphaMax = 250;
inline constexpr float trailAlphaRange = 230.0F;

inline constexpr float seedSmallMass = 12.0F;
inline constexpr float seedCentralMass = 4000.0F;
inline constexpr float seedSpeed = 1.20F;
inline constexpr float seedCenterX = 640.0F;
inline constexpr float seedCenterY = 360.0F;
inline constexpr float seedOffsetX = 200.0F;

inline constexpr float zoomWheelScale = 0.1F;
inline constexpr float minZoom = 0.05F;
inline constexpr float maxZoom = 10.0F;

inline constexpr float fixedDtMin = 1e-4F;
inline constexpr float fixedDtMax = 0.05F;
inline constexpr float timeScaleMin = 0.0F;
inline constexpr float timeScaleMax = 10.0F;
inline constexpr float gMin = 0.0F;
inline constexpr float gMax = 0.02F;
inline constexpr float softeningMin = 0.0F;
inline constexpr float softeningMax = 20.0F;
inline constexpr float velocityCapMin = 0.0F;
inline constexpr float velocityCapMax = 200.0F;
inline constexpr int trailLengthMax = 2000;

inline constexpr float spawnMassMin = 1.0F;
inline constexpr float spawnMassMax = 5000.0F;
inline constexpr float spawnVelMin = -5.0F;
inline constexpr float spawnVelMax = 5.0F;
inline constexpr float dragVelScaleMin = 0.001F;
inline constexpr float dragVelScaleMax = 0.2F;
inline constexpr float selectedMassMin = 1.0F;
inline constexpr float selectedMassMax = 10000.0F;
inline constexpr float selectedVelMin = -200.0F;
inline constexpr float selectedVelMax = 200.0F;
inline constexpr float duplicateOffsetX = 20.0F;

inline constexpr int randomColorMin = 64;
inline constexpr int randomColorMax = 255;
}  // namespace nbody::constants
