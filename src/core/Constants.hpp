#pragma once

#include <raylib.h>

namespace nbody::constants {
inline constexpr int windowWidth = 1280;
inline constexpr int windowHeight = 720;
inline constexpr int targetFps = 120;

inline constexpr ::Color background{10, 10, 14, 255};

inline constexpr float pickRadiusPx = 24.0F;
inline constexpr float selectThresholdSq = 9.0F;
inline constexpr float dragVelScale = 100.0F;

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

inline constexpr float gridSpacing = 1.0e7F;
inline constexpr float gridAxisEpsilon = 1e-4F;
inline constexpr float gridStepsEpsilon = 1e-6F;
inline constexpr ::Color gridColor{40, 40, 40, 255};
inline constexpr ::Color axisColor{80, 80, 80, 255};

inline constexpr int trailAlphaMin = 20;
inline constexpr int trailAlphaMax = 250;
inline constexpr float trailAlphaRange = 230.0F;

inline constexpr double seedSmallMass = 7.342e22;  // kg (Moon mass)
inline constexpr double seedCentralMass = 5.972e24;  // kg (Earth mass)
inline constexpr double seedCenterX = 0.0;
inline constexpr double seedCenterY = 0.0;
inline constexpr double seedOffsetX = 3.844e8;  // m (Earth-Moon distance)

inline constexpr float zoomWheelScale = 0.1F;
inline constexpr float minZoom = 0.05F;
inline constexpr float maxZoom = 10.0F;

inline constexpr float fixedDtMin = 1e-4F;
inline constexpr float fixedDtMax = 0.05F;
inline constexpr float timeScaleMin = 0.0F;
inline constexpr float timeScaleMax = 10.0F;
inline constexpr float gMin = 0.0F;
inline constexpr float gMax = 1e-9F;
inline constexpr float softeningMin = 0.0F;
inline constexpr float softeningMax = 1e9F;
inline constexpr float velocityCapMin = 0.0F;
inline constexpr float velocityCapMax = 1e6F;
inline constexpr int trailLengthMax = 2000;

inline constexpr float spawnMassMin = 1e20F;
inline constexpr float spawnMassMax = 1e28F;
inline constexpr float spawnVelMin = -1e4F;
inline constexpr float spawnVelMax = 1e4F;
inline constexpr float dragVelScaleMin = 1.0F;
inline constexpr float dragVelScaleMax = 1000.0F;
inline constexpr float selectedMassMin = 1e20F;
inline constexpr float selectedMassMax = 1e30F;
inline constexpr float selectedVelMin = -1e5F;
inline constexpr float selectedVelMax = 1e5F;
inline constexpr float duplicateOffsetX = 20.0F;

inline constexpr int randomColorMin = 64;
inline constexpr int randomColorMax = 255;
}  // namespace nbody::constants
