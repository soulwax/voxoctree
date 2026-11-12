import 'dart:math' as math;

import 'math.dart';

/// Half-space frustum plane in Hess normal form (`normal . p + distance >= 0` inside).
final class FrustumPlane {
  final OctreeVec3 normal;
  final double distance;

  const FrustumPlane({required this.normal, required this.distance});

  double signedDistance(OctreeVec3 p) => normal.dot(p) + distance;
}

/// 6-plane convex frustum visibility tester for [OctreeAabb] and spheres.
final class FrustumCullSet {
  final List<FrustumPlane> planes;

  const FrustumCullSet(this.planes);

  /// Builds an axis-aligned orthographic frustum box `[min..max]`.
  factory FrustumCullSet.orthoBox(OctreeVec3 min, OctreeVec3 max) => FrustumCullSet([
        FrustumPlane(normal: const OctreeVec3(1, 0, 0), distance: -min.x),
        FrustumPlane(normal: const OctreeVec3(-1, 0, 0), distance: max.x),
        FrustumPlane(normal: const OctreeVec3(0, 1, 0), distance: -min.y),
        FrustumPlane(normal: const OctreeVec3(0, -1, 0), distance: max.y),
        FrustumPlane(normal: const OctreeVec3(0, 0, 1), distance: -min.z),
        FrustumPlane(normal: const OctreeVec3(0, 0, -1), distance: max.z),
      ]);

  /// Builds an axis-oriented perspective frustum around eye/forward/up.
  factory FrustumCullSet.perspective({
    required OctreeVec3 eye,
    required OctreeVec3 forward,
    OctreeVec3 up = const OctreeVec3(0, 1, 0),
    double fovYRadians = 1.0,
    double aspect = 16.0 / 9.0,
    double near = 0.1,
    double far = 200.0,
  }) {
    final fwd = forward.normalized();
    var right = OctreeVec3(
      fwd.y * up.z - fwd.z * up.y,
      fwd.z * up.x - fwd.x * up.z,
      fwd.x * up.y - fwd.y * up.x,
    ).normalized();
    if (right.lengthSquared <= 1e-6) {
      right = const OctreeVec3(1, 0, 0);
    }
    final realUp = OctreeVec3(
      right.y * fwd.z - right.z * fwd.y,
      right.z * fwd.x - right.x * fwd.z,
      right.x * fwd.y - right.y * fwd.x,
    ).normalized();

    final halfV = math.tan(fovYRadians * 0.5);
    final halfH = halfV * math.max(1e-3, aspect);
    final nearCenter = eye + fwd * near;
    final farCenter = eye + fwd * far;

    FrustumPlane makePlane(OctreeVec3 pt, OctreeVec3 n) {
      final unitN = n.normalized();
      return FrustumPlane(normal: unitN, distance: -unitN.dot(pt));
    }

    // Left / right normals point inward
    final leftN = (right + fwd * halfH).normalized();
    final rightN = (-right + fwd * halfH).normalized();
    final botN = (realUp + fwd * halfV).normalized();
    final topN = (-realUp + fwd * halfV).normalized();

    return FrustumCullSet([
      makePlane(nearCenter, fwd),
      makePlane(farCenter, -fwd),
      makePlane(eye, leftN),
      makePlane(eye, rightN),
      makePlane(eye, botN),
      makePlane(eye, topN),
    ]);
  }

  bool intersectsAabb(OctreeAabb box) {
    final center = box.center;
    final half = box.halfExtents;
    for (final p in planes) {
      final projR =
          half.x * p.normal.x.abs() +
          half.y * p.normal.y.abs() +
          half.z * p.normal.z.abs();
      if (p.signedDistance(center) + projR < 0.0) {
        return false;
      }
    }
    return true;
  }

  bool intersectsSphere(OctreeVec3 center, double radius) {
    for (final p in planes) {
      if (p.signedDistance(center) < -radius) return false;
    }
    return true;
  }
}
