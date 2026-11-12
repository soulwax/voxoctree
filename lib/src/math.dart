import 'dart:math' as math;

/// Lightweight platform-neutral 3D vector for octree partitioning and ray queries.
final class OctreeVec3 {
  final double x;
  final double y;
  final double z;

  const OctreeVec3(this.x, this.y, this.z);

  static const OctreeVec3 zero = OctreeVec3(0.0, 0.0, 0.0);

  OctreeVec3 operator +(OctreeVec3 other) =>
      OctreeVec3(x + other.x, y + other.y, z + other.z);

  OctreeVec3 operator -(OctreeVec3 other) =>
      OctreeVec3(x - other.x, y - other.y, z - other.z);

  OctreeVec3 operator -() => OctreeVec3(-x, -y, -z);

  OctreeVec3 operator *(double s) => OctreeVec3(x * s, y * s, z * s);

  double dot(OctreeVec3 other) => x * other.x + y * other.y + z * other.z;
  double get lengthSquared => x * x + y * y + z * z;
  double get length => math.sqrt(lengthSquared);

  OctreeVec3 normalized() {
    final l = length;
    if (l <= 1e-9) return const OctreeVec3(0.0, 0.0, -1.0);
    final inv = 1.0 / l;
    return OctreeVec3(x * inv, y * inv, z * inv);
  }
}

/// Axis-aligned 3D bounding box for octree nodes and entries.
final class OctreeAabb {
  final OctreeVec3 min;
  final OctreeVec3 max;

  const OctreeAabb(this.min, this.max);

  OctreeVec3 get center => OctreeVec3(
        (min.x + max.x) * 0.5,
        (min.y + max.y) * 0.5,
        (min.z + max.z) * 0.5,
      );

  OctreeVec3 get halfExtents => OctreeVec3(
        (max.x - min.x) * 0.5,
        (max.y - min.y) * 0.5,
        (max.z - min.z) * 0.5,
      );

  double get maxExtent => math.max(
        max.x - min.x,
        math.max(max.y - min.y, max.z - min.z),
      );

  bool containsAabb(OctreeAabb other) =>
      other.min.x >= min.x &&
      other.max.x <= max.x &&
      other.min.y >= min.y &&
      other.max.y <= max.y &&
      other.min.z >= min.z &&
      other.max.z <= max.z;

  bool intersectsAabb(OctreeAabb other) =>
      min.x <= other.max.x &&
      max.x >= other.min.x &&
      min.y <= other.max.y &&
      max.y >= other.min.y &&
      min.z <= other.max.z &&
      max.z >= other.min.z;

  bool intersectsSphere(OctreeVec3 center, double radius) {
    final cx = center.x.clamp(min.x, max.x);
    final cy = center.y.clamp(min.y, max.y);
    final cz = center.z.clamp(min.z, max.z);
    final dx = center.x - cx;
    final dy = center.y - cy;
    final dz = center.z - cz;
    return dx * dx + dy * dy + dz * dz <= radius * radius;
  }

  /// Branchless slab ray-AABB test returning entry distance `tEnter >= 0` or `null` if missed.
  double? intersectRay(OctreeVec3 origin, OctreeVec3 dir, {double maxDist = double.infinity}) {
    var tMin = 0.0;
    var tMax = maxDist;

    double checkAxis(double o, double d, double bMin, double bMax) {
      if (d.abs() <= 1e-12) {
        return (o < bMin || o > bMax) ? double.nan : 0.0;
      }
      final inv = 1.0 / d;
      var t1 = (bMin - o) * inv;
      var t2 = (bMax - o) * inv;
      if (t1 > t2) {
        final tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      if (t1 > tMin) tMin = t1;
      if (t2 < tMax) tMax = t2;
      return 1.0;
    }

    if (checkAxis(origin.x, dir.x, min.x, max.x).isNaN) return null;
    if (checkAxis(origin.y, dir.y, min.y, max.y).isNaN) return null;
    if (checkAxis(origin.z, dir.z, min.z, max.z).isNaN) return null;
    if (tMax < tMin || tMax < 0.0) return null;
    return math.max(0.0, tMin);
  }
}
