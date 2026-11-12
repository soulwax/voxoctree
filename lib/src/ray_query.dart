import 'dynamic_loose_octree.dart';
import 'math.dart';

/// Closest-hit entry record returned by [OctreeRayQuery].
final class OctreeRayHit<T> {
  final OctreeEntry<T> entry;
  final double distance;
  final OctreeVec3 point;

  const OctreeRayHit({
    required this.entry,
    required this.distance,
    required this.point,
  });
}

/// Traverses [DynamicLooseOctree] nodes intersecting a ray and returns closest or distance-sorted hits.
final class OctreeRayQuery {
  const OctreeRayQuery._();

  static List<OctreeRayHit<T>> raycastAll<T>(
    DynamicLooseOctree<T> tree, {
    required OctreeVec3 origin,
    required OctreeVec3 direction,
    double maxDistance = double.infinity,
  }) {
    final dir = direction.normalized();
    final hits = <OctreeRayHit<T>>[];
    tree.traverseRayCandidateNodes(origin, dir, maxDistance, (entries) {
      for (final e in entries) {
        final t = e.bounds.intersectRay(origin, dir, maxDist: maxDistance);
        if (t != null) {
          hits.add(
            OctreeRayHit<T>(
              entry: e,
              distance: t,
              point: origin + dir * t,
            ),
          );
        }
      }
    });
    hits.sort((a, b) => a.distance.compareTo(b.distance));
    return hits;
  }

  static OctreeRayHit<T>? raycastClosest<T>(
    DynamicLooseOctree<T> tree, {
    required OctreeVec3 origin,
    required OctreeVec3 direction,
    double maxDistance = double.infinity,
  }) {
    final all = raycastAll(
      tree,
      origin: origin,
      direction: direction,
      maxDistance: maxDistance,
    );
    return all.isEmpty ? null : all.first;
  }
}
