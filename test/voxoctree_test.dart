import 'package:test/test.dart';
import 'package:voxoctree/voxoctree.dart';

void main() {
  test('DynamicLooseOctree subdivides, updates bounds dynamically & queries AABB regions', () {
    final tree = DynamicLooseOctree<String>(halfExtent: 64.0, splitThreshold: 4, maxDepth: 4);

    for (var i = 0; i < 10; i++) {
      tree.insertOrUpdate(
        i,
        'Obj_$i',
        OctreeAabb(
          OctreeVec3(i * 4.0, 0.0, 0.0),
          OctreeVec3(i * 4.0 + 1.0, 1.0, 1.0),
        ),
      );
    }

    expect(tree.count, 10);
    expect(tree.nodeCount, greaterThan(1));

    // Move Obj_0 from x=0 to x=50 and confirm query updates accurately
    tree.insertOrUpdate(
      0,
      'Obj_0',
      const OctreeAabb(OctreeVec3(50.0, 0.0, 0.0), OctreeVec3(51.0, 1.0, 1.0)),
    );

    final nearOrigin = tree.queryAabb(
      const OctreeAabb(OctreeVec3(-1.0, -1.0, -1.0), OctreeVec3(2.0, 2.0, 2.0)),
    );
    expect(nearOrigin, isEmpty);

    final near50 = tree.queryAabb(
      const OctreeAabb(OctreeVec3(49.0, -1.0, -1.0), OctreeVec3(52.0, 2.0, 2.0)),
    );
    expect(near50.map((e) => e.item), ['Obj_0']);
  });

  test('FrustumCullSet filters visible entries inside camera view frustum', () {
    final tree = DynamicLooseOctree<String>(halfExtent: 64.0);
    tree.insertOrUpdate(
      1,
      'InFront',
      const OctreeAabb(OctreeVec3(-1.0, -1.0, -12.0), OctreeVec3(1.0, 1.0, -10.0)),
    );
    tree.insertOrUpdate(
      2,
      'Behind',
      const OctreeAabb(OctreeVec3(-1.0, -1.0, 10.0), OctreeVec3(1.0, 1.0, 12.0)),
    );

    final frustum = FrustumCullSet.perspective(
      eye: OctreeVec3.zero,
      forward: const OctreeVec3(0.0, 0.0, -1.0),
      fovYRadians: 1.0,
      aspect: 1.5,
      near: 0.5,
      far: 50.0,
    );

    final visible = tree.queryFrustum(frustum);
    expect(visible.map((e) => e.item), ['InFront']);
  });

  test('OctreeRayQuery returns distance-sorted slab raycast hits', () {
    final tree = DynamicLooseOctree<String>(halfExtent: 64.0);
    tree.insertOrUpdate(
      1,
      'FarBox',
      const OctreeAabb(OctreeVec3(-1.0, -1.0, -22.0), OctreeVec3(1.0, 1.0, -20.0)),
    );
    tree.insertOrUpdate(
      2,
      'NearBox',
      const OctreeAabb(OctreeVec3(-1.0, -1.0, -10.0), OctreeVec3(1.0, 1.0, -8.0)),
    );

    final hit = OctreeRayQuery.raycastClosest(
      tree,
      origin: OctreeVec3.zero,
      direction: const OctreeVec3(0.0, 0.0, -1.0),
    );

    expect(hit, isNotNull);
    expect(hit!.entry.item, 'NearBox');
    expect(hit.distance, closeTo(8.0, 1e-4));
  });
}
